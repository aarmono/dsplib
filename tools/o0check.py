#!/usr/bin/env python3
"""o0check.py - O0 identity checker for the dsplibs port (RS1).  Run from anywhere inside the repository.

THE CRITERION: a change to the library is "identical" when
  (1) the i386 struct layouts are unchanged, and
  (2) the -m32 -O0 assembly of every function is unchanged, with symbol names / relocation targets normalised.

What it does
  * Builds EVERY production TU of the DSP library twice - a REF tree (a git ref, default HEAD, extracted with `git archive` into a
    scratch directory) and the WORK tree (the working tree of this checkout, uncommitted edits included; or --work-ref / --work-dir).
    The compile commands are NOT re-invented: they are printed by the library's Makefile (`make -n -B BITS=32|64 PORT_OPT=-O0
    OBJDIR=<scratch> <port objects>`), i.e. the production recipe (PIN_FLAGS from flags.mk, -w -fno-stack-protector -fno-pie, C++
    -fno-exceptions -fno-rtti, v04* + V04_DEFS) with only PORT_OPT and OBJDIR overridden.  The layout build adds
    `-gdwarf-4 -fno-eliminate-unused-debug-types` to PORT_OPT (a separate object set, so the code compare uses objects built exactly
    like production apart from -O0).
  * Per object pair (paired by identical source basename):
      functions  objdump -d per FUNC symbol (aliases at the same address are one function "a=b"), normalised: addresses are
                 function-relative, branch / call targets resolved to <function+off> (".+off" = itself), relocations taken from the ELF
                 (symbol names through --map, section-relative targets resolved to the function / object / string they point at, the
                 x86-64 RIP-relative bias and the i386 in-place addend accounted for), gcc's numbered local statics (`x.3`) unnumbered.
                 Reported: added / removed / changed functions by name, unified diff for changed ones.  Header line = bind, visibility,
                 size, section kind.
      data       every allocated non-code section except .eh_frame (.data* .rodata* .bss* .init_array ...): size, flags, bytes, relocations
                 (resolved as above; for i386 REL the in-place addend is masked and moved into the relocation), and the OBJECT symbols
                 (name, bind, section, value, size).
      imports    the set of undefined symbols.
      layouts    from DWARF of the layout build: every struct / class / union with a name (or the typedef name of an anonymous one,
                 nested names qualified, typedef aliases of records included), sizeof and every member's offset / size / kind; anonymous
                 members flattened.  Compared per TU.  A type present in both trees is UNCHANGED (reported as NEUTRAL when its member
                 list differs) iff sizeof is equal and, after pairing members by (--map'ed) name: same-name members keep offset and size
                 (a kind change such as uint8_t[] -> short[] is INFO); a vanished and a new member at the same offset with the same size
                 are a rename; every other vanished / moved / resized REF member is OPAQUE (an array of 1-byte elements, or zero-size)
                 and the new members cover exactly its bytes (an opaque region refined into named members; zero-size pad64 markers are
                 free).  (RS9) A vanished non-opaque member whose offset and size equal those of a member present in the work tree is an alias
                 collapse (anonymous-union alias removed) and is free too.  Anything else is CHANGED.  Types present in only one tree (e.g. a struct that became a typedef) are INFO.
                 Changed functions also get a one-line classification: "constants only: old->new xN" (displacement / immediate changes,
                 same instructions) or "structural change".
  * Widths: -m32 -O0 (always; any difference fails) and -m64 -O0 (default on; a difference fails unless listed in the expected-diffs
    file, tools/o0check_expected.tsv; --m64-advisory makes it report-only; --no-m64 skips it).  --o2 adds the production i386 recipe at
    its own -O2 (pinned) and (RS5) the production x86-64 recipe at -O2 as ADVISORY whole-object comparisons (reported, never fatal).
  * The REF side is cached per (commit, configuration, gcc version, checker version) under the scratch directory.

Exit status: 0 = no unexpected difference, 1 = unexpected difference(s), 2 = usage / build error.

Maps (all optional; tab-separated, '#' comments):
  --map FILE        old<TAB>new  identifier renames (symbols incl. mangled names, section-name suffixes, type names, member names); applied
                    to the REF side before comparing.  DEFAULT tools/o0check_renames.tsv (an empty table to start with: add a row when a pass renames a
                    static helper or a member; the self-test passes --map '').
  --unified FILE    width<TAB>type<TAB>old sizeof<TAB>new sizeof<TAB>reason  (default tools/o0check_unified.tsv, RS6): a type that was a PREFIX declaration of an
                    object and became the whole object (several definitions of one object unified): its sizeof change old -> new is accepted at the
                    stated width (m32 or m64) and the bytes between the sizes may be new; no member may move or change, everything else is checked as usual.
                    RS21: a row whose first column is `member` (member<TAB>type<TAB>old<TAB>new<TAB>reason) renames the member `old` of the REF type `type` to `new` before
                    the layout comparison (the type-qualified counterpart of --map, needed where several name sets overlay the same bytes and the rename pairing cannot
                    choose; offset and size are still compared, so a rename that moves or resizes the member is reported).
  --expected FILE   width<TAB>kind<TAB>tu<TAB>name<TAB>reason   width m64 (or m32o2), kind func|data|import|layout, tu = object basename
                    without .o (work side) or a glob, name = mangled function / section / symbol / type name or a glob.  m32 entries are
                    rejected: the -m32 -O0 criterion has no exceptions.

Self-test: `tools/o0check.py --self-test` (REF vs REF clean; a one-line source change detected; a comment change + a local-variable rename
+ a static-function rename under --map not detected).
"""
import argparse, bisect, collections, difflib, fnmatch, hashlib, json, os, pickle, re, shutil, subprocess, sys, tempfile, time
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor

from elftools.elf.elffile import ELFFile
from elftools.elf.relocation import RelocationSection
from elftools.elf.sections import SymbolTableSection
from elftools.elf.descriptions import describe_reloc_type

VERSION = '1'
HERE = os.path.dirname(os.path.abspath(__file__))
PORTROOT = os.path.dirname(HERE)                      # the repository root
DEFAULT_REF = 'HEAD'
DEFAULT_EXPECTED = os.path.join(HERE, 'o0check_expected.tsv')
LAYOUT_DBG = '-gdwarf-4 -fno-eliminate-unused-debug-types'
ARCHIVE_PATHS = ['Makefile', 'dsplib.mk', 'flags.mk', 'sources.mk']   # + the source directories, see archive_paths()


def die(msg, code=2):
    print('o0check: ' + msg, file=sys.stderr)
    sys.exit(code)


def run(cmd, cwd=None, check=True, **kw):
    r = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, **kw)
    if check and r.returncode != 0:
        die('command failed (%d): %s\n%s' % (r.returncode, cmd if isinstance(cmd, str) else ' '.join(cmd), r.stderr[-4000:]))
    return r


def git_top():
    return run(['git', 'rev-parse', '--show-toplevel'], cwd=PORTROOT).stdout.strip()


def gcc_version():
    a = run(['gcc', '--version']).stdout.splitlines()[0]
    b = run(['g++', '--version']).stdout.splitlines()[0]
    c = run(['objdump', '--version']).stdout.splitlines()[0]
    return a + ' | ' + b + ' | ' + c


def self_hash():
    with open(os.path.abspath(__file__), 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()[:16]


# ----------------------------------------------------------------------------------------------------------------- maps / expected

def read_tsv(path, ncols_min):
    rows = []
    if not path:
        return rows
    with open(path) as f:
        for ln, line in enumerate(f, 1):
            line = line.rstrip('\n')
            if not line.strip() or line.lstrip().startswith('#'):
                continue
            cols = line.split('\t')
            if len(cols) < ncols_min:
                die('%s:%d: expected >= %d tab-separated columns' % (path, ln, ncols_min))
            rows.append((ln, cols))
    return rows


class Expected:
    def __init__(self, path):
        self.rows = []
        for ln, c in read_tsv(path, 5):
            width, kind, tu, name, reason = c[0], c[1], c[2], c[3], '\t'.join(c[4:])
            if width not in ('m64', 'm32o2', 'm64o2'):
                die('%s:%d: width %r not allowed (only m64 / m32o2 / m64o2: the -m32 -O0 criterion has no exceptions)' % (path, ln, width))
            if kind not in ('func', 'data', 'import', 'layout'):
                die('%s:%d: kind %r unknown' % (path, ln, kind))
            if not reason.strip():
                die('%s:%d: an expected difference needs a reason' % (path, ln))
            self.rows.append(dict(ln=ln, width=width, kind=kind, tu=tu, name=name, reason=reason, hits=0))

    def match(self, width, kind, tu, name):
        tus = (tu, TU_REF_NAME.get(tu, tu))      # RS3: an entry names the TU by its REF (canonical) name or by its work name
        for r in self.rows:
            if r['width'] == width and r['kind'] == kind and any(fnmatch.fnmatchcase(t, r['tu']) for t in tus) and fnmatch.fnmatchcase(name, r['name']):
                r['hits'] += 1
                return r
        return None


# ----------------------------------------------------------------------------------------------------------------- trees / builds

def archive_paths(top, sha):
    """What `git archive` needs of commit sha to build the library: ARCHIVE_PATHS and the directories of the include root and of every
    listed source (read from the sources.mk of that commit)."""
    paths = list(ARCHIVE_PATHS)
    r = run(['git', 'cat-file', '-p', sha + ':sources.mk'], cwd=top)
    sys.path.insert(0, HERE)
    import port_objs
    mk = os.path.join(tempfile.mkdtemp(prefix='o0check-mk-'), 'sources.mk')
    open(mk, 'w').write(r.stdout)
    v = port_objs.parse(mk)
    shutil.rmtree(os.path.dirname(mk), ignore_errors=True)
    dirs = {v['DSP_INCDIR'][0]} | {os.path.dirname(s) for s in v['DSP_SRCS']}
    dirs = sorted(d for d in dirs if not any(d != e and d.startswith(e + '/') for e in dirs))
    return paths + dirs


def extract_ref(top, ref, scratch):
    sha = run(['git', 'rev-parse', '--verify', ref + '^{commit}'], cwd=top).stdout.strip()
    d = os.path.join(scratch, 'ref-' + sha[:16])
    tree = os.path.join(d, 'tree')
    if not os.path.isfile(os.path.join(tree, '.complete')):
        shutil.rmtree(d, ignore_errors=True)
        os.makedirs(tree)
        p1 = subprocess.Popen(['git', 'archive', sha] + archive_paths(top, sha), cwd=top, stdout=subprocess.PIPE)
        r = subprocess.run(['tar', '-x', '-C', tree], stdin=p1.stdout)
        p1.stdout.close()
        if p1.wait() != 0 or r.returncode != 0:
            die('git archive %s failed' % ref)
        open(os.path.join(tree, '.complete'), 'w').close()
    os.utime(d, None)       # "used now": trim_scratch keeps the recently used refs
    return sha, d, tree


def trim_scratch(scratch, keep_n, keep_dirs=()):
    """Bound the scratch directory (the ref cache grows by ~0.5 GB per distinct REF commit): keep the KEEP_N most recently used
    ref-* directories (use = mtime, touched by extract_ref), never one used in the last 2 hours (another run may be using it) or in
    KEEP_DIRS; remove work-* / selftest-* leftovers of crashed runs older than a day.  keep_n < 0 disables the trim."""
    if keep_n < 0:
        return
    now = time.time()
    refs = []
    for n in os.listdir(scratch):
        d = os.path.join(scratch, n)
        try:
            age = now - os.stat(d).st_mtime
        except OSError:
            continue
        if n.startswith('ref-') and os.path.isdir(d):
            refs.append((age, d))
        elif (n.startswith('work-') or n.startswith('selftest-')) and age > 86400:
            shutil.rmtree(d, ignore_errors=True) if os.path.isdir(d) else os.unlink(d)
    refs.sort()
    for age, d in refs[keep_n:]:
        if age > 7200 and d not in keep_dirs:
            shutil.rmtree(d, ignore_errors=True)


def build_dir(tree):
    """The directory the make / compile commands run in: the root of the extracted tree."""
    if not (os.path.isfile(os.path.join(tree, 'Makefile')) and os.path.isfile(os.path.join(tree, 'dsplib.mk'))):
        die('%s has no Makefile / dsplib.mk' % tree)
    return tree


def tu_name(src):
    """The TU label: the source path as the Makefile prints it (./src/x.c), normalised."""
    return os.path.normpath(src)


def port_commands(tree, bits, opt, objdir):
    """The compile command of every port object, as the Makefile prints it (make -n), cwd = build_dir(tree).  Returns (obj, TU label, command)."""
    sl = build_dir(tree)
    mk = os.path.join(objdir, '.o0check-list.mk')
    os.makedirs(objdir, exist_ok=True)
    with open(mk, 'w') as f:
        f.write('o0check-list:\n\t@echo $(PORT_OBJS)\n')
    common = ['BITS=%s' % bits, 'OBJDIR=%s' % objdir, 'PORT_OPT=%s' % opt]
    objs = run(['make', '-s', '--no-print-directory', '-f', 'Makefile', '-f', mk] + common + ['o0check-list'], cwd=sl).stdout.split()
    if not objs:
        die('no port objects listed by %s/Makefile' % sl)
    out = run(['make', '-n', '-B', '--no-print-directory'] + common + objs, cwd=sl).stdout.splitlines()
    cmds = {}
    for line in out:
        line = line.strip()
        if not line or line.startswith('mkdir '):
            continue
        m = re.search(r'\s-c\s+(\S+)\s+-o\s+(\S+)$', line)
        if not m:
            die('unexpected make -n line: ' + line)
        cmds[m.group(2)] = (tu_name(m.group(1)), line)
    missing = [o for o in objs if o not in cmds]
    if missing:
        die('make -n printed no command for %s' % missing[:3])
    return [(o, cmds[o][0], cmds[o][1]) for o in objs]


def build(tree, bits, opt, objdir, jobs):
    shutil.rmtree(objdir, ignore_errors=True)
    lst = port_commands(tree, bits, opt, objdir)
    sl = build_dir(tree)
    for o, _, _ in lst:
        os.makedirs(os.path.dirname(o), exist_ok=True)

    def one(item):
        o, src, cmd = item
        r = subprocess.run(cmd, shell=True, cwd=sl, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        return (o, src, cmd, r.returncode, r.stderr)

    with ThreadPoolExecutor(jobs) as ex:
        res = list(ex.map(one, lst))
    bad = [r for r in res if r[3] != 0]
    if bad:
        for o, src, cmd, rc, err in bad[:5]:
            print('COMPILE FAIL %s\n  %s\n%s' % (src, cmd, err[-3000:]), file=sys.stderr)
        die('%d compile failure(s) in %s' % (len(bad), tree))
    return [(o, src) for o, src, _, _, _ in res], lst[0][2] if lst else ''


# ----------------------------------------------------------------------------------------------------------------- object extraction

INSN_RE = re.compile(r'^\s*([0-9a-f]+):\t(.*)$')
TARGET_RE = re.compile(r'\b([0-9a-f]+) <([^>]*)>')
NUMSTATIC_RE = re.compile(r'^(.+)\.(\d+)$')
PCREL = ('R_386_PC32', 'R_386_PLT32', 'R_X86_64_PC32', 'R_X86_64_PLT32', 'R_X86_64_PC64')


def is_exec(sec):
    return bool(sec['sh_flags'] & 0x4)


def is_alloc(sec):
    return bool(sec['sh_flags'] & 0x2)


class Obj:
    """Everything the comparison needs from one object, names NOT yet mapped (mapping happens at compare time)."""

    def __init__(self, path):
        self.path = path
        with open(path, 'rb') as f:
            elf = ELFFile(f)
            self.elfclass = elf.elfclass
            self.secs = list(elf.iter_sections())
            self.secname = {i: s.name for i, s in enumerate(self.secs)}
            self.secdata = {}
            for i, s in enumerate(self.secs):
                if is_alloc(s) and s['sh_type'] != 'SHT_NOBITS':
                    self.secdata[i] = s.data()
            symtab = elf.get_section_by_name('.symtab')
            self.syms = []
            for s in symtab.iter_symbols():
                self.syms.append(dict(name=s.name, value=s['st_value'], size=s['st_size'], bind=s['st_info']['bind'][4:],
                                      type=s['st_info']['type'][4:], vis=s['st_other']['visibility'][4:], shndx=s['st_shndx']))
            self.relocs = collections.defaultdict(list)       # target section index -> [(off, type, symidx, addend or None)]
            for s in self.secs:
                if isinstance(s, RelocationSection):
                    tgt = s['sh_info']
                    rela = s.is_RELA()
                    for r in s.iter_relocations():
                        self.relocs[tgt].append((r['r_offset'], describe_reloc_type(r['r_info_type'], elf), r['r_info_sym'],
                                                 r['r_addend'] if rela else None))
            for k in self.relocs:
                self.relocs[k].sort()
            self.rela = any(s.is_RELA() for s in self.secs if isinstance(s, RelocationSection))
        self.secidx = {n: i for i, n in self.secname.items()}
        # function / object symbols per section
        self.funcs = collections.defaultdict(list)
        self.objs = collections.defaultdict(list)
        for i, s in enumerate(self.syms):
            if not isinstance(s['shndx'], int):
                continue
            if s['type'] == 'FUNC':
                self.funcs[s['shndx']].append(i)
            elif s['type'] in ('OBJECT', 'TLS'):
                self.objs[s['shndx']].append(i)
        self.dis = self._disasm()

    def _disasm(self):
        out = run(['objdump', '-d', '-z', '-w', '--no-show-raw-insn', self.path]).stdout
        dis = collections.defaultdict(list)
        cur = None
        for line in out.splitlines():
            if line.startswith('Disassembly of section '):
                cur = self.secidx.get(line[len('Disassembly of section '):-1])
                continue
            m = INSN_RE.match(line)
            if m and cur is not None:
                dis[cur].append((int(m.group(1), 16), m.group(2)))
        return dis


def addend_of(obj, secidx, off, rtype, addend):
    if addend is not None:
        return addend
    data = obj.secdata.get(secidx)
    if data is None or rtype in ('R_386_NONE',):
        return 0
    v = int.from_bytes(data[off:off + 4], 'little')
    return v - (1 << 32) if v & 0x80000000 else v


class Norm:
    """Name normalisation of one side: the identifier map (REF side only) + gcc's numbered statics."""

    def __init__(self, idmap):
        self.idmap = idmap or {}

    def name(self, n):
        n = self.idmap.get(n, n)
        m = NUMSTATIC_RE.match(n)
        if m and not n.startswith('.'):
            n = m.group(1) + '.N'
        return n

    def sec(self, n):
        for k, v in self.idmap.items():
            if n.endswith('.' + k):
                return n[:-len(k)] + v
        return n


def func_groups(obj, norm):
    """section index -> sorted [(start, end, canonical name, symbol indices)] (aliases merged)."""
    res = {}
    for sx, idxs in obj.funcs.items():
        by = collections.defaultdict(list)
        for i in idxs:
            by[obj.syms[i]['value']].append(i)
        lst = []
        for start, ii in sorted(by.items()):
            size = max(obj.syms[i]['size'] for i in ii)
            name = '='.join(sorted(norm.name(obj.syms[i]['name']) for i in ii))
            lst.append((start, start + size, name, ii))
        res[sx] = lst
    return res


def find_in(groups, off):
    i = bisect.bisect_right([g[0] for g in groups], off) - 1
    if i >= 0 and (groups[i][0] <= off < groups[i][1] or groups[i][0] == off):
        return groups[i]
    return None


def obj_at(obj, norm, sx, off):
    best = None
    for i in obj.objs.get(sx, []):
        s = obj.syms[i]
        if s['value'] <= off < s['value'] + max(s['size'], 1):
            best = s
            break
    if best is not None:
        return norm.name(best['name']) + ('+0x%x' % (off - best['value']) if off != best['value'] else '')
    return None


def resolve(obj, norm, fg, symidx, target_off_extra):
    """Name of a relocation target.  For a section symbol the section-relative offset is resolved to function / object / string."""
    s = obj.syms[symidx]
    if s['type'] == 'SECTION' or (s['name'] == '' and isinstance(s['shndx'], int)):
        sx = s['shndx']
        sec = obj.secs[sx]
        sname = norm.sec(sec.name)
        off = s['value'] + target_off_extra
        if is_exec(sec):
            g = find_in(fg.get(sx, []), off)
            if g:
                return '%s%s' % (g[2], '+0x%x' % (off - g[0]) if off != g[0] else '')
        else:
            o = obj_at(obj, norm, sx, off)
            if o:
                return o
            if sec['sh_flags'] & 0x20 and sx in obj.secdata:          # SHF_STRINGS: the string itself
                data = obj.secdata[sx]
                if 0 <= off < len(data):
                    end = data.find(b'\0', off)
                    return '%s:str%r' % (sname.split('.str')[0], data[off:end if end >= 0 else len(data)].decode('latin-1'))
        return '%s%+#x' % (sname, off) if off else sname
    n = norm.name(s['name']) if s['name'] else '?sym%d' % symidx
    return n + ('%+#x' % target_off_extra if target_off_extra else '')


def norm_functions(obj, norm):
    """canonical function name -> list of normalised lines."""
    fg = func_groups(obj, norm)
    out = {}
    for sx, groups in fg.items():
        insns = obj.dis.get(sx, [])
        iaddr = [a for a, _ in insns]
        rel = obj.relocs.get(sx, [])
        raddr = [r[0] for r in rel]
        secname = norm.sec(obj.secs[sx].name)
        for gi, (start, end, name, ii) in enumerate(groups):
            syms = [obj.syms[i] for i in ii]
            if secname == '.text':
                sk = '.text'
            elif any(secname == '.text.' + n for n in name.split('=')):
                sk = '.text.<own name>'
            else:
                sk = secname
            lines = ['FUNC %s %s size=%#x sec=%s' % ('/'.join(sorted(set(s['bind'] for s in syms))),
                                                   '/'.join(sorted(set(s['vis'] for s in syms))), end - start, sk)]
            body = insns[bisect.bisect_left(iaddr, start):bisect.bisect_left(iaddr, max(end, start + 1))]
            for k, (a, t) in enumerate(body):
                nxt = body[k + 1][0] if k + 1 < len(body) else max(end, a + 1)
                rs = rel[bisect.bisect_left(raddr, a):bisect.bisect_left(raddr, nxt)]
                text = t
                rtxt = []
                if rs:
                    text = TARGET_RE.sub('<R>', text)
                    text = re.sub(r'\s+#.*$', '', text)
                    for (ro, rtype, symidx, add) in rs:
                        A = addend_of(obj, sx, ro, rtype, add)
                        extra = A + (nxt - ro) if rtype in PCREL else A
                        tgt = resolve(obj, norm, fg, symidx, extra)
                        if add is None and rtype == 'R_386_32' and A:
                            # i386 REL: the addend is printed inside the operand; replace it by the resolved target
                            tok = '%#x' % (A & 0xffffffff)
                            text = re.sub(r'(?<![0-9a-fx])' + re.escape(tok) + r'(?![0-9a-f])', lambda m: '{' + tgt + '}', text, count=1)
                        rtxt.append('%s@+%d %s' % (rtype, ro - a, tgt))
                else:
                    def sub(m):
                        ta = int(m.group(1), 16)
                        if start <= ta < end:
                            return '<.+%#x>' % (ta - start)
                        g = find_in(groups, ta)
                        if g:
                            return '<%s%s>' % (g[2], '+%#x' % (ta - g[0]) if ta != g[0] else '')
                        return '<?%s>' % m.group(2)
                    text = TARGET_RE.sub(sub, text)
                text = re.sub(r'\s+', ' ', text).strip()
                lines.append('%x: %s%s' % (a - start, text, ('    {' + '; '.join(rtxt) + '}') if rtxt else ''))
            key = name
            while key in out:
                key += '#2'
            out[key] = lines
    return out


SKIP_DATA = ('.eh_frame', '.comment', '.note.GNU-stack', '.group')


def norm_data(obj, norm):
    """mapped section name -> list of normalised lines (alloc, non-exec, not .eh_frame)."""
    fg = func_groups(obj, norm)
    out = {}
    for sx, sec in enumerate(obj.secs):
        if not is_alloc(sec) or is_exec(sec) or sec.name in SKIP_DATA or sec.name.startswith('.note'):
            continue
        name = norm.sec(sec.name)
        lines = ['SECTION %s type=%s flags=%#x size=%#x align=%d' % (name, sec['sh_type'][4:], sec['sh_flags'], sec['sh_size'],
                                                                     sec['sh_addralign'])]
        for i in sorted(obj.objs.get(sx, []), key=lambda i: (obj.syms[i]['value'], obj.syms[i]['name'])):
            s = obj.syms[i]
            lines.append('SYM %s %s %s @%#x size=%#x' % (s['type'], s['bind'], norm.name(s['name']), s['value'], s['size']))
        data = bytearray(obj.secdata.get(sx, b''))
        rl = []
        for (ro, rtype, symidx, add) in obj.relocs.get(sx, []):
            A = addend_of(obj, sx, ro, rtype, add)
            if add is None:
                data[ro:ro + 4] = b'\xee' * 4                 # masked: the addend is in the relocation line
            extra = A + 4 if rtype in PCREL else A
            rl.append('RELOC +%#x %s %s' % (ro, rtype, resolve(obj, norm, fg, symidx, extra)))
        for o in range(0, len(data), 16):
            lines.append('%04x: %s' % (o, data[o:o + 16].hex()))
        lines.extend(rl)
        out[name] = lines
    return out


def norm_imports(obj, norm):
    return sorted(set(norm.name(s['name']) for s in obj.syms if s['shndx'] == 'SHN_UNDEF' and s['name']))


# ----------------------------------------------------------------------------------------------------------------- DWARF layouts

TAGS = ('DW_TAG_structure_type', 'DW_TAG_class_type', 'DW_TAG_union_type')


def _attr(d, n):
    a = d.attributes.get(n)
    return a.value if a is not None else None


def _name(d):
    v = _attr(d, 'DW_AT_name')
    return v.decode() if isinstance(v, bytes) else v


def _moff(m):
    v = _attr(m, 'DW_AT_data_member_location')
    if v is None:
        return 0
    if isinstance(v, int):
        return v
    if isinstance(v, list) and len(v) >= 2 and v[0] == 0x23:
        r, s, i = 0, 0, 1
        while True:
            b = v[i]; r |= (b & 0x7f) << s; s += 7; i += 1
            if not b & 0x80:
                break
        return r
    return None


def _strip(t):
    while t is not None and t.tag in ('DW_TAG_typedef', 'DW_TAG_const_type', 'DW_TAG_volatile_type'):
        t = t.get_DIE_from_attribute('DW_AT_type') if 'DW_AT_type' in t.attributes else None
    return t


def _kind(t):
    t = _strip(t)
    if t is None:
        return 'void'
    if t.tag in ('DW_TAG_pointer_type', 'DW_TAG_reference_type', 'DW_TAG_rvalue_reference_type', 'DW_TAG_ptr_to_member_type'):
        return 'ptr'
    if t.tag == 'DW_TAG_array_type':
        return 'arr:' + _kind(t.get_DIE_from_attribute('DW_AT_type'))
    if t.tag in TAGS:
        return 'rec:' + (_name(t) or '?')
    if t.tag == 'DW_TAG_base_type':
        return _name(t)
    if t.tag == 'DW_TAG_enumeration_type':
        return 'enum'
    return t.tag[7:]


def _tsize(st, ptrsize):
    if st is None:
        return None
    if st.tag == 'DW_TAG_array_type':
        es = _tsize(_strip(st.get_DIE_from_attribute('DW_AT_type')), ptrsize)
        if es is None:
            return None
        n = 1
        for c in st.iter_children():
            if c.tag == 'DW_TAG_subrange_type':
                if 'DW_AT_count' in c.attributes:
                    n *= c.attributes['DW_AT_count'].value
                elif 'DW_AT_upper_bound' in c.attributes:
                    ub = c.attributes['DW_AT_upper_bound'].value
                    n *= (ub + 1) if isinstance(ub, int) and ub < 0x7fffffff else 0
                else:
                    n *= 0
        return es * n
    if st.tag in ('DW_TAG_pointer_type', 'DW_TAG_reference_type', 'DW_TAG_rvalue_reference_type'):
        return _attr(st, 'DW_AT_byte_size') or ptrsize
    return _attr(st, 'DW_AT_byte_size')


def _members(d, base, out, ptrsize, prefix=''):
    for m in d.iter_children():
        if m.tag not in ('DW_TAG_member', 'DW_TAG_inheritance'):
            continue
        if _attr(m, 'DW_AT_external') or 'DW_AT_declaration' in m.attributes:
            continue
        o = _moff(m)
        if o is None:
            continue
        t = m.get_DIE_from_attribute('DW_AT_type') if 'DW_AT_type' in m.attributes else None
        st = _strip(t)
        n = _name(m)
        if m.tag == 'DW_TAG_inheritance':
            out['<base:%s>' % (_name(st) if st is not None else '?')] = [base + o, 'base', _tsize(st, ptrsize)]
            continue
        bits = _attr(m, 'DW_AT_bit_size')
        if n is None:
            if st is not None and st.tag in TAGS:
                _members(st, base + o, out, ptrsize, prefix)
            continue
        k = _kind(t)
        if bits is not None:
            bo = _attr(m, 'DW_AT_data_bit_offset')
            k += ':%d@%s' % (bits, bo if bo is not None else _attr(m, 'DW_AT_bit_offset'))
        out[prefix + n] = [base + o, k, _tsize(st, ptrsize)]


def dwarf_layouts(path):
    types = {}
    with open(path, 'rb') as f:
        elf = ELFFile(f)
        if not elf.has_dwarf_info():
            return types
        ptrsize = 8 if elf.elfclass == 64 else 4
        dw = elf.get_dwarf_info()
        for cu in dw.iter_CUs():
            anon_td = {}
            tds = []
            for d in cu.iter_DIEs():
                if d.tag == 'DW_TAG_typedef' and 'DW_AT_type' in d.attributes:
                    t = d.get_DIE_from_attribute('DW_AT_type')
                    tds.append((d, t))
                    if t.tag in TAGS and _name(t) is None:
                        anon_td.setdefault(t.offset, _name(d))
            key_of = {}
            for d in cu.iter_DIEs():
                if d.tag not in TAGS or 'DW_AT_declaration' in d.attributes:
                    continue
                sz = _attr(d, 'DW_AT_byte_size')
                if sz is None:
                    continue
                n = _name(d) or anon_td.get(d.offset)
                if n is None:
                    continue
                p = d.get_parent()
                while p is not None and p.tag in TAGS + ('DW_TAG_namespace',):
                    pn = _name(p) or anon_td.get(p.offset)
                    if pn:
                        n = pn + '::' + n
                    p = p.get_parent()
                mem = {}
                _members(d, 0, mem, ptrsize)
                base = n
                k = 2
                while n in types and (types[n]['size'] != sz or types[n]['members'] != mem):
                    n = '%s#%d' % (base, k); k += 1
                types[n] = {'size': sz, 'members': mem}
                key_of[d.offset] = n
            for d, t in tds:                                    # typedef aliases of named records
                n = _name(d)
                st = _strip(t)
                if n and st is not None and st.offset in key_of and n not in types:
                    p = d.get_parent()
                    while p is not None and p.tag in TAGS + ('DW_TAG_namespace',):
                        pn = _name(p) or anon_td.get(p.offset)
                        if pn:
                            n = pn + '::' + n
                        p = p.get_parent()
                    if n not in types:
                        src = types[key_of[st.offset]]
                        types[n] = {'size': src['size'], 'members': src['members'], 'alias_of': key_of[st.offset]}
    return types


# ----------------------------------------------------------------------------------------------------------------- extraction driver

def extract_one(args):
    path, kind, idmap = args
    norm = Norm(idmap)
    if kind == 'layout':
        return dwarf_layouts(path)
    o = Obj(path)
    return dict(funcs=norm_functions(o, norm), data=norm_data(o, norm), imports=norm_imports(o, norm))


def extract_all(objs, kind, idmap, jobs):
    with ProcessPoolExecutor(jobs) as ex:
        res = list(ex.map(extract_one, [(o, kind, idmap) for o, _ in objs], chunksize=4))
    return res


def tu_key(src):
    return os.path.basename(src)


def collect(tree, cfgname, bits, opt, scratch_side, jobs, idmap, cache_file=None):
    if cache_file and os.path.isfile(cache_file):
        with open(cache_file, 'rb') as f:
            return pickle.load(f)
    objdir = os.path.join(scratch_side, cfgname)
    t0 = time.time()
    objs, sample = build(tree, bits, opt, objdir, jobs)
    t1 = time.time()
    kind = 'layout' if LAYOUT_DBG in opt else 'code'
    res = extract_all(objs, kind, idmap if kind == 'code' else None, jobs)
    t2 = time.time()
    out = dict(sample=sample, build_s=t1 - t0, extract_s=t2 - t1, tus={})
    for (o, src), r in zip(objs, res):
        out['tus'][src] = r
    if cache_file:
        tmp = cache_file + '.tmp'
        with open(tmp, 'wb') as f:
            pickle.dump(out, f)
        os.replace(tmp, cache_file)
    shutil.rmtree(objdir, ignore_errors=True)
    return out


# ----------------------------------------------------------------------------------------------------------------- comparison

class Report:
    def __init__(self, maxdiff):
        self.lines = []
        self.maxdiff = maxdiff
        self.unexpected = collections.Counter()
        self.expected = collections.Counter()
        self.info = collections.Counter()

    def p(self, s=''):
        self.lines.append(s)


def udiff(a, b, maxdiff, la='ref', lb='work'):
    d = list(difflib.unified_diff(a, b, la, lb, n=2, lineterm=''))
    if len(d) > maxdiff:
        d = d[:maxdiff] + ['... (%d more diff lines)' % (len(d) - maxdiff)]
    return d


TU_REF_NAME = {}    # TU label (work stem) -> the REF stem (identical to the label while TUs are paired by basename)


def pair_tus(ref_tus, work_tus):
    """[(tu label, ref src, work src)], unpaired lists: TUs are paired by identical source basename."""
    wb = {}
    for w in work_tus:
        wb.setdefault(os.path.basename(w), []).append(w)
    pairs, ronly = [], []
    used = set()
    for r in sorted(ref_tus):
        c = wb.get(os.path.basename(r), [])
        tgt = c[0] if len(c) == 1 else None
        if tgt is None or tgt in used:
            ronly.append(r)
        else:
            used.add(tgt)
            pairs.append((os.path.splitext(os.path.basename(tgt))[0], r, tgt))
            TU_REF_NAME[pairs[-1][0]] = os.path.splitext(os.path.basename(r))[0]
    wonly = [w for w in sorted(work_tus) if w not in used]
    return pairs, ronly, wonly


def classify(rep, exp, width, fatal, kind, tu, name, label):
    if not fatal:
        rep.info[kind] += 1
        return 'ADVISORY'
    e = exp.match(width, kind, tu, name) if width != 'm32' else None
    if e:
        rep.expected[kind] += 1
        return 'EXPECTED (%s)' % e['reason'][:100]
    rep.unexpected[kind] += 1
    return 'UNEXPECTED'


HEX_RE = re.compile(r'-?0x[0-9a-f]+|\b[0-9a-f]+(?=: )')


def constant_delta(a, b):
    """If two normalised function bodies differ only in numeric constants (displacements / immediates), return a Counter of the
    (old -> new) operand-constant changes (line addresses and in-function branch targets excluded) and whether instruction
    addresses moved; else None."""
    if len(a) != len(b):
        return None
    pairs = collections.Counter()
    moved = False
    for x, y in zip(a, b):
        if x == y:
            continue
        if HEX_RE.sub('#', x) != HEX_RE.sub('#', y):
            return None
        ax, ay = x.split(': ', 1), y.split(': ', 1)
        if len(ax) == 2 and len(ay) == 2:
            if ax[0] != ay[0]:
                moved = True
            x, y = ax[1], ay[1]
        x2 = re.sub(r'<\.\+0x[0-9a-f]+>', '<.>', x)
        y2 = re.sub(r'<\.\+0x[0-9a-f]+>', '<.>', y)
        if x != y and x2 == y2:
            moved = True
        for u, v in zip(HEX_RE.findall(x2), HEX_RE.findall(y2)):
            if u != v:
                pairs[(u, v)] += 1
    return pairs, moved


def compare_code(rep, exp, width, fatal, ref, work, pairs, demangle):
    nf = collections.Counter()
    for tu, rs, ws in pairs:
        R, W = ref['tus'][rs], work['tus'][ws]
        hdr_done = [False]

        def head():
            if not hdr_done[0]:
                rep.p('  -- %s%s' % (tu, '' if os.path.basename(rs) == os.path.basename(ws) else '  (ref %s)' % rs))
                hdr_done[0] = True
        rf, wf = R['funcs'], W['funcs']
        for n in sorted(set(rf) | set(wf)):
            nf['total'] += 1
            if n in rf and n in wf:
                if rf[n] == wf[n]:
                    nf['identical'] += 1
                    continue
                nf['changed'] += 1
                v = classify(rep, exp, width, fatal, 'func', tu, n, 'changed')
                head()
                rep.p('     func CHANGED %s %s  [%s]' % (n, demangle(n), v))
                cd = constant_delta(rf[n], wf[n])
                if cd is None:
                    rep.p('        (structural change: instructions / operands other than constants differ)')
                else:
                    rep.p('        (constants only%s: %s)' % (', instruction addresses moved' if cd[1] else '',
                                                             ', '.join('%s->%s x%d' % (u, v, c) for (u, v), c in sorted(cd[0].items()))))
                if v.startswith('UNEXPECTED') or v == 'ADVISORY' or rep.verbose_expected:
                    rep.lines.extend('        ' + l for l in udiff(rf[n], wf[n], rep.maxdiff))
            else:
                what = 'REMOVED' if n in rf else 'ADDED'
                nf[what.lower()] += 1
                v = classify(rep, exp, width, fatal, 'func', tu, n, what)
                head()
                rep.p('     func %s %s %s  [%s]' % (what, n, demangle(n), v))
        rd, wd = R['data'], W['data']
        for s in sorted(set(rd) | set(wd)):
            nf['sections'] += 1
            if rd.get(s) == wd.get(s):
                continue
            nf['data_diff'] += 1
            v = classify(rep, exp, width, fatal, 'data', tu, s, '')
            head()
            rep.p('     data %s %s  [%s]' % ('CHANGED' if s in rd and s in wd else ('REMOVED' if s in rd else 'ADDED'), s, v))
            rep.lines.extend('        ' + l for l in udiff(rd.get(s, []), wd.get(s, []), rep.maxdiff))
        ri, wi = set(R['imports']), set(W['imports'])
        for n in sorted(ri ^ wi):
            v = classify(rep, exp, width, fatal, 'import', tu, n, '')
            head()
            rep.p('     import %s %s  [%s]' % ('REMOVED' if n in ri else 'ADDED', n, v))
    return nf


OPAQUE_KINDS = ('arr:unsigned char', 'arr:char', 'arr:signed char')


def layout_verdict(a, b, idmap, grow=None, mren=None):
    """None if equal; ('INFO', text) if the layout is unchanged; ('CHANGE', text) otherwise.
    Unchanged = same sizeof, and after pairing members by (mapped) name:
      * a member with the same name, offset and size is unchanged (a different kind, e.g. int -> unsigned, is reported as INFO);
      * a REF-only and a WORK-only member with the same offset and size are a rename (INFO);
      * every other REF-side member that disappeared / moved / resized must be OPAQUE (an array of 1-byte elements, or zero-size),
        and the remaining WORK-side members must cover exactly the same bytes (an opaque region refined into named members, INFO).
    RS6: `grow` = (old sizeof, new sizeof) for a type listed in tools/o0check_unified.tsv: the sizeof change itself is then accepted, and the bytes between the
    old and the new size may be new; every other condition above stays (no member may move, shrink or change)."""
    am = {idmap.get(k, k): v for k, v in a['members'].items()}
    if mren:      # RS21: per-type member renames of tools/o0check_unified.tsv (rows `member<TAB>type<TAB>old<TAB>new<TAB>reason`)
        am = {mren.get(k, k): v for k, v in am.items()}
    bm = b['members']
    if a['size'] == b['size'] and am == bm:
        return None
    grown = set()
    if grow and grow == (a['size'], b['size']):
        grown = set(range(a['size'], b['size']))
    ch_a = {k: v for k, v in am.items() if bm.get(k) is None or bm[k][0] != v[0] or bm[k][2] != v[2]}
    ch_b = {k: v for k, v in bm.items() if am.get(k) is None or am[k][0] != v[0] or am[k][2] != v[2]}
    kinds = sorted(k for k in am if k in bm and am[k][0] == bm[k][0] and am[k][2] == bm[k][2] and am[k][1] != bm[k][1])
    renames = []
    for ka in sorted(ch_a):
        if ka in bm:
            continue
        for kb in sorted(ch_b):
            if kb not in am and ch_b[kb][0] == ch_a[ka][0] and ch_b[kb][2] == ch_a[ka][2]:
                renames.append((ka, kb))
                break
    for ka, kb in renames:
        ch_a.pop(ka, None)
        ch_b.pop(kb, None)
    # RS9: an ALIAS collapse.  A REF-only member (an anonymous-union alias, `union { int w1218; int ans_org; }` -> `int ans_org;`) whose offset and size equal those of a WORK
    # member (the surviving name, unchanged or renamed) adds and removes no bytes: the bytes stay named by that member.  Opaque (byte array) and zero-size members are never
    # treated this way (they are refined into several members, which the coverage rule below checks).
    wsame = {(v[0], v[2]) for v in bm.values() if v[2]}
    # ... or one whose bytes are all named by members that are the same on both sides (`union { int w00; FaxSgd *sgd; }` -> `FaxSgd *sgd;`: the int is 4 bytes at -m32 and -m64,
    # the pointer 4 / 8; `union { struct { short s08, s0a; }; int w8; }` -> `short s08, s0a;`)
    same_bytes = set()
    for k2, v2 in am.items():
        if k2 in bm and bm[k2][0] == v2[0] and bm[k2][2] == v2[2] and v2[2]:
            same_bytes.update(range(v2[0], v2[0] + v2[2]))
    aliases = [ka for ka in sorted(ch_a) if ka not in bm and ch_a[ka][2] and ch_a[ka][1] not in OPAQUE_KINDS
               and ((ch_a[ka][0], ch_a[ka][2]) in wsame or all(b2 in same_bytes for b2 in range(ch_a[ka][0], ch_a[ka][0] + ch_a[ka][2])))]
    for ka in aliases:
        ch_a.pop(ka, None)

    def cover(d):
        s = set()
        for k, (off, kind, sz) in d.items():
            if sz is None:
                return None
            s.update(range(off, off + sz))
        return s
    ca, cb = cover(ch_a), cover(ch_b)
    # RS5: a WORK-only member may also lie over bytes of members that exist unchanged on both sides: an overlay of one object by several
    # descriptions (the V.34 object is ONE struct for the plain-C tiers and the C++ ports: flat members and `rx` / `info0a_` / `retrain_` record members
    # share their bytes through anonymous unions; a zero-size marker names a region).  Such members add no bytes, so they are no coverage mismatch.
    unch = set()
    for k, v in am.items():
        if k in bm and bm[k][0] == v[0] and bm[k][2] == v[2] and v[2]:
            unch.update(range(v[0], v[0] + v[2]))
    # ... and the bytes of REF `pad*` members that vanished may be compiler padding on the WORK side (the x86-64 `pad_NNNN_` byte arrays that tracked the widening of
    # pointer slots are gone; the real pointer members make the compiler insert exactly that padding)
    allwork = set()
    for v in bm.values():
        if v[2]:
            allwork.update(range(v[0], v[0] + v[2]))
    padding = set()      # only REF members named pad* (explicit padding arrays) may turn into compiler padding
    for k, v in ch_a.items():
        if k.startswith('pad') and k not in bm and v[2]:
            padding.update(range(v[0], v[0] + v[2]))
    gaps = (set(range(b['size'])) - allwork) & padding
    # (a record member that only overlays other WORK members, e.g. `rx`, covers the alignment padding inside it: it is left out of the "new bytes" side)
    wrecs = [k for k, v in ch_b.items() if k not in am and v[1].startswith('rec') and v[2] and
             any(k2 != k and s2 and v[0] <= o2 and o2 + s2 <= v[0] + v[2] for k2, (o2, _, s2) in bm.items())]
    cb_new = cover({k: v for k, v in ch_b.items() if k not in wrecs})
    overlay = sorted(k for k, v in ch_b.items() if k not in am and v[2] and all(b in unch for b in range(v[0], v[0] + v[2])))
    nonopaque = [k for k, v in ch_a.items() if v[2] and v[1] not in OPAQUE_KINDS]
    txt = []
    if a['size'] != b['size']:
        txt.append('sizeof %#x -> %#x' % (a['size'], b['size']))
    for k in sorted(set(ch_a) | set(ch_b), key=lambda k: ((ch_a.get(k) or ch_b.get(k))[0], k)):
        txt.append('%s: %s -> %s%s' % (k, ch_a.get(k) and '@%#x %s size %s' % tuple(ch_a[k]) or '-',
                                       ch_b.get(k) and '@%#x %s size %s' % tuple(ch_b[k]) or '-',
                                       '   [NOT OPAQUE]' if k in nonopaque else ''))
    for ka, kb in renames:
        txt.append('%s -> %s: renamed (@%#x size %s; kind %s -> %s)' % (ka, kb, am[ka][0], am[ka][2], am[ka][1], bm[kb][1]))
    for ka in aliases:
        txt.append('%s: alias removed (@%#x size %s; the bytes keep their other name)' % (ka, am[ka][0], am[ka][2]))
    for k in kinds:
        txt.append('%s: kind %s -> %s (same offset / size)' % (k, am[k][1], bm[k][1]))
    if overlay:
        txt.append('%d WORK-only member(s) (%s ...) lie over bytes of members that exist unchanged on both sides (overlays, no new bytes)' % (len(overlay), overlay[0]))
    if (a['size'] != b['size'] and not grown) or ca is None or cb is None or (ca - cb - gaps) or (cb_new - ca - unch - grown) or nonopaque:
        return ('CHANGE', txt)
    return ('INFO', txt)


def compare_layouts(rep, exp, width, fatal, ref, work, pairs, idmap, unified=None, mrens=None):
    agg = collections.OrderedDict()           # (type, verdict, text) -> [tus]
    only = collections.defaultdict(list)
    ntypes = 0
    for tu, rs, ws in pairs:
        R = {idmap.get(k, k): v for k, v in ref['tus'][rs].items()}
        W = work['tus'][ws]
        for n in sorted(set(R) | set(W)):
            ntypes += 1
            if n not in W:
                only[('REF-ONLY', n)].append(tu)
                continue
            if n not in R:
                only[('WORK-ONLY', n)].append(tu)
                continue
            v = layout_verdict(R[n], W[n], idmap, (unified or {}).get((width, n)), (mrens or {}).get(n))
            if v is None:
                continue
            agg.setdefault((n, v[0], tuple(v[1])), []).append(tu)
    nchg = 0
    for (n, verdict, txt), tus in agg.items():
        if verdict == 'CHANGE':
            nchg += 1
            vs = set()
            for tu in tus:
                vs.add(classify(rep, exp, width, fatal, 'layout', tu, n, ''))
            v = ' / '.join(sorted(vs))
        else:
            rep.info['layout-neutral'] += 1
            v = 'INFO: layout unchanged (renames / opaque bytes refined / kinds)'
        rep.p('     layout %s %s in %d TU(s) [%s]: %s' % ('CHANGED' if verdict == 'CHANGE' else 'NEUTRAL', n, len(tus), v,
                                                         ' '.join(tus[:6]) + (' ...' if len(tus) > 6 else '')))
        rep.lines.extend('        ' + t for t in txt[:rep.maxdiff])
        if len(txt) > rep.maxdiff:
            rep.p('        ... (%d more)' % (len(txt) - rep.maxdiff))
    for (w, n), tus in sorted(only.items()):
        rep.info['layout-only'] += 1
        rep.p('     layout %s %s in %d TU(s) [INFO]: %s' % (w, n, len(tus), ' '.join(tus[:6]) + (' ...' if len(tus) > 6 else '')))
    return ntypes, nchg


# ----------------------------------------------------------------------------------------------------------------- main

def load_map(path):
    m = {}
    for ln, c in read_tsv(path, 2):
        m[c[0]] = c[1]
    return m


def make_demangler(names):
    names = sorted(set(n for n in names if n.startswith('_Z')))
    dm = {}
    if names:
        r = subprocess.run(['c++filt'], input='\n'.join(names) + '\n', stdout=subprocess.PIPE, universal_newlines=True)
        for a, b in zip(names, r.stdout.splitlines()):
            if a != b:
                dm[a] = b
    return lambda n: ('(%s)' % '='.join(dm.get(x, x) for x in n.split('='))) if any(x in dm for x in n.split('=')) else ''


def compare(opts, ref_tree, ref_sha, ref_cache_dir, work_tree, work_label, scratch, rep):
    idmap = load_map(opts.map)
    mrens = {}        # type -> {old member: new member}: the `member` rows of tools/o0check_unified.tsv (RS21)
    unified = {}      # (width, type) -> (old sizeof, new sizeof): tools/o0check_unified.tsv (RS6)
    for ln, c in read_tsv(opts.unified if opts.unified and os.path.exists(opts.unified) else None, 5):
        if c[0] == 'member':      # RS21: member rename of ONE type (the type-qualified counterpart of --map, which renames a name in every type)
            mrens.setdefault(c[1], {})[c[2]] = c[3]
            continue
        unified[(c[0], c[1])] = (int(c[2], 0), int(c[3], 0))
    exp = Expected(opts.expected if opts.expected and os.path.exists(opts.expected) else None)
    rep.exp = exp
    configs = [('m32', '32', '-O0', True)]
    if not opts.no_m64:
        configs.append(('m64', '64', '-O0', not opts.m64_advisory))
    if opts.o2 or opts.o2_only:
        o2fatal = opts.o2_only                                  # --o2-only: the -O2 identity IS the gate (RL2)
        if opts.o2_only:
            configs = []
        configs.append(('m32o2', '32', '-O2', o2fatal))
        if not opts.no_m64:
            configs.append(('m64o2', '64', '-O2', o2fatal))      # RS5: production x86-64 recipe at -O2 (advisory): found a -m64 -O2 difference that -m64 -O0 does not show
    tag = hashlib.sha256((gcc_version() + VERSION + self_hash()).encode()).hexdigest()[:12]
    wside = os.path.join(scratch, 'work-%d' % os.getpid())
    summary = []
    timings = []
    try:
        for width, bits, opt, fatal in configs:
            for what, o in (('code', opt), ('layout', opt + ' ' + LAYOUT_DBG)):
                if what == 'layout' and (opts.no_layout or width.endswith('o2')):
                    continue
                cfgname = '%s-%s' % (width, what)
                cache = None
                if ref_cache_dir and not opts.no_cache:
                    cache = os.path.join(ref_cache_dir, '%s-%s.pkl' % (cfgname, tag))
                t0 = time.time()
                R = collect(ref_tree, cfgname, bits, o, os.path.join(ref_cache_dir or wside, 'objs'), opts.jobs, None, cache)
                t1 = time.time()
                W = collect(work_tree, cfgname, bits, o, wside, opts.jobs, None, None)
                t2 = time.time()
                timings.append('%s: ref %.1fs%s, work %.1fs (build %.1fs + extract %.1fs)' % (
                    cfgname, t1 - t0, ' (cached)' if cache and t1 - t0 < 1 else '', t2 - t1, W['build_s'], W['extract_s']))
                # apply the identifier map to the REF side of the code by re-normalising? names were normalised without the map;
                # mapping is applied here on the canonical names / lines.
                if idmap and what == 'code':
                    R = map_code(R, idmap)
                pairs, ronly, wonly = pair_tus(list(R['tus']), list(W['tus']))
                fatal_here = fatal
                rep.p('')
                rep.p('== %s %s  (%s; ref %s vs %s)' % (cfgname, '' if fatal else '[ADVISORY: never fatal]', opt, ref_sha[:12], work_label))
                rep.p('   recipe sample (work): %s' % W['sample'])
                for r in ronly:
                    rep.unexpected['tu'] += fatal_here
                    rep.p('   TU only in REF: %s [%s]' % (r, 'UNEXPECTED' if fatal_here else 'ADVISORY'))
                for w in wonly:
                    rep.unexpected['tu'] += fatal_here
                    rep.p('   TU only in WORK: %s [%s]' % (w, 'UNEXPECTED' if fatal_here else 'ADVISORY'))
                if what == 'code':
                    allnames = [n for t in W['tus'].values() for n in t['funcs']] + [n for t in R['tus'].values() for n in t['funcs']]
                    dm = make_demangler([x for n in allnames for x in n.split('=')])
                    before = sum(rep.unexpected.values())
                    nf = compare_code(rep, exp, width, fatal_here, R, W, pairs, dm)
                    summary.append('%-12s %3d TU pairs, %5d functions: %d identical, %d changed, %d added, %d removed; %d data sections, %d differ'
                                   ' -> %s' % (cfgname, len(pairs), nf['total'], nf['identical'], nf['changed'], nf['added'],
                                               nf['removed'], nf['sections'], nf['data_diff'],
                                               'ADVISORY' if not fatal_here else ('CLEAN' if sum(rep.unexpected.values()) == before
                                                                                   else 'UNEXPECTED DIFFERENCES')))
                else:
                    before = sum(rep.unexpected.values())
                    nt, nchg = compare_layouts(rep, exp, width, fatal_here, R, W, pairs, idmap, unified, mrens)
                    summary.append('%-12s %3d TU pairs, %6d (TU, type) pairs, %d changed layouts (by type) -> %s' % (
                        cfgname, len(pairs), nt, nchg,
                        'ADVISORY' if not fatal_here else ('CLEAN' if sum(rep.unexpected.values()) == before else 'UNEXPECTED DIFFERENCES')))
    finally:
        shutil.rmtree(wside, ignore_errors=True)
    return summary, timings, exp


def map_code(R, idmap):
    """Apply the identifier map to already-normalised REF code (function keys, symbol names inside lines)."""
    if not idmap:
        return R
    pat = re.compile(r'(?<![\w$])('+ '|'.join(re.escape(k) for k in sorted(idmap, key=len, reverse=True)) + r')(?![\w$])')

    def ml(s):
        return pat.sub(lambda m: idmap[m.group(1)], s)
    out = dict(R)
    out['tus'] = {}
    for src, t in R['tus'].items():
        f = {}
        for k, lines in t['funcs'].items():
            f['='.join(sorted(ml(x) for x in k.split('=')))] = [ml(l) for l in lines]
        d = {ml(k): [ml(l) for l in v] for k, v in t['data'].items()}
        out['tus'][src] = dict(funcs=f, data=d, imports=sorted(ml(x) for x in t['imports']))
    return out


def work_tree_for(opts, top, scratch):
    if opts.work_dir:
        return os.path.abspath(opts.work_dir), 'dir ' + opts.work_dir, None
    if opts.work_ref:
        sha, d, tree = extract_ref(top, opts.work_ref, scratch)
        return tree, '%s (%s)' % (opts.work_ref, sha[:12]), d
    return top, 'working tree', None


def main_check(opts):
    top = git_top()
    scratch = os.path.abspath(opts.scratch)
    os.makedirs(scratch, exist_ok=True)
    t0 = time.time()
    trim_scratch(scratch, opts.cache_keep)
    ref_sha, ref_dir, ref_tree = extract_ref(top, opts.ref, scratch)
    work_tree, work_label, work_dir = work_tree_for(opts, top, scratch)
    rep = Report(opts.max_diff)
    rep.verbose_expected = opts.verbose
    summary, timings, exp = compare(opts, ref_tree, ref_sha, ref_dir, work_tree, work_label, scratch, rep)
    print('o0check: ref %s (%s) vs %s' % (opts.ref, ref_sha[:12], work_label))
    for l in rep.lines:
        print(l)
    print('')
    print('== summary')
    for s in summary:
        print('   ' + s)
    for r in exp.rows:
        if r['hits'] == 0:
            print('   expected entry not observed (line %d): %s %s %s %s' % (r['ln'], r['width'], r['kind'], r['tu'], r['name']))
    print('   timings: ' + '; '.join(timings))
    print('   wall %.1fs, -j %d' % (time.time() - t0, opts.jobs))
    nunexp = sum(rep.unexpected.values())
    print('RESULT: %s (unexpected %d: %s; expected %d; advisory/info %d)' % (
        ('IDENTICAL under the -O2 criterion (production recipes)' if opts.o2_only else 'IDENTICAL under the -m32 -O0 criterion' + ('' if opts.no_m64 else ' (+ -m64 within expected)')) if nunexp == 0 else 'DIFFERENT',
        nunexp, dict(rep.unexpected), sum(rep.expected.values()), sum(rep.info.values())))
    return 0 if nunexp == 0 else 1


# ----------------------------------------------------------------------------------------------------------------- self-test



def self_test(opts):
    """REF vs REF clean; a one-line change detected; comment + local rename + static rename under --map clean."""
    top = git_top()
    scratch = os.path.abspath(opts.scratch)
    me = os.path.abspath(__file__)
    base = [sys.executable, me, '-r', opts.ref, '--scratch', scratch, '-j', str(opts.jobs), '--expected', '', '--unified', '', '--map', '']
    res = []

    def go(label, extra, want, funcs=None, must=()):
        t = time.time()
        r = subprocess.run(base + extra, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        ok = (r.returncode == 0) == want
        out = r.stdout.splitlines()
        if funcs is not None:                     # exactly these functions reported (at every width)
            got = set(m.group(2) for m in (re.search(r'\bfunc (CHANGED|ADDED|REMOVED) (\S+)', l) for l in out) if m)
            if got != set(funcs):
                ok = False
                print('    reported functions %s != expected %s' % (sorted(got), sorted(funcs)))
        for pat in must:
            if not any(re.search(pat, l) for l in out):
                ok = False
                print('    no report line matches %r' % pat)
        last = [l for l in out if l.startswith('RESULT') or 'CHANGED' in l or 'UNEXPECTED' in l or 'NEUTRAL' in l][:12]
        res.append((label, ok, r.returncode, time.time() - t, last))
        print('SELFTEST %-60s %s (exit %d, %.1fs)' % (label, 'PASS' if ok else 'FAIL', r.returncode, time.time() - t))
        for l in last:
            print('    ' + l)
        return r

    go('1. REF vs REF (all widths, --o2): must be clean', ['--work-ref', opts.ref, '--o2'], True)
    sha, d, tree = extract_ref(top, opts.ref, scratch)
    edir = os.path.join(scratch, 'selftest-%d' % os.getpid())
    try:
        def fresh():
            shutil.rmtree(edir, ignore_errors=True)
            shutil.copytree(tree, edir)
            os.remove(os.path.join(edir, '.complete'))

        def edit(rel, old, new, count):
            path = os.path.join(edir, rel)
            if not os.path.exists(path):
                die('self-test: no file %s in the REF tree' % rel)
            s = open(path).read()
            if s.count(old) < 1:
                die('self-test edit anchor not found in %s: %r' % (path, old))
            s = s.replace(old, new, count)
            open(path, 'w').write(s)
        mp = os.path.join(scratch, 'selftest-map-%d.tsv' % os.getpid())
        member = cadence_member(tree)
        for label, edits, mapping, want, funcs, must in SELFTEST_CASES:
            label = label.replace('{M}', member)
            fresh()
            for rel, old, new, cnt in edits:
                edit(rel, old.replace('{M}', member), new.replace('{M}', member), cnt)
            extra = ['--work-dir', edir]
            if mapping:
                open(mp, 'w').write(''.join('%s\t%s\n' % kv for kv in mapping))
                extra += ['--map', mp]
            go(label, extra, want, funcs, must)
    finally:
        shutil.rmtree(edir, ignore_errors=True)
        if os.path.exists(mp):
            os.remove(mp)
    allok = all(r[1] for r in res)
    print('SELFTEST %s (%d cases)' % ('PASS' if allok else 'FAIL', len(res)))
    return 0 if allok else 1


def cadence_member(tree):
    """The name of the int member of Cadence that cases 6 / 7 edit (reportTone; the anchor is looked up in the REF tree)."""
    p = os.path.join(tree, 'src/tone/tone_detect.h')
    if os.path.exists(p) and re.search(r'\bint reportTone;', open(p).read()):
        return 'reportTone'
    die('self-test: no int reportTone member of Cadence found in the REF tree')


# (label, [(file below the repository root, old, new, count (-1 = all))], [(map old, map new)], expect clean, exactly these functions reported
#  (None = not checked), regexes that must match a report line).  Anchors are checked at run time.
SELFTEST_CASES = [
    ('2. one-line change (cadence_create maxCycles 100 -> 101): must be DETECTED',
     [('src/tone/tone_cadence.c', 'c->maxCycles = 100;', 'c->maxCycles = 101;', 1)], [], False, ['cadence_create'],
     [r'm32-code .* 1 changed']),
    ('3. comment change + local rename (absd: d -> diff): must be clean',
     [('src/tone/tone_cadence.c', '/* |a - b| with wrapping', '/* self-test: a changed comment. |a - b| with wrapping', 1),
      ('src/tone/tone_cadence.c', 'int d = (int)((unsigned)a - (unsigned)b);', 'int diff = (int)((unsigned)a - (unsigned)b);', 1),
      ('src/tone/tone_cadence.c', 'return d < 0 ? (int)(0u - (unsigned)d) : d;', 'return diff < 0 ? (int)(0u - (unsigned)diff) : diff;', 1)],
     [], True, [], ()),
    ('4. static function rename absd -> abs_diff WITHOUT --map: must be DETECTED',
     [('src/tone/tone_cadence.c', 'absd(', 'abs_diff(', -1)], [], False, ['absd', 'abs_diff', 'cadence_progress'], ()),
    ('5. static function rename absd -> abs_diff WITH --map absd<TAB>abs_diff: must be clean',
     [('src/tone/tone_cadence.c', 'absd(', 'abs_diff(', -1)], [('absd', 'abs_diff')], True, [], ()),
    ('6. struct member rename Cadence.{M} -> holdMode (tone_detect.h + tone_cadence.c): must be clean (layout INFO)',
     [('src/tone/tone_detect.h', 'int {M};', 'int holdMode;', 1), ('src/tone/tone_cadence.c', 'c->{M}', 'c->holdMode', -1)], [], True, [],
     [r'layout NEUTRAL Cadence ']),
    ('7. struct layout change (Cadence: int {M} -> short {M}; short pad): must be DETECTED as a layout change',
     [('src/tone/tone_detect.h', 'int {M};', 'short {M}; short rs1SelfTestPad;', 1)], [], False, None,
     [r'm32-layout .* [1-9]\d* changed layouts.*UNEXPECTED', r'layout CHANGED Cadence .*UNEXPECTED']),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-r', '--ref', default=DEFAULT_REF)
    ap.add_argument('--work-ref', help='compare against this git ref instead of the working tree')
    ap.add_argument('--work-dir', help='compare against this extracted tree (a copy of the repository root)')
    ap.add_argument('--map', default=os.path.join(HERE, 'o0check_renames.tsv') if os.path.exists(os.path.join(HERE, 'o0check_renames.tsv')) else None,
                    help='identifier renames old<TAB>new (default tools/o0check_renames.tsv, RS8: the renames of static helpers made when duplicates were unified; harmless for a REF after them)')
    ap.add_argument('--expected', default=DEFAULT_EXPECTED)
    ap.add_argument('--unified', default=os.path.join(HERE, 'o0check_unified.tsv'))
    ap.add_argument('--no-m64', action='store_true')
    ap.add_argument('--m64-advisory', action='store_true')
    ap.add_argument('--o2', action='store_true', help='also compare the production i386 recipe at -O2 (advisory)')
    ap.add_argument('--o2-only', action='store_true',
                    help='only the production -O2 recipes (-m32 and -m64), and FATAL: for passes meant to be neutral at -O2 (identical -O2 objects); faster than the full check')
    ap.add_argument('--no-layout', action='store_true')
    ap.add_argument('--no-cache', action='store_true')
    ap.add_argument('--cache-keep', type=int, default=4,
                    help='keep the N most recently used ref-* cache directories (~0.5 GB each) in the scratch dir, never one used in the last 2 hours; -1 = no trim (default 4)')
    ap.add_argument('-j', '--jobs', type=int, default=os.cpu_count() or 4)
    ap.add_argument('--scratch', default=os.path.join(os.environ.get('TMPDIR', '/tmp'), 'o0check-%s' % os.environ.get('USER', 'u')))
    ap.add_argument('--max-diff', type=int, default=80)
    ap.add_argument('-v', '--verbose', action='store_true', help='print the diff of expected changes too')
    ap.add_argument('--self-test', action='store_true')
    opts = ap.parse_args()
    if opts.self_test:
        sys.exit(self_test(opts))
    sys.exit(main_check(opts))


if __name__ == '__main__':
    main()
