#!/usr/bin/env python3
"""dwarf_types.py OBJDIR OUT.json : every struct / class / union type DEFINED in each object's DWARF (gcc -g), with
byte size and member offsets (anonymous struct / union members flattened into the parent, as C/C++ name lookup does).
Type key = DW_AT_name, or the typedef name of an anonymous struct.  Read-only scan helper (layout-scan.md)."""
import json, os, sys
from elftools.elf.elffile import ELFFile

TAGS = ('DW_TAG_structure_type', 'DW_TAG_class_type', 'DW_TAG_union_type')


def attr(d, n):
    a = d.attributes.get(n)
    return a.value if a is not None else None


def name_of(d):
    v = attr(d, 'DW_AT_name')
    return v.decode() if isinstance(v, bytes) else v


def member_off(m):
    v = attr(m, 'DW_AT_data_member_location')
    if v is None:
        return 0
    if isinstance(v, int):
        return v
    # exprloc DW_OP_plus_uconst
    if isinstance(v, list) and len(v) >= 2 and v[0] == 0x23:
        r, s, i = 0, 0, 1
        while True:
            b = v[i]; r |= (b & 0x7f) << s; s += 7; i += 1
            if not b & 0x80:
                break
        return r
    return None


def type_die(cu, d):
    a = d.attributes.get('DW_AT_type')
    if a is None:
        return None
    return d.get_DIE_from_attribute('DW_AT_type')


def strip(t):
    while t is not None and t.tag in ('DW_TAG_typedef', 'DW_TAG_const_type', 'DW_TAG_volatile_type'):
        t = t.get_DIE_from_attribute('DW_AT_type') if 'DW_AT_type' in t.attributes else None
    return t


def kind_of(t):
    t = strip(t)
    if t is None:
        return 'void'
    if t.tag == 'DW_TAG_pointer_type' or t.tag == 'DW_TAG_reference_type':
        return 'ptr'
    if t.tag == 'DW_TAG_array_type':
        e = strip(t.get_DIE_from_attribute('DW_AT_type'))
        return 'arr:' + kind_of(e)
    if t.tag in TAGS:
        return 'rec:' + (name_of(t) or '?')
    if t.tag == 'DW_TAG_base_type':
        return name_of(t)
    if t.tag == 'DW_TAG_enumeration_type':
        return 'enum'
    return t.tag


def tsize(st):
    if st is None:
        return None
    if st.tag == 'DW_TAG_array_type':
        e = strip(st.get_DIE_from_attribute('DW_AT_type'))
        es = tsize(e)
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
    return attr(st, 'DW_AT_byte_size')


def members(d, base, out, prefix=''):
    for m in d.iter_children():
        if m.tag != 'DW_TAG_member' and m.tag != 'DW_TAG_inheritance':
            continue
        if attr(m, 'DW_AT_external') or m.attributes.get('DW_AT_declaration'):
            continue  # static data member
        o = member_off(m)
        if o is None:
            continue
        t = type_die(None, m)
        n = name_of(m)
        st = strip(t)
        if m.tag == 'DW_TAG_inheritance':
            if st is not None:
                members(st, base + o, out, prefix)
            continue
        if n is None:   # anonymous struct / union: flatten
            if st is not None and st.tag in TAGS:
                members(st, base + o, out, prefix)
            continue
        sz = tsize(st)
        out[prefix + n] = [base + o, kind_of(t), sz]


def main():
    objdir, outp = sys.argv[1], sys.argv[2]
    res = {}
    for fn in sorted(os.listdir(objdir)):
        if not fn.endswith('.o'):
            continue
        tu = fn[:-2]
        types = {}
        with open(os.path.join(objdir, fn), 'rb') as f:
            elf = ELFFile(f)
            if not elf.has_dwarf_info():
                continue
            dw = elf.get_dwarf_info()
            for cu in dw.iter_CUs():
                anon_td = {}
                key_of = {}
                for d in cu.iter_DIEs():
                    if d.tag == 'DW_TAG_typedef' and 'DW_AT_type' in d.attributes:
                        t = d.get_DIE_from_attribute('DW_AT_type')
                        if t.tag in TAGS and name_of(t) is None:
                            anon_td[t.offset] = name_of(d)
                for d in cu.iter_DIEs():
                    if d.tag not in TAGS or 'DW_AT_declaration' in d.attributes:
                        continue
                    sz = attr(d, 'DW_AT_byte_size')
                    if sz is None:
                        continue
                    n = name_of(d) or anon_td.get(d.offset)
                    if n is None:
                        continue
                    # qualify nested types by parent
                    p = d.get_parent()
                    while p is not None and p.tag in TAGS + ('DW_TAG_namespace',):
                        pn = name_of(p) or anon_td.get(p.offset)
                        if pn:
                            n = pn + '::' + n
                        p = p.get_parent()
                    mem = {}
                    members(d, 0, mem)
                    if n in types and types[n]['size'] != sz:
                        n = n + '#2'
                    types[n] = {'size': sz, 'tag': d.tag[7:], 'members': mem}
                    key_of[d.offset] = n
                # RS1: typedef aliases of named records (`typedef V90Phase2Info V90Phase2InfoView;`) are listed under the alias name too
                for d in cu.iter_DIEs():
                    if d.tag != 'DW_TAG_typedef' or 'DW_AT_type' not in d.attributes:
                        continue
                    t = strip(d.get_DIE_from_attribute('DW_AT_type'))
                    n = name_of(d)
                    if n and t is not None and t.offset in key_of and n not in types and anon_td.get(t.offset) != n:
                        types[n] = dict(types[key_of[t.offset]], alias_of=key_of[t.offset])
        res[tu] = types
    json.dump(res, open(outp, 'w'))
    print(len(res), 'TUs', sum(len(v) for v in res.values()), 'type defs')


main()
