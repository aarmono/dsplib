/**
 * @file tone_detect.h
 * @brief Tone-detect core: the band-pass energy detector (ToneIir), the on / off cadence recogniser that drives it (Cadence), and the table words
 *        through which both refer to their coefficient tables.
 *
 * Users: the call-progress detectors (callprog/callprog.c) and the voice tone detectors (voice/voice_tone.c) create busy and dial Cadence objects
 * and feed them one line sample per call; nothing else creates these objects.  Implementation: tone_iir.c (ToneIir), tone_cadence.c (Cadence),
 * tone_tables.c (coefficient tables, default configurations, table registry), tone_table_words.c (table-word lookup, 64-bit builds only).
 *
 * @par Table words
 * The coefficient tables of a ToneIirCfg (a / b coefficients and scales) are referenced by TABLE WORDS: a 32-bit int slot that does not change the
 * size of the configuration block.  On a 32-bit build the word is the address itself (tone_word() / tone_ptr() are plain casts).  On a 64-bit build
 * a word is (table id << 16) | byte offset into one of the tables of the registry tone_tabs[] (the CP_*, Filter_*, toneiir_default_* tables of
 * tone_tables.c, so that a row such as `Filter_x_a + 24 * i` is representable), and tone_ptr() resolves it.  Code that builds a ToneIirCfg of its own
 * (the test harness; in the library only cadence_create() does) must register its coefficient array first with tone_tab_register() (a no-op macro
 * on 32-bit builds).
 *
 * @par Object layouts
 * ToneIirCfg and ToneIir have the same layout on every host (ints and shorts only; the offsets are asserted below).  Cadence holds real pointers
 * (the ToneIir it drives, its three coefficient tables, a name string, the host `struct modem`), so on a 64-bit build every member after the first
 * pointer sits at a different offset than on a 32-bit build: nothing may address a Cadence by a raw byte offset, always use the member names and
 * sizeof(Cadence).  The 32-bit offsets are asserted below.
 *
 * @par Cadence interface
 * - cadence_create(): allocate (or re-initialise a caller-supplied object) and configure from the host parameters.
 * - cadence_progress(): one sample per call; returns 0 (nothing), 1 (cadence recognised, or a tone window for the dial detector), 7 ("no answer").
 * - cadence_reset() / cadence_delete().
 */
#ifndef DSPLIBS_PORT_TONE_OBJ_H
#define DSPLIBS_PORT_TONE_OBJ_H
#include <stddef.h>
#include <stdint.h>

/* ---- table words ---- */

/** One entry of the table registry: a coefficient table that a table word can refer to. */
typedef struct ToneTab {
  const char *name;
  const short *base;
  unsigned bytes;
} ToneTab;
extern const ToneTab tone_tabs[];
extern const unsigned tone_ntabs;

#if UINTPTR_MAX == 0xffffffffu
#define TONEW(sym) ((int)(intptr_t)(sym))
#define tone_word(p) ((int)(intptr_t)(p))
#define tone_ptr(w) ((const short *)(intptr_t)(w))
#define tone_tab_register(name, base, bytes) ((void)(name), (void)(base), (void)(bytes))
#else
#define TONE_ID_(i) TID_##i
int tone_word(const void *p);          /**< pointer into a registered table -> table word (0 for NULL or an unknown address) */
const short *tone_ptr(int w);          /**< table word -> pointer (NULL for 0 or an unknown id) */
int tone_tab_register(const char *name, const void *base, unsigned bytes); /**< makes an array addressable by a table word; returns its id */
/** Table ids of the built-in tables: the position in tone_tabs[] plus one. */
enum {
  TID_NONE = 0,
  TID_toneiir_default_scales, TID_toneiir_default_b, TID_toneiir_default_a, TID_tone_allpass_b,
  TID_CP_450_630_scales, TID_CP_450_630_a, TID_CP_450_630_b, TID_CP_276_504_scales, TID_CP_276_504_a, TID_CP_276_504_b,
  TID_CP_100_550_scales, TID_CP_100_550_a, TID_CP_100_550_b, TID_CP_350_600_scales, TID_CP_350_600_a, TID_CP_350_600_b,
  TID_Filter_350_500_scales, TID_Filter_350_500_a, TID_Filter_350_500_b, TID_Filter_100_550_scales, TID_Filter_100_550_a, TID_Filter_100_550_b,
  TID_Filter_276_504_scales, TID_Filter_276_504_a, TID_Filter_276_504_b
};
#define TONEW(sym) ((int)(TID_##sym << 16))
#endif

/* ---- ToneIir ---- */

/**
 * Configuration block of a ToneIir (11 words, 0x2c bytes).  Sources: toneiir_get_default_configuration(), the stack block cadence_create()
 * builds, and the first 0x2c bytes of every ToneIir.
 */
typedef struct ToneIirCfg {
  int a_w;        /**< table word: 4 sections x {a0, a1, a2} (a0 unused; Q13 denominators) */
  int b_w;        /**< table word: 4 sections x {b0, b1, b2} */
  int na;         /**< number of y (feedback) state shorts cleared at create (clamped to 25) */
  int nb;         /**< number of w state shorts cleared at create (clamped to 25) */
  int win;        /**< samples per detection window, i.e. toneiir_progress() calls between two decisions */
  short ratio;    /**< Q14 limit on the energy ratio between consecutive windows */
  short mode;     /**< set to 2 by toneiir_progress() at every window end; not read */
  short level;    /**< minimum filtered-signal energy for a detection */
  short levelHi;  /**< not read */
  int integ;      /**< integration length (scaled by the window length into ToneIir::integThr) */
  int miss;       /**< a window without detection still counts as a hit when it ends a run of more than `miss` hit windows */
  int hold;       /**< 0: a window without detection clears the hit counters (unless it ends a run of more than `miss` hit windows) */
  int scales_w;   /**< table word: 5 shorts, input shift, 3 inter-section shifts, output shift */
} ToneIirCfg;

/** Four-section biquad band-pass detector state; one sample per toneiir_progress() call. */
typedef struct ToneIir {
  ToneIirCfg cfg; /**< active configuration (copied at create) */
  int cnt;        /**< samples of the current window */
  short w[25];    /**< section input histories (4 x 3 used) */
  short y[25];    /**< section output histories (4 x 2 used) */
  short e0;       /**< leaky energy of the input */
  short e1;       /**< leaky energy of the filtered signal */
  short p;        /**< e1 of the previous window */
  short pad9a;    /**< padding */
  int integThr;   /**< integration threshold in windows */
  int hits;       /**< windows with a detection (see toneiir_progress()) */
  int run;        /**< consecutive windows with a detection */
} ToneIir;

/* ---- Cadence ---- */

/**
 * CadenceParams::type: the tone the cadence detector looks for.  The names are the entries of cadence_names[] ("BUSY", "DIAL", "CONG", "RING",
 * "INVALID"; printed as "TYPE %s" at debug level 2).  cadence_create() treats every value from 4 on as DIAL and writes the type back clamped to 4.
 */
enum CadenceType {
  CADENCE_BUSY = 0,
  CADENCE_DIAL = 1,
  CADENCE_CONGESTION = 2,
  CADENCE_RINGBACK = 3,
  CADENCE_INVALID = 4
};

/**
 * Parameter record the callers of cadence_create() build (at least 0x1c bytes).  Only callerParamA, type and callerParamB are used; w0, w4, w8 and
 * w14 are filled in by the callers but not read.  `type` is also WRITTEN BACK (clamped to 4).
 */
typedef struct CadenceParams {
  int w0, w4, w8;
  int callerParamA;        /**< copied to Cadence::callerParamA (not read here) */
  unsigned type;           /**< CadenceType: 0 busy, 1 dial, 2 congestion, 3 ringback, 4 invalid (treated as dial); written back clamped to 4 */
  int w14;
  int callerParamB;        /**< copied to Cadence::callerParamB */
} CadenceParams;

struct modem;
/** On / off cadence recogniser state.  Create with cadence_create(), release with cadence_delete(). */
typedef struct Cadence {
  ToneIir *ti;           /**< the tone detector this cadence drives (owned) */
  int state;             /**< current phase: 0 = in a tone, 1 = in silence (cadence_create() starts in silence) */
  int cnt;               /**< windows in the current phase */
  int on[60];            /**< tone durations of the cycle so far, in windows */
  int off[60];           /**< silence durations, in windows (off[i] is the silence before on[i]) */
  int idx;               /**< next slot of on[] / off[] (wraps to 0 after slot 59) */
  int cycles;            /**< completed cycles; more than 9 is reported as "no answer" */
  int reservedA[25];     /**< not read or written by any code in the library */
  int maxOn;             /**< longest accepted on duration, windows */
  int maxOff;            /**< longest accepted off duration, windows */
  int minOn;             /**< shortest accepted on duration, windows */
  int minOff;            /**< shortest accepted off duration, windows */
  int maxCycles;         /**< slots to collect before the cadence is judged */
  int reportTone;        /**< 1: every tone window is reported (cadence_progress() returns 1): the dial-tone detector, where the tone itself is the
                          *       event; 0 for busy / congestion / ringback and whenever cadence_create() gets extra > 0.  Also the ToneIir's cfg.hold */
  int resetOff;          /**< silence length (windows) that restarts the cycle */
  int scaledLimitA;      /**< a caller-set duration, rescaled to windows by cadence_create() like the on / off limits; not read elsewhere */
  int scaledLimitB;      /**< likewise (rescaled first) */
  int callerParamA;      /**< copy of CadenceParams::callerParamA (stored by cadence_create(), not read) */
  int ordA;              /**< order of the a coefficients (12) */
  int ordB;              /**< order of the b coefficients (12) */
  const short *tabA;     /**< a coefficients of the selected filter */
  const short *tabB;     /**< b coefficients of the selected filter */
  const short *tabScales;/**< scales of the selected filter */
  short thr;             /**< level threshold (ToneIirCfg::level) */
  short thrPad;          /**< alignment padding after thr */
  int integ;             /**< integration length: dial type: validation-time parameter * 100 (less 100 when above 100); other types: 0 */
  int bufLen;            /**< samples per detection window */
  const char *name;      /**< "BUSY" / "DIAL" / "CONG" / "RING" / "INVALID" */
  int callerParamB;      /**< copy of CadenceParams::callerParamB (stored by cadence_create(), not read) */
  int allpass;           /**< cleared by cadence_create(); the all-pass configuration branch that tests it is therefore unreachable */
  int loose;             /**< busy-tone loose detection (host parameter) */
  int ser[4];            /**< series compare values (nothing in the library sets them) */
  int reservedB[4];      /**< not read or written by any code in the library */
  int seriesFlag;        /**< non-zero: judge against ser[] (cleared by cadence_create(); nothing in the library sets it) */
  int seriesMin;         /**< minimum slot count for the series comparison (nothing in the library sets it) */
  struct modem *modem;   /**< host modem object the parameters are read from */
} Cadence;

/* The 32-bit layout is asserted (ToneIirCfg and ToneIir are the same on every host). */
_Static_assert(sizeof(ToneIirCfg) == 0x2c && offsetof(ToneIirCfg, ratio) == 0x14 && offsetof(ToneIirCfg, mode) == 0x16 && offsetof(ToneIirCfg, level) == 0x18 &&
                   offsetof(ToneIirCfg, integ) == 0x1c && offsetof(ToneIirCfg, hold) == 0x24 && offsetof(ToneIirCfg, scales_w) == 0x28, "ToneIirCfg");
_Static_assert(sizeof(ToneIir) == 0xa8 && offsetof(ToneIir, cnt) == 0x2c && offsetof(ToneIir, w) == 0x30 && offsetof(ToneIir, y) == 0x62 && offsetof(ToneIir, e0) == 0x94 &&
                   offsetof(ToneIir, p) == 0x98 && offsetof(ToneIir, integThr) == 0x9c && offsetof(ToneIir, run) == 0xa4, "ToneIir");
#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(Cadence) == 0x2dc && offsetof(Cadence, state) == 4 && offsetof(Cadence, on) == 0xc && offsetof(Cadence, off) == 0xfc && offsetof(Cadence, idx) == 0x1ec &&
                   offsetof(Cadence, cycles) == 0x1f0 && offsetof(Cadence, maxOn) == 0x258 && offsetof(Cadence, maxCycles) == 0x268 && offsetof(Cadence, resetOff) == 0x270 &&
                   offsetof(Cadence, tabA) == 0x288 && offsetof(Cadence, thr) == 0x294 && offsetof(Cadence, bufLen) == 0x29c && offsetof(Cadence, name) == 0x2a0 &&
                   offsetof(Cadence, allpass) == 0x2a8 && offsetof(Cadence, ser) == 0x2b0 && offsetof(Cadence, seriesFlag) == 0x2d0 && offsetof(Cadence, modem) == 0x2d8,
               "Cadence");
#endif

/* ---- functions ---- */
extern ToneIir *toneiir_create(ToneIir *obj, const ToneIirCfg *cfg);
extern void toneiir_delete(ToneIir *obj);
extern void toneiir_reset(ToneIir *obj);
extern int toneiir_progress(ToneIir *obj, short sample); /* 0 mid-window; at a window end 1 (no tone) or 2 (tone) */
extern void toneiir_get_default_configuration(ToneIirCfg *out);
extern short GetFP_Value(short a, short b);              /* ceil(|a| * 16384 / |b|) as a short, negated unless a * b > 0 (returns 0 for b == 0 with a != 0) */
extern short Get_Detection_Threshold_Table(short level); /* threshold table entry 45 - level (level 30 .. 45; see tone_thr_window) */
extern Cadence *cadence_create(Cadence *obj, CadenceParams *p, int extra, struct modem *m);
extern int cadence_progress(Cadence *c, short sample);
extern void cadence_reset(Cadence *c);
extern void cadence_delete(Cadence *c);

/* ---- tables (tone_tables.c) ---- */
extern const short toneiir_default_scales[5], toneiir_default_b[12], toneiir_default_a[12], tone_allpass_b[1];
#define TONE_CP_DECL(n) extern const short n##_scales[5], n##_a[12], n##_b[12];
TONE_CP_DECL(CP_450_630) TONE_CP_DECL(CP_276_504) TONE_CP_DECL(CP_100_550) TONE_CP_DECL(CP_350_600)
#define TONE_FILTER_DECL(n) extern const short n##_scales[35], n##_a[84], n##_b[84];
TONE_FILTER_DECL(Filter_350_500) TONE_FILTER_DECL(Filter_100_550) TONE_FILTER_DECL(Filter_276_504)
extern const short tone_thr_window[256];
extern const ToneIirCfg toneiir_default_cfg, toneiir_configuration_allpass;
extern const char *const cadence_names[5];
#endif
