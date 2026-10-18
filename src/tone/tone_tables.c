/**
 * @file tone_tables.c
 * @brief Constant tables of the tone-detect core (tone_iir.c, tone_cadence.c): filter coefficient sets, default configurations, the detection
 *        threshold table, the cadence names and the table registry behind the table words.
 *
 * - CP_450_630, CP_276_504, CP_100_550, CP_350_600: single-row coefficient sets (named for the band they cover, in Hz), each `_a[12]` (4 sections x
 *   {a0, a1, a2}, Q13), `_b[12]` (4 sections x {b0, b1, b2}) and `_scales[5]` (input shift, 3 inter-section shifts, output shift).
 * - Filter_350_500, Filter_100_550, Filter_276_504: seven rows of the same shapes each (`_a[84]`, `_b[84]`, `_scales[35]`); cadence_create() selects
 *   row (sub-index - 1) by the dial-tone filter sub-index (1 .. 7).
 * - toneiir_default_a / _b / _scales: all-zero tables of the default configuration; tone_allpass_b: the single coefficient of the all-pass configuration.
 * - toneiir_default_cfg, toneiir_configuration_allpass: ToneIirCfg blocks whose table words (see tone_detect.h) refer to the tables above.
 * - tone_thr_window: the detection thresholds with their surrounding words (see Get_Detection_Threshold_Table()).
 * - cadence_names: the tone-type names printed by cadence_create().
 * - tone_tabs / tone_ntabs: the registry of the built-in tables that table words index on 64-bit builds (the order defines the TID_* ids).
 */
#include "tone_detect.h"

/** toneiir_default_scales: 5 shorts, all zero */
const short toneiir_default_scales[5] = {
    0, 0, 0, 0, 0,
};
/** toneiir_default_b: 12 shorts, all zero */
const short toneiir_default_b[12] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
/** toneiir_default_a: 12 shorts, all zero */
const short toneiir_default_a[12] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
/** tone_allpass_b: 1 short */
const short tone_allpass_b[1] = {
    1,
};
/** CP_450_630_scales: 5 shorts */
const short CP_450_630_scales[5] = {
    5, 0, 0, 3, 0,
};
/** CP_450_630_a: 12 shorts */
const short CP_450_630_a[12] = {
    8192, -14092, 7471, 8192, -14757, 7599, 8192, -14031, 7911, 8192, -15382, 8011,
};
/** CP_450_630_b: 12 shorts */
const short CP_450_630_b[12] = {
    8192, -8156, 8192, 8192, -16210, 8192, 8192, -15893, 8192, 8192, -12938, 8192,
};
/** CP_276_504_scales: 5 shorts */
const short CP_276_504_scales[5] = {
    5, 0, 3, 0, 0,
};
/** CP_276_504_a: 12 shorts */
const short CP_276_504_a[12] = {
    8192, -14644, 7313, 8192, -15334, 7566, 8192, -14686, 7832, 8192, -15926, 8021,
};
/** CP_276_504_b: 12 shorts */
const short CP_276_504_b[12] = {
    8192, -7618, 8192, 8192, -16349, 8192, 8192, -13488, 8192, 8192, -16253, 8192,
};
/** CP_100_550_scales: 5 shorts */
const short CP_100_550_scales[5] = {
    6, 2, 0, 0, 0,
};
/** CP_100_550_a: 12 shorts */
const short CP_100_550_a[12] = {
    8192, -14227, 6339, 8192, -14395, 7265, 8192, -14513, 7880, 8192, -14584, 8123,
};
/** CP_100_550_b: 12 shorts */
const short CP_100_550_b[12] = {
    8192, -3688, 8192, 8192, -13180, 8192, 8192, -14233, 8192, 8192, -14449, 8192,
};
/** CP_350_600_scales: 5 shorts */
const short CP_350_600_scales[5] = {
    5, 0, 0, 2, 0,
};
/** CP_350_600_a: 12 shorts */
const short CP_350_600_a[12] = {
    8192, -12974, 6657, 8192, -14791, 7249, 8192, -12339, 7588, 8192, -15792, 7979,
};
/** CP_350_600_b: 12 shorts */
const short CP_350_600_b[12] = {
    8192, 1200, 8192, 8192, -16343, 8192, 8192, -16217, 8192, 8192, -9156, 8192,
};
/** Filter_350_500_scales: 35 shorts, 7 rows of 5 */
const short Filter_350_500_scales[35] = {
    5, 0, 3, 0, 0, 5, 0, 3, 0, 0, 5, 0,
    3, 0, 0, 5, 0, 3, 0, 0, 5, 0, 3, 0,
    0, 5, 0, 3, 0, 0, 5, 0, 3, 0, 0,
};
/** Filter_350_500_a: 84 shorts, 7 rows of 12 */
const short Filter_350_500_a[84] = {
    8192, -14584, 7422, 8192, -15206, 7602, 8192, -14602, 7883, 8192, -15783, 8022,
    8192, -14432, 7283, 8192, -15183, 7529, 8192, -14430, 7822, 8192, -15826, 8008,
    8192, -14273, 7137, 8192, -15165, 7461, 8192, -14245, 7756, 8192, -15869, 7997,
    8192, -14117, 6986, 8192, -15159, 7401, 8192, -14058, 7688, 8192, -15914, 7991,
    8192, -13955, 6829, 8192, -15159, 7351, 8192, -13860, 7617, 8192, -15955, 7989,
    8192, -13797, 6675, 8192, -15168, 7312, 8192, -13667, 7548, 8192, -15995, 7990,
    8192, -13624, 6500, 8192, -15192, 7284, 8192, -13456, 7471, 8192, -16039, 7995,
};
/** Filter_350_500_b: 84 shorts, 7 rows of 12 */
const short Filter_350_500_b[84] = {
    8192, -8523, 8192, 8192, -16317, 8192, 8192, -13563, 8192, 8192, -16161, 8192,
    8192, -6863, 8192, 8192, -16336, 8192, 8192, -16207, 8192, 8192, -13085, 8192,
    8192, -5165, 8192, 8192, -16349, 8192, 8192, -16245, 8192, 8192, -12550, 8192,
    8192, -3520, 8192, 8192, -16358, 8192, 8192, -16276, 8192, 8192, -11982, 8192,
    8192, -1948, 8192, 8192, -16365, 8192, 8192, -16301, 8192, 8192, -11372, 8192,
    8192, -541, 8192, 8192, -16370, 8192, 8192, -16319, 8192, 8192, -10762, 8192,
    8192, 862, 8192, 8192, -16374, 8192, 8192, -16336, 8192, 8192, -10084, 8192,
};
/** Filter_100_550_scales: 35 shorts, 7 rows of 5 */
const short Filter_100_550_scales[35] = {
    5, 0, 3, 0, 0, 5, 0, 3, 0, 0, 5, 0,
    3, 0, 0, 5, 0, 3, 0, 0, 5, 0, 3, 0,
    0, 5, 0, 0, 3, 0, 5, 0, 0, 3, 0,
};
/** Filter_100_550_a: 84 shorts, 7 rows of 12 */
const short Filter_100_550_a[84] = {
    8192, -14313, 7045, 8192, -15270, 7448, 8192, -14306, 7711, 8192, -15969, 8005,
    8192, -14148, 6885, 8192, -15268, 7400, 8192, -14114, 7640, 8192, -16008, 8003,
    8192, -13985, 6723, 8192, -15279, 7364, 8192, -13923, 7567, 8192, -16047, 8005,
    8192, -13810, 6547, 8192, -15301, 7340, 8192, -13719, 7490, 8192, -16086, 8012,
    8192, -13632, 6365, 8192, -15340, 7334, 8192, -13517, 7413, 8192, -16125, 8022,
    8192, -13436, 6167, 8192, -13302, 7333, 8192, -15396, 7348, 8192, -16163, 8036,
    8192, -13307, 6086, 8192, -13130, 7300, 8192, -15377, 7330, 8192, -16162, 8035,
};
/** Filter_100_550_b: 84 shorts, 7 rows of 12 */
const short Filter_100_550_b[84] = {
    8192, -4485, 8192, 8192, -16362, 8192, 8192, -16294, 8192, 8192, -12458, 8192,
    8192, -2861, 8192, 8192, -16368, 8192, 8192, -16316, 8192, 8192, -11867, 8192,
    8192, -1364, 8192, 8192, -16373, 8192, 8192, -16333, 8192, 8192, -11260, 8192,
    8192, 71, 8192, 8192, -16376, 8192, 8192, -16347, 8192, 8192, -10606, 8192,
    8192, 1364, 8192, 8192, -16379, 8192, 8192, -16357, 8192, 8192, -9947, 8192,
    8192, 2583, 8192, 8192, -16380, 8192, 8192, -16366, 8192, 8192, -9250, 8192,
    8192, 3214, 8192, 8192, -16380, 8192, 8192, -16366, 8192, 8192, -8815, 8192,
};
/** Filter_276_504_scales: 35 shorts, 7 rows of 5 */
const short Filter_276_504_scales[35] = {
    6, 0, 0, 3, 0, 5, 0, 3, 0, 0, 6, 0,
    0, 3, 0, 5, 0, 0, 3, 0, 5, 0, 0, 3,
    0, 5, 0, 0, 3, 0, 5, 0, 0, 3, 0,
};
/** Filter_276_504_a: 84 shorts, 7 rows of 12 */
const short Filter_276_504_a[84] = {
    8192, -15047, 7773, 8192, -15458, 7854, 8192, -15010, 8035, 8192, -15831, 8094,
    8192, -14947, 7726, 8192, -15415, 7823, 8192, -14891, 8016, 8192, -15823, 8087,
    8192, -14850, 7681, 8192, -15374, 7793, 8192, -14774, 7998, 8192, -15816, 8080,
    8192, -14752, 7636, 8192, -15335, 7765, 8192, -14653, 7979, 8192, -15809, 8073,
    8192, -14653, 7591, 8192, -15296, 7737, 8192, -14529, 7961, 8192, -15803, 8067,
    8192, -14554, 7546, 8192, -15257, 7709, 8192, -14402, 7942, 8192, -15797, 8061,
    8192, -14448, 7498, 8192, -15218, 7680, 8192, -14265, 7923, 8192, -15790, 8055,
};
/** Filter_276_504_b: 84 shorts, 7 rows of 12 */
const short Filter_276_504_b[84] = {
    8192, -11660, 8192, 8192, -16277, 8192, 8192, -16097, 8192, 8192, -14460, 8192,
    8192, -10948, 8192, 8192, -16285, 8192, 8192, -16106, 8192, 8192, -14235, 8192,
    8192, -10243, 8192, 8192, -16292, 8192, 8192, -16114, 8192, 8192, -14010, 8192,
    8192, -9519, 8192, 8192, -16298, 8192, 8192, -16121, 8192, 8192, -13773, 8192,
    8192, -8779, 8192, 8192, -16303, 8192, 8192, -16128, 8192, 8192, -13526, 8192,
    8192, -8030, 8192, 8192, -16307, 8192, 8192, -16134, 8192, 8192, -13269, 8192,
    8192, -7238, 8192, 8192, -16311, 8192, 8192, -16140, 8192, 8192, -12988, 8192,
};
/**
 * Detection thresholds: tone_thr_window[210 + i] is entry i of the 16-entry threshold table (90 ... 560, ascending), for i = 0 .. 15.  The 210 words
 * before it and the 30 words after it are not thresholds (coefficient-like words, then a slowly decaying run starting at 16384); they are here so that
 * Get_Detection_Threshold_Table() returns a defined value for every level 0 .. 255, indexing as (45 - level) + 210.
 */
const short tone_thr_window[256] = {
    5, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 8192, -11660, 8192, 8192, -16277, 8192,
    8192, -16097, 8192, 8192, -14460, 8192, 8192, -10948, 8192, 8192, -16285, 8192,
    8192, -16106, 8192, 8192, -14235, 8192, 8192, -10243, 8192, 8192, -16292, 8192,
    8192, -16114, 8192, 8192, -14010, 8192, 8192, -9519, 8192, 8192, -16298, 8192,
    8192, -16121, 8192, 8192, -13773, 8192, 8192, -8779, 8192, 8192, -16303, 8192,
    8192, -16128, 8192, 8192, -13526, 8192, 8192, -8030, 8192, 8192, -16307, 8192,
    8192, -16134, 8192, 8192, -13269, 8192, 8192, -7238, 8192, 8192, -16311, 8192,
    8192, -16140, 8192, 8192, -12988, 8192, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 8192, -15047, 7773, 8192, -15458, 7854,
    8192, -15010, 8035, 8192, -15831, 8094, 8192, -14947, 7726, 8192, -15415, 7823,
    8192, -14891, 8016, 8192, -15823, 8087, 8192, -14850, 7681, 8192, -15374, 7793,
    8192, -14774, 7998, 8192, -15816, 8080, 8192, -14752, 7636, 8192, -15335, 7765,
    8192, -14653, 7979, 8192, -15809, 8073, 8192, -14653, 7591, 8192, -15296, 7737,
    8192, -14529, 7961, 8192, -15803, 8067, 8192, -14554, 7546, 8192, -15257, 7709,
    8192, -14402, 7942, 8192, -15797, 8061, 8192, -14448, 7498, 8192, -15218, 7680,
    8192, -14265, 7923, 8192, -15790, 8055, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 90, 92, 96, 97, 99, 102,
    185, 188, 190, 250, 280, 285, 370, 390, 470, 560, 16384, 16384,
    16384, 16383, 16383, 16382, 16381, 16380, 16379, 16378, 16376, 16375, 16373, 16371,
    16369, 16367, 16364, 16362, 16359, 16356, 16353, 16350, 16347, 16343, 16340, 16336,
    16332, 16328, 16324, 16319,
};
/**
 * Default ToneIirCfg: all-zero tables (toneiir_default_a / _b / _scales), order 12 / 12, 500-sample window, ratio 0x2ccb (about 0.7), level 80,
 * integration 2200, miss 4, hold 0.
 */
const ToneIirCfg toneiir_default_cfg = {
    TONEW(toneiir_default_a), TONEW(toneiir_default_b), 12, 12, 500, (short)0x2ccb, (short)0x2, (short)80, (short)0x0, 2200, 4, 0, TONEW(toneiir_default_scales),
};
/**
 * All-pass ToneIirCfg: a = NULL (order 0), b = tone_allpass_b {1} (order 1), scales = NULL.  Only the allpass branch of cadence_create() uses it, and
 * that branch cannot run (see tone_cadence.c).
 * @warning toneiir_progress() with this configuration would dereference the NULL scales table.
 */
const ToneIirCfg toneiir_configuration_allpass = {
    0, TONEW(tone_allpass_b), 0, 1, 500, (short)0x2ccb, (short)0x2, (short)80, (short)0x0, 2200, 4, 0, 0,
};
/** Names of the CadenceType values, in enum order (printed as "TYPE %s" and kept in Cadence::name). */
const char *const cadence_names[5] = {"BUSY", "DIAL", "CONG", "RING", "INVALID"};
/** Registry of the built-in tables for table words (see tone_detect.h): the table id of an entry is its index + 1, in the order of the TID_* enum. */
const ToneTab tone_tabs[] = {
    {"toneiir_default_scales", toneiir_default_scales, 10},
    {"toneiir_default_b", toneiir_default_b, 24},
    {"toneiir_default_a", toneiir_default_a, 24},
    {"tone_allpass_b", tone_allpass_b, 2},
    {"CP_450_630_scales", CP_450_630_scales, 10},
    {"CP_450_630_a", CP_450_630_a, 24},
    {"CP_450_630_b", CP_450_630_b, 24},
    {"CP_276_504_scales", CP_276_504_scales, 10},
    {"CP_276_504_a", CP_276_504_a, 24},
    {"CP_276_504_b", CP_276_504_b, 24},
    {"CP_100_550_scales", CP_100_550_scales, 10},
    {"CP_100_550_a", CP_100_550_a, 24},
    {"CP_100_550_b", CP_100_550_b, 24},
    {"CP_350_600_scales", CP_350_600_scales, 10},
    {"CP_350_600_a", CP_350_600_a, 24},
    {"CP_350_600_b", CP_350_600_b, 24},
    {"Filter_350_500_scales", Filter_350_500_scales, 70},
    {"Filter_350_500_a", Filter_350_500_a, 168},
    {"Filter_350_500_b", Filter_350_500_b, 168},
    {"Filter_100_550_scales", Filter_100_550_scales, 70},
    {"Filter_100_550_a", Filter_100_550_a, 168},
    {"Filter_100_550_b", Filter_100_550_b, 168},
    {"Filter_276_504_scales", Filter_276_504_scales, 70},
    {"Filter_276_504_a", Filter_276_504_a, 168},
    {"Filter_276_504_b", Filter_276_504_b, 168},
};
const unsigned tone_ntabs = sizeof tone_tabs / sizeof tone_tabs[0];
