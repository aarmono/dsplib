/**
 * @file dsp_copyword_plain.h
 * @brief copyWord(float *, const float *) as a plain 4-byte memcpy (C++).
 *
 * A float cell moved as an integer (no x87 load: signalling NaN payloads survive).  The variant with an optimizer barrier is dsp_copyword.h; the two
 * must not be included in one file.
 */
#ifndef DSP_COPYWORD_PLAIN_H
#define DSP_COPYWORD_PLAIN_H

/** Copy the 32-bit cell `*src` to `*dst` bit for bit (a float copy through the x87 would quiet a signalling NaN). */
static inline void copyWord(float* dst, const float* src) { __builtin_memcpy(dst, src, sizeof(float)); }

#endif /* DSP_COPYWORD_PLAIN_H */
