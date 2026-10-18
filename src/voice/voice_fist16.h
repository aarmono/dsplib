/**
 * @file voice_fist16.h
 * @brief fist16(): convert to a 16-bit integer the way the x87 `fistp` does with the truncating control word.
 *
 * NaN and values outside (-32769, 32768) give the "integer indefinite" 0x8000, where a C cast would be undefined.  Used by voice_api.c and
 * voice_engine.c wherever the audio path converts a float or extended sample to a 16-bit sample.
 */
#ifndef VOICE_FIST16_H
#define VOICE_FIST16_H

/** Truncate `v` toward zero to 16 bits; NaN and out-of-range input give -32768. */
static inline short fist16(long double v)
{
  if (!(v > -32769.0L && v < 32768.0L))
    return (short)0x8000;
  return (short)(int)v;
}

#endif /* VOICE_FIST16_H */
