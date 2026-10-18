/**
 * @file voice_f32.h
 * @brief F(): round a value to float exactly where the voice code stores a float.
 *
 * The voice code evaluates in x87 extended precision (`long double`) and rounds to single precision only where a value is stored into a float.
 * F() is that store: the value is rounded to float through a volatile object, so the compiler cannot keep it unrounded in a register.
 * LD() names the extended operand width.  Both are used by voice_api.c, voice_engine.c, voice_dtmf.c and voice_tone.c.
 */
#ifndef VOICE_F32_H
#define VOICE_F32_H

/** LD(x): convert to the x87 operand width (`long double`). */
#define LD(x) ((long double)(x))

/** Round `v` to float and return it (the value of a float store). */
static inline float F(long double v)
{
  volatile float f = (float)v;
  return f;
}

#endif /* VOICE_F32_H */
