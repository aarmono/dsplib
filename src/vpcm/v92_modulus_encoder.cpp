/**
 * @file v92_modulus_encoder.cpp
 * @brief V92ModulusEncoder: the V.92 modulus encoder of the transmitter (data bits of one symbol group -> twelve mixed-radix digits).
 *
 * Defines the class declared in v92_modulus_encoder.h: the constructor, reset() and progress().  The 64-bit arithmetic uses int64_t / uint64_t
 * operators (wrapping products, signed divisions); no floating point and no tables are involved.
 *
 * reset(): copies the number of bits and the twelve radices m_[0..11]; Q1 = m0 * .. * m5 and Q2 = m6 * .. * m11 (64-bit, wrapping), prod_ = Q1 * Q2.
 * a_ and b_ are then the two constants of a normalisation: with w(k) = Q1 * (int64)(Q2 >> k) (wrapping), k = 0 .. 8 is searched while the product is
 * positive and w(k) > 0; k == 8 gives a_ = 0, b_ = prod_; otherwise a_ = (int64)w(k + 1) >> (62 - k) and b_ = (w(k + 1) << (k + 1)) & 0x7fff..f
 * (k stays 0 when the product is not positive).
 *
 * progress() in mode 0 forms the number N = hi * 2^63 + lo of the data bits (lo = bits 0 .. 62, hi = the bits from 63 on), compares (hi, lo) with a
 * threshold (a_ / 2, (b_ - 1) / 2 + (a_ % 2) * 2^62), toggles flag_ accordingly (flag_ != 0 complements the number: hi' = a_ - hi, lo' = b_ - lo - 1),
 * and then writes the twelve mixed-radix digits d_k = n_k mod m_k, n_{k+1} = (n_k - d_k) / m_k.  A number with hi' > 0 does not fit 63 bits: hi' * 2^63
 * is then reduced with the constant -2^63 (INT64_MIN, the signed value of the bit pattern 2^63) through hiTerm(), for digit 0 and digit 1.
 * Modes 1 and 2 write the largest digit value (m_k - 1) for six of the twelve digits and 0 for the others.
 *
 * Details a reader should know:
 * - The digit subtraction uses the 32-bit digit zero-extended (a NEGATIVE signed remainder is truncated to 32 bits first).
 * - The bit counter of the accumulation loop is a signed int; a bit count >= 0x40 (unsigned test) splits the accumulation at bit 63.
 * - m_[11] is only used in prod_ and in mode 2; digit 11 is the last quotient (n_11), not a remainder.
 * - A zero radix traps (division by zero).
 */

#include "v92_modulus_encoder.h"

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

/** @brief Wrapping 64-bit signed product (the low 64 bits of the exact product). */
static inline int64_t mulw(int64_t a, int64_t b) { return (int64_t)((uint64_t)a * (uint64_t)b); }

/** @brief Wrapping 64-bit signed sum. */
static inline int64_t addw(int64_t a, int64_t b) { return (int64_t)((uint64_t)a + (uint64_t)b); }

/**
 * @brief Reduction term of hi * 2^63 modulo m for a high part v > 0, with the constant K = INT64_MIN = -2^63.
 *
 * q0 = K / m, r = v % m, S = r * m.  If S <= 1 the term is 0.  Otherwise T = (S * (q0 >> k)) << k masked to 63 bits, with k the first shift
 * (1 .. 8) at which S * (q0 >> k) is no longer positive, or T = q0 * r * m when the search runs to k == 8.  The term is (K - T) % m.
 *
 * @param v  high part of the number (greater than 0)
 * @param m  radix of the digit being formed (non-zero)
 * @return   the term to add to the low part's remainder
 */
static inline int64_t hiTerm(int64_t v, uint32_t m)
{
  const int64_t K = (int64_t)(((uint64_t)1) << 63); /* INT64_MIN */
  int64_t sm = (int64_t)m;
  int64_t q0 = K / sm;
  int64_t r = v % sm;
  int64_t S = (int64_t)((uint64_t)r * (uint64_t)m);
  if (S <= 1)
    return 0;

  uint64_t T = 0;
  unsigned k = 0;
  bool full = false;
  if (mulw(q0, S) > 0)
  {
    int64_t p;
    do
    {
      k++;
      p = mulw(S, q0 >> k);
    } while (k < 8 && p > 0);
    if (k == 8)
    {
      T = (uint64_t)q0 * (uint64_t)r * (uint64_t)m;
      full = true;
    }
  }
  if (!full)
  {
    k++;
    T = ((uint64_t)mulw(S, q0 >> k) << k) & 0x7fffffffffffffffull;
  }
  return (int64_t)((uint64_t)K - T) % sm;
}

/* ================================================================================================ V92ModulusEncoder */

V92ModulusEncoder::V92ModulusEncoder()
{
  nbits_ = 0;
  for (int k = 11; k >= 0; k--)
    m_[k] = 0;
}

void V92ModulusEncoder::reset(V92MappingParams* p)
{
  nbits_ = (uint32_t)p->w0_;
  for (int k = 0; k < 12; k++)
    m_[k] = p->m_[k];

  /* the two half products and their (wrapping) product */
  uint64_t q1 = (uint64_t)m_[0] * m_[1] * m_[2] * m_[3] * m_[4] * m_[5];
  uint64_t q2 = (uint64_t)m_[6] * m_[7] * m_[8] * m_[9] * m_[10] * m_[11];
  prod_ = q1 * q2;

  /* shift search: the largest k (at most 8) for which Q1 * (Q2 >> k) is still positive */
  unsigned k = 0;
  if ((int64_t)(q1 * q2) > 0)
  {
    int64_t w;
    do
    {
      k++;
      w = mulw((int64_t)q1, (int64_t)q2 >> k);
    } while (k < 8 && w > 0);
    if (k == 8)
    {
      a_ = 0;
      b_ = prod_;
      flag_ = 0;
      mode_ = 0;
      return;
    }
  }

  int64_t w = mulw((int64_t)q1, (int64_t)q2 >> (k + 1));
  a_ = w >> (62 - k);
  b_ = ((uint64_t)w << (k + 1)) & 0x7fffffffffffffffull;
  flag_ = 0;
  mode_ = 0;
}

void V92ModulusEncoder::progress(unsigned char* bits, unsigned* digits)
{
  if (mode_ == 1)
  { /* the largest digits of the pattern (m0, m1, 0, 0, m4, m5, 0, 0, m8, m9, 0, 0) */
    digits[0] = m_[0] - 1;
    digits[1] = m_[1] - 1;
    digits[2] = 0;
    digits[3] = 0;
    digits[4] = m_[4] - 1;
    digits[5] = m_[5] - 1;
    digits[6] = 0;
    digits[7] = 0;
    digits[8] = m_[8] - 1;
    digits[9] = m_[9] - 1;
    digits[10] = 0;
    digits[11] = 0;
    return;
  }
  if (mode_ == 2)
  { /* the complementary pattern (0, 0, m2, m3, 0, 0, m6, m7, 0, 0, m10, m11) */
    digits[0] = 0;
    digits[1] = 0;
    digits[2] = m_[2] - 1;
    digits[3] = m_[3] - 1;
    digits[4] = 0;
    digits[5] = 0;
    digits[6] = m_[6] - 1;
    digits[7] = m_[7] - 1;
    digits[8] = 0;
    digits[9] = 0;
    digits[10] = m_[10] - 1;
    digits[11] = m_[11] - 1;
    return;
  }
  if (mode_ != 0)
    return;

  /* accumulate the number N = hi * 2^63 + lo: lo = bits 0 .. 62, hi = bits 63 .. nbits_ - 1 (most significant first; the counters are signed ints) */
  uint64_t accLo = 0, accHi = 0;
  if (nbits_ < 0x40)
  {
    for (int e = (int)(nbits_ - 1u); e >= 0; e--)
      accLo = (accLo << 1) | (uint64_t)(bits[e] & 1u);
  }
  else
  {
    int e;
    for (e = (int)(nbits_ - 1u); e > 0x3e; e--)
      accHi = (accHi << 1) | (uint64_t)(bits[e] & 1u);
    for (e = 0x3e; e >= 0; e--)
      accLo = (accLo << 1) | (uint64_t)(bits[e] & 1u);
  }
  int64_t X = (int64_t)accHi;
  int64_t Y = (int64_t)accLo;

  /* threshold (a_ / 2, R) and the toggle t: t = 0 when (X, Y) < (a_ / 2, (b_ - 1) / 2 + (a_ % 2) * 2^62) lexicographically (signed compares) */
  int64_t a = a_;
  int64_t halfA = a / 2;
  int64_t bm1 = (int64_t)(b_ - 1);
  int64_t R = addw(bm1 / 2, (int64_t)((uint64_t)(a % 2) << 62));
  uint32_t t = 1;
  if (X < halfA || (X == halfA && Y < R))
    t = 0;

  /* complement the number when the flag from the previous call is set, then normalise the sign of the low part */
  int64_t Xp = X, Yp = Y;
  if (flag_ != 0)
  {
    Xp = 0;
    if (a != 0)
      Xp = (int64_t)((uint64_t)a - (uint64_t)X);
    Yp = (int64_t)((uint64_t)b_ - (uint64_t)Y - 1u);
  }
  if (Yp < 0 && Xp > 0)
  { /* bit 63 set with a positive high part: borrow one from it */
    Yp = (int64_t)((uint64_t)Yp + 0x8000000000000000ull);
    Xp = (int64_t)((uint64_t)Xp - 1u);
  }
  flag_ ^= t;

  /* digits 0 (and 1 when the high part is large): fold the high part in through hiTerm() */
  const int64_t K = (int64_t)(((uint64_t)1) << 63); /* INT64_MIN */
  int64_t q;
  int from;
  uint32_t m0 = m_[0];
  if (Xp <= 0)
  {
    unsigned d0 = (unsigned)(Yp % (int64_t)m0);
    digits[0] = d0;
    q = (int64_t)((uint64_t)Yp - (uint64_t)d0) / (int64_t)m0;
    from = 1;
  }
  else
  {
    int64_t V = Xp, W = Yp;
    int64_t h0 = hiTerm(V, m0);
    int64_t w0 = W % (int64_t)m0;
    digits[0] = (unsigned)(addw(h0, w0) % (int64_t)m0);
    int32_t vh = (int32_t)((uint64_t)V >> 32);
    uint32_t vl = (uint32_t)V;
    if (vh < 1 && ((uint32_t)vh > 0x7fffffffu || vl < m0))
    { /* 0 < V < m0 */
      q = addw(W / (int64_t)m0, mulw(K / (int64_t)m0, V));
      from = 1;
    }
    else
    {
      int64_t Vq = V / (int64_t)m0;
      int64_t qd = K / (int64_t)m0;
      int64_t Vr = V % (int64_t)m0;
      int64_t Wq = W / (int64_t)m0;
      int64_t c = addw(mulw(Vr, qd), Wq);
      if (0 < Vq)
      {
        uint32_t m1 = m_[1];
        int64_t h1 = hiTerm(Vq, m1);
        int64_t c1 = c % (int64_t)m1;
        digits[1] = (unsigned)(addw(h1, c1) % (int64_t)m1);
        q = addw(mulw(Vq % (int64_t)m1, K / (int64_t)m1), c / (int64_t)m1);
        from = 2;
      }
      else
      {
        q = c;
        from = 1;
      }
    }
  }

  /* the remaining digits by repeated division; the last quotient is digit 11 */
  for (int k = from; k <= 10; k++)
  {
    unsigned dk = (unsigned)(q % (int64_t)m_[k]);
    digits[k] = dk;
    q = (int64_t)((uint64_t)q - (uint64_t)dk) / (int64_t)m_[k];
  }
  digits[11] = (unsigned)q;
}
