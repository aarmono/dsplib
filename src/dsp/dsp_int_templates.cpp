/**
 * @file dsp_int_templates.cpp
 * @brief Scrambler, Descrambler, parallel and serial differential coders and Queue<float> (declarations in dsp_int_templates.h).
 *
 * Integer / bit work only: no floating-point arithmetic, no tables, the only callees are sysdep_malloc() and sysdep_free().  The explicit instantiations at
 * the end of the file name exactly the member functions that exist (51 of them); a class-wide `template class` would emit more.
 *
 * Behaviour notes.
 *  - copyHistoryTail() is a FORWARD element-by-element copy, not memmove: the source [buf_, buf_+tail_) and destination [head0_+1, head0_+tail_] overlap
 *    when slack + 1 < tapFar, and the forward copy then smears the history.  The loop is written so that the compiler keeps that order.
 *  - Scrambler<*,uchar> stores the 8-bit result zero-extended into the history cell.
 *  - Descrambler::process stores the input FIRST and then reads the three cells (*wr_ ^ *t1_ ^ *t2_): with tapNear == 0 the read-back sees the new symbol.
 *  - The parallel encoder and decoder differ in one constructor store (the encoder clears `state_` before the allocation, the decoder does not); not
 *    observable.
 *  - Queue: used = (wr_ + size_ - rd_) % size_ is an unsigned 32-bit division (size_ == 0 faults); the signed comparisons in write(T*, n) / read()
 *    (`tail < (int)n`) and the unsigned `room - 1 < n` / `used < n` are meant exactly so; the `n > 0` guard around the copy and the `p == last_ + 1` wrap
 *    test are kept as they are (the latter runs even when n == 0).
 *  - Queue<float> moves its elements as 32-bit WORDS: every element copy is a __builtin_memcpy, and write(float v) copies the by-value argument through an
 *    integer register (see P01_COPY_ELEM): a plain `*p = v` is a floating-point load and store on i386 and quiets signalling NaNs.
 *  - No bounds checks anywhere: tapNear > tapFar, a wild object, size_ == 0 and the like misbehave without a diagnostic.
 */

#include "dsp_int_templates.h"
#include "core/dsp_asm.h"

/* ------------------------------------------------------------------------------------------------ Scrambler<H,O> */

/** @brief Build the scrambler: allocate the history block, set the positions (see the class description) and clear it with reset(0). */
template <class H, class O> Scrambler<H, O>::Scrambler(unsigned tapNear, unsigned tapFar, unsigned slack)
{
  tail_ = tapFar;
  buf_ = (H*)sysdep_malloc((tapFar + slack + 1) * (unsigned)sizeof(H));
  head0_ = buf_ + slack;
  t1_0_ = head0_ + tapNear;
  end_ = head0_ + tapFar;
  reset(0);
}

template <class H, class O> Scrambler<H, O>::~Scrambler()
{
  if (buf_)
    sysdep_free(buf_);
}

/** @brief Write position and both tap positions back to their start values. */
template <class H, class O> void Scrambler<H, O>::resetHistoryIndexes()
{
  wr_ = head0_;
  t1_ = t1_0_;
  t2_ = end_;
}

/** @brief Move the newest `tail_` history elements (buf_[0 .. tail_-1]) to head0_[1 .. tail_]; a forward copy (see the file header). */
template <class H, class O> void Scrambler<H, O>::copyHistoryTail()
{
  H* dst = head0_ + 1;
  const H* src = buf_;
  unsigned n = tail_;

  while (n-- != 0)
    *dst++ = *src++;
}

/** @brief Restart: indices to the start, all history cells head0_[1 .. tapFar] set to the low bit of `v`. */
template <class H, class O> void Scrambler<H, O>::reset(H v)
{
  resetHistoryIndexes();
  H bit = (H)(v & 1);

  for (H* p = head0_ + 1; p <= end_; ++p)
    *p = bit;
}

/** @brief Scramble one symbol: v = in ^ y[n-tapNear] ^ y[n-tapFar]; v is stored in the history (wrapping the block when it is full) and returned. */
template <class H, class O> O Scrambler<H, O>::process(H in)
{
  O v = (O)(*t1_ ^ in ^ *t2_);

  --t1_;
  --t2_;
  *wr_ = (H)v;
  --wr_;
  if (wr_ < buf_)
  {
    resetHistoryIndexes();
    copyHistoryTail();
  }
  return v;
}

/** @brief Scramble `n` symbols from `in` to `out`; as process(H) per symbol. */
template <class H, class O> void Scrambler<H, O>::process(const H* in, O* out, unsigned n)
{
  while (n != 0)
  {
    O v = (O)(*t1_ ^ *in ^ *t2_);
    --t1_;
    --t2_;
    ++in;
    *out++ = v;
    *wr_ = (H)v;
    --wr_;
    if (wr_ < buf_)
    {
      resetHistoryIndexes();
      copyHistoryTail();
    }
    --n;
  }
}

/** @brief Scramble `n` symbols of a constant 1 input: v = y[n-tapNear] ^ y[n-tapFar] ^ 1. */
template <class H, class O> void Scrambler<H, O>::processAllOnes(O* out, unsigned n)
{
  while (n != 0)
  {
    O v = (O)(*t1_ ^ *t2_ ^ 1);
    --t1_;
    --t2_;
    *out++ = v;
    *wr_ = (H)v;
    --wr_;
    if (wr_ < buf_)
    {
      resetHistoryIndexes();
      copyHistoryTail();
    }
    --n;
  }
}

/** @brief Scramble `n` symbols of a constant 0 input: v = y[n-tapNear] ^ y[n-tapFar]. */
template <class H, class O> void Scrambler<H, O>::processAllZeros(O* out, unsigned n)
{
  while (n != 0)
  {
    O v = (O)(*t1_ ^ *t2_);
    --t1_;
    --t2_;
    *out++ = v;
    *wr_ = (H)v;
    --wr_;
    if (wr_ < buf_)
    {
      resetHistoryIndexes();
      copyHistoryTail();
    }
    --n;
  }
}

/* ---------------------------------------------------------------------------------------------- Descrambler<H,O> */

/** @brief Build the descrambler: same history layout as Scrambler, cleared with reset(0). */
template <class H, class O> Descrambler<H, O>::Descrambler(unsigned tapNear, unsigned tapFar, unsigned slack)
{
  tail_ = tapFar;
  buf_ = (H*)sysdep_malloc((tapFar + slack + 1) * (unsigned)sizeof(H));
  head0_ = buf_ + slack;
  t1_0_ = head0_ + tapNear;
  end_ = head0_ + tapFar;
  reset(0);
}

template <class H, class O> Descrambler<H, O>::~Descrambler()
{
  if (buf_)
    sysdep_free(buf_);
}

/** @brief Write position and both tap positions back to their start values. */
template <class H, class O> void Descrambler<H, O>::resetHistoryIndexes()
{
  wr_ = head0_;
  t1_ = t1_0_;
  t2_ = end_;
}

/** @brief Move the newest `tail_` history elements to head0_[1 .. tail_]; a forward copy (see the file header). */
template <class H, class O> void Descrambler<H, O>::copyHistoryTail()
{
  H* dst = head0_ + 1;
  const H* src = buf_;
  unsigned n = tail_;

  while (n-- != 0)
    *dst++ = *src++;
}

/** @brief Restart: indices to the start, all history cells head0_[1 .. tapFar] set to the low bit of `v`. */
template <class H, class O> void Descrambler<H, O>::reset(H v)
{
  resetHistoryIndexes();
  H bit = (H)(v & 1);

  for (H* p = head0_ + 1; p <= end_; ++p)
    *p = bit;
}

/** @brief Descramble one symbol: the input is stored in the history, v = in ^ in[n-tapNear] ^ in[n-tapFar] is returned. */
template <class H, class O> O Descrambler<H, O>::process(H in)
{
  *wr_ = in;
  O v = (O)(*wr_ ^ *t1_ ^ *t2_);
  --wr_;
  --t1_;
  --t2_;
  if (wr_ < buf_)
  {
    resetHistoryIndexes();
    copyHistoryTail();
  }
  return v;
}

/** @brief Descramble `n` symbols from `in` to `out`; as process(H) per symbol. */
template <class H, class O> void Descrambler<H, O>::process(const H* in, O* out, unsigned n)
{
  while (n != 0)
  {
    *wr_ = *in++;
    O v = (O)(*wr_ ^ *t1_ ^ *t2_);
    --t1_;
    --t2_;
    *out++ = v;
    --wr_;
    if (wr_ < buf_)
    {
      resetHistoryIndexes();
      copyHistoryTail();
    }
    --n;
  }
}

/* ------------------------------------------------------------------------------- Parallel differential en/decoder */

/** @brief Allocate `cap` lanes, all cleared; no lane is in use (`n_` == 0) until reset(). */
template <class T> ParallelDifferentialEncoder<T>::ParallelDifferentialEncoder(unsigned cap)
{
  state_ = 0;
  cap_ = cap;
  n_ = 0;
  state_ = (T*)sysdep_malloc(cap * (unsigned)sizeof(T));
  for (unsigned i = 0; i < cap; ++i)
    state_[i] = 0;
}

template <class T> ParallelDifferentialEncoder<T>::~ParallelDifferentialEncoder()
{
  if (state_)
    sysdep_free(state_);
}

/** @brief Use `n` lanes, each state set to `v`.  Returns 1 (nothing changed) when n exceeds the allocated lanes, else 0. */
template <class T> int ParallelDifferentialEncoder<T>::reset(unsigned n, T v)
{
  if (cap_ < n)
    return 1;
  for (unsigned i = 0; i < n; ++i)
    state_[i] = v;
  n_ = n;
  return 0;
}

/** @brief Encode one vector of n_ lanes: out[i] = state[i] ^ in[i], state[i] = out[i]. */
template <class T> void ParallelDifferentialEncoder<T>::process(T* in, T* out)
{
  for (unsigned i = 0; i < n_; ++i)
  {
    T y = (T)(state_[i] ^ in[i]);
    out[i] = y;
    state_[i] = y;
  }
}

/** @brief Allocate `cap` lanes, all cleared; no lane is in use (`n_` == 0) until reset(). */
template <class T> ParallelDifferentialDecoder<T>::ParallelDifferentialDecoder(unsigned cap)
{
  cap_ = cap;
  n_ = 0;
  state_ = (T*)sysdep_malloc(cap * (unsigned)sizeof(T));
  for (unsigned i = 0; i < cap; ++i)
    state_[i] = 0;
}

template <class T> ParallelDifferentialDecoder<T>::~ParallelDifferentialDecoder()
{
  if (state_)
    sysdep_free(state_);
}

/** @brief Use `n` lanes, each state set to `v`.  Returns 1 (nothing changed) when n exceeds the allocated lanes, else 0. */
template <class T> int ParallelDifferentialDecoder<T>::reset(unsigned n, T v)
{
  if (cap_ < n)
    return 1;
  for (unsigned i = 0; i < n; ++i)
    state_[i] = v;
  n_ = n;
  return 0;
}

/** @brief Decode one vector of n_ lanes: out[i] = in[i] ^ state[i], state[i] = in[i]. */
template <class T> void ParallelDifferentialDecoder<T>::process(T* in, T* out)
{
  for (unsigned i = 0; i < n_; ++i)
  {
    T x = in[i];
    out[i] = (T)(x ^ state_[i]);
    state_[i] = x;
  }
}

/* ---------------------------------------------------------------------------------- Serial differential en/decoder */

/** @brief Encode one symbol: y = state ^ in; the state becomes y, which is returned. */
template <class T> T SerialDifferentialEncoder<T>::process(T in)
{
  T y = (T)(state_ ^ in);
  state_ = y;
  return y;
}

/** @brief Decode one symbol: y = state ^ in is returned; the state becomes `in`. */
template <class T> T SerialDifferentialDecoder<T>::process(T in)
{
  T y = (T)(state_ ^ in);
  state_ = in;
  return y;
}

/* ------------------------------------------------------------------------------------------------------ Queue<T> */

/** @brief Allocate a queue that holds `capacity` elements (size_ = capacity + 1) and empty it. */
template <class T> Queue<T>::Queue(unsigned capacity)
{
  size_ = capacity + 1;
  buf_ = (T*)sysdep_malloc(size_ * (unsigned)sizeof(T));
  last_ = buf_ + size_ - 1;
  reset();
}

template <class T> Queue<T>::~Queue()
{
  if (buf_)
    sysdep_free(buf_);
}

/** @brief Empty the queue. */
template <class T> void Queue<T>::reset()
{
  rd_ = buf_;
  wr_ = buf_;
}

/* Element transfer: the 32-bit words are moved with integer moves, so a float is never loaded into the x87 and signalling NaN payloads survive.  `*p = v`
 * on a float can compile to an x87 load and store on i386 and quiet them: copy the bytes. */
#define P01_COPY_ELEM(dst, src) __builtin_memcpy((dst), (src), sizeof(*(dst)))
/* the by-value argument of write(T v) is copied through a 32-bit INTEGER temporary (see write(T v)) */
DSP_STATIC_ASSERT(sizeof(float) == sizeof(uint32_t), "Queue<float> element is a 32-bit word");

/* number of elements in the queue: (wr_ + size_ - rd_) % size_, an unsigned 32-bit division (size_ == 0 faults) */
#define P01_QUEUE_USED(q) ((unsigned)(((q)->wr_ + (q)->size_) - (q)->rd_) % (q)->size_)

/** @brief Append one element.  Returns 0, or -1 when the queue is full (nothing stored). */
template <class T> int Queue<T>::write(T v)
{
  unsigned room = size_ - P01_QUEUE_USED(this);

  if (room - 1 == 0)
    return -1;

  T* p = wr_;
  uint32_t word;

  __builtin_memcpy(&word, &v, sizeof word);
  OPTIMIZER_HIDE_VAR(word); /* keep it in an integer register: without this gcc -O2 still picks an x87 load and store (quiets a signalling NaN) */
  __builtin_memcpy(p, &word, sizeof word);
  wr_ = (p == last_) ? buf_ : p + 1;
  return 0;
}

/**
 * @brief Append `n` elements.  Returns 0, or -1 when fewer than `n` free slots remain (nothing stored).
 *
 * The copy is split at the end of the block when it wraps.
 */
template <class T> int Queue<T>::write(T* src, unsigned n)
{
  unsigned room = size_ - P01_QUEUE_USED(this);

  if (room - 1 < n)
    return -1;

  T* p = wr_;
  int tail = (int)(last_ - p) + 1;

  if (tail < (int)n)
  {
    int done = 0;

    if (0 < tail)
    {
      for (int i = tail; i != 0; --i)
      {
        P01_COPY_ELEM(p, src);
        ++p;
        ++src;
      }
      done = tail;
    }
    T* w = buf_;

    if (done < (int)n)
    {
      w = buf_;
      for (int i = done; i < (int)n; ++i)
      {
        P01_COPY_ELEM(w, src);
        ++w;
        ++src;
      }
    }
    wr_ = w;
    return 0;
  }
  if ((int)n > 0)
  {
    for (int i = (int)n; i != 0; --i)
    {
      P01_COPY_ELEM(p, src);
      ++p;
      ++src;
    }
    wr_ = p;
  }
  if (p == last_ + 1)
    wr_ = buf_;
  return 0;
}

/**
 * @brief Remove `n` elements into `dst`.  Returns 0, or -1 when fewer than `n` elements are queued (nothing removed).
 *
 * The copy is split at the end of the block when it wraps.
 */
template <class T> int Queue<T>::read(T* dst, unsigned n)
{
  unsigned used = P01_QUEUE_USED(this);

  if (used < n)
    return -1;

  T* p = rd_;
  int tail = (int)(last_ - p) + 1;

  if (tail < (int)n)
  {
    int done = 0;

    if (0 < tail)
    {
      for (int i = tail; i != 0; --i)
      {
        P01_COPY_ELEM(dst, p);
        ++dst;
        ++p;
      }
      done = tail;
    }
    T* r = buf_;

    if (done < (int)n)
    {
      r = buf_;
      for (int i = done; i < (int)n; ++i)
      {
        P01_COPY_ELEM(dst, r);
        ++dst;
        ++r;
      }
    }
    rd_ = r;
    return 0;
  }
  if ((int)n > 0)
  {
    for (int i = (int)n; i != 0; --i)
    {
      P01_COPY_ELEM(dst, p);
      ++dst;
      ++p;
    }
    rd_ = p;
  }
  if (p == last_ + 1)
    rd_ = buf_;
  return 0;
}

/* ------------------------------------------------------------------------------------ explicit instantiations
 * Exactly the 51 member functions that are used (a class-wide `template class X<...>;` would also emit members that nothing uses). */

#define P01_SCR_COMMON(H, O)                                                                                                                                   \
  template Scrambler<H, O>::Scrambler(unsigned, unsigned, unsigned);                                                                                           \
  template Scrambler<H, O>::~Scrambler();                                                                                                                      \
  template void Scrambler<H, O>::reset(H);                                                                                                                     \
  template void Scrambler<H, O>::resetHistoryIndexes();                                                                                                        \
  template void Scrambler<H, O>::copyHistoryTail()
P01_SCR_COMMON(unsigned char, unsigned char);
template unsigned char Scrambler<unsigned char, unsigned char>::process(unsigned char);
template void Scrambler<unsigned char, unsigned char>::process(const unsigned char*, unsigned char*, unsigned);
template void Scrambler<unsigned char, unsigned char>::processAllOnes(unsigned char*, unsigned);
template void Scrambler<unsigned char, unsigned char>::processAllZeros(unsigned char*, unsigned);
P01_SCR_COMMON(int, unsigned char);
template void Scrambler<int, unsigned char>::process(const int*, unsigned char*, unsigned);
P01_SCR_COMMON(unsigned char, int);
template int Scrambler<unsigned char, int>::process(unsigned char);

#define P01_DESCR_COMMON(H, O)                                                                                                                                 \
  template Descrambler<H, O>::Descrambler(unsigned, unsigned, unsigned);                                                                                       \
  template Descrambler<H, O>::~Descrambler();                                                                                                                  \
  template void Descrambler<H, O>::reset(H);                                                                                                                   \
  template void Descrambler<H, O>::resetHistoryIndexes();                                                                                                      \
  template void Descrambler<H, O>::copyHistoryTail();                                                                                                          \
  template O Descrambler<H, O>::process(H)
P01_DESCR_COMMON(unsigned char, int);
template void Descrambler<unsigned char, int>::process(const unsigned char*, int*, unsigned);
P01_DESCR_COMMON(int, int);

#define P01_PAR(CLS)                                                                                                                                           \
  template CLS<unsigned char>::CLS(unsigned);                                                                                                                  \
  template CLS<unsigned char>::~CLS();                                                                                                                         \
  template int CLS<unsigned char>::reset(unsigned, unsigned char);                                                                                             \
  template void CLS<unsigned char>::process(unsigned char*, unsigned char*)
P01_PAR(ParallelDifferentialEncoder);
P01_PAR(ParallelDifferentialDecoder);

template unsigned char SerialDifferentialEncoder<unsigned char>::process(unsigned char);
template unsigned char SerialDifferentialDecoder<unsigned char>::process(unsigned char);
template int SerialDifferentialDecoder<int>::process(int);

template Queue<float>::Queue(unsigned);
template Queue<float>::~Queue();
template void Queue<float>::reset();
template int Queue<float>::write(float);
template int Queue<float>::write(float*, unsigned);
template int Queue<float>::read(float*, unsigned);
