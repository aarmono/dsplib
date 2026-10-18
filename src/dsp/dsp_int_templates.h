/**
 * @file dsp_int_templates.h
 * @brief Integer / bit utility class templates of the V.90 / V.92 modems: scramblers, differential encoders and decoders, and a float ring buffer.
 *
 * Definitions and the explicit instantiations are in dsp_int_templates.cpp; only the member functions instantiated there exist (another combination of
 * template arguments or another member fails to link):
 *
 *   Scrambler<H, O>       H = history element type, O = output element type: <uchar,uchar> <int,uchar> <uchar,int>
 *   Descrambler<H, O>     <uchar,int> <int,int>
 *   ParallelDifferentialEncoder<uchar> / ParallelDifferentialDecoder<uchar>
 *   SerialDifferentialEncoder<uchar> / SerialDifferentialDecoder<uchar> / SerialDifferentialDecoder<int>
 *   Queue<float>
 *
 * The members are laid out so that the 32-bit layout is fixed (the asserts at the end of the file); the owner classes in src/vpcm embed these objects by
 * value and call their methods, so all data members are public.  None of the classes has a virtual function or a base class, and the only resource any of
 * them owns is one heap buffer (freed by the destructor).  They are not copyable.
 */
#ifndef DSP_CLASSES_P01_H
#define DSP_CLASSES_P01_H

#include "core/dsp_cxx.h"

/**
 * Scrambler<H, O>: self-synchronising scrambler  y[n] = x[n] ^ y[n-tapNear] ^ y[n-tapFar]  (the V.34 / V.90 generator 1 + x^-18 + x^-23 is tapNear 18,
 * tapFar 23), over a history that is written DOWNWARDS.  Heap block of (tapFar + slack + 1) elements H:
 *
 *     buf_ ................. head0_ ......................... end_
 *     [0 .. slack-1]  slack  [slack]  [slack+1 .. slack+tapFar]
 *
 * `wr_` is the next write position (starts at head0_, walks DOWN); `t1_` / `t2_` are the two tap read positions (start at head0_ + tapNear and
 * head0_ + tapFar, walk down in lock step), so *(t) = y[n-tap].  When `wr_` falls below `buf_`, copyHistoryTail() copies the `tail_` (= tapFar) elements at
 * buf_[0 .. tail_-1] (the newest outputs) up to head0_[1 .. tail_], and resetHistoryIndexes() moves wr_ / t1_ / t2_ back to their start values.  Every
 * `slack + 1` outputs therefore cost one copy of tapFar elements.
 *
 * Construction: buf_ = alloc((tapFar + slack + 1) * sizeof H); head0_ = buf_ + slack; t1_0_ = head0_ + tapNear; t2_0_ = end_ = head0_ + tapFar;
 * tail_ = tapFar; reset(0).
 * @warning There are no bounds checks: the allocation is not checked, and copyHistoryTail() is a FORWARD element copy whose source [buf_, buf_ + tail_)
 *          and destination [head0_ + 1, head0_ + tail_] overlap when slack + 1 < tapFar, which smears the history (run with tapNear 18, tapFar 23: slack 5
 *          deviates from the plain recurrence at the 7th output, slack 22 and 99 do not); callers use slack >= tapFar - 1 (the V.90 modem uses 18 / 23 / 99).
 */
template <class H, class O> class Scrambler
{
public:
  Scrambler(unsigned tapNear, unsigned tapFar, unsigned slack);
  ~Scrambler();

  void reset(H v);                               ///< history cells head0_[1 .. tapFar] = v & 1, indices back to the start
  O process(H in);                               ///< one symbol (<uchar,uchar> and <uchar,int> only)
  void process(const H* in, O* out, unsigned n); ///< `n` symbols (<uchar,uchar> and <int,uchar> only)
  void processAllOnes(O* out, unsigned n);       ///< `n` outputs for a constant 1 input stream (<uchar,uchar> only)
  void processAllZeros(O* out, unsigned n);      ///< `n` outputs for a constant 0 input stream (<uchar,uchar> only)

  void resetHistoryIndexes(); ///< wr_, t1_, t2_ back to their start values
  void copyHistoryTail();     ///< move the newest tail_ elements to the head of the block

  H* buf_;        ///< heap block
  H* head0_;      ///< buf_ + slack: initial write position
  H* t1_0_;       ///< head0_ + tapNear: initial near tap
  H* end_;        ///< head0_ + tapFar: last history cell = initial far tap
  H* wr_;         ///< write position (walks down)
  H* t1_;         ///< near tap read position
  H* t2_;         ///< far tap read position
  unsigned tail_; ///< tapFar: number of elements copyHistoryTail() moves
};

/**
 * Descrambler<H, O>: the inverse of Scrambler with the same history layout; the history holds the INPUT symbols: out = in ^ in[n-tapNear] ^ in[n-tapFar].
 * The new input is stored at wr_ first and read back, so a tapNear of 0 reads the symbol just stored.  Same preconditions as Scrambler.
 */
template <class H, class O> class Descrambler
{
public:
  Descrambler(unsigned tapNear, unsigned tapFar, unsigned slack);
  ~Descrambler();

  void reset(H v);
  O process(H in);                               ///< one symbol (<uchar,int> and <int,int>)
  void process(const H* in, O* out, unsigned n); ///< `n` symbols (<uchar,int> only)

  void resetHistoryIndexes();
  void copyHistoryTail();

  H* buf_;        ///< heap block
  H* head0_;      ///< initial write position
  H* t1_0_;       ///< initial near tap
  H* end_;        ///< last history cell = initial far tap
  H* wr_;         ///< write position (walks down)
  H* t1_;         ///< near tap read position
  H* t2_;         ///< far tap read position
  unsigned tail_; ///< number of elements copyHistoryTail() moves
};

/**
 * Differential encoder over `n_` parallel lanes (of `cap_` allocated): out[i] = state[i] ^ in[i], and the state takes the output.
 */
template <class T> class ParallelDifferentialEncoder
{
public:
  explicit ParallelDifferentialEncoder(unsigned cap);
  ~ParallelDifferentialEncoder();
  int reset(unsigned n, T v);  ///< 1 if n > cap_ (nothing changed), else fills n lanes with v, n_ = n, returns 0
  void process(T* in, T* out); ///< n_ lanes

  T* state_;     ///< heap block of cap_ lanes
  unsigned cap_; ///< allocated lanes
  unsigned n_;   ///< lanes in use
};

/** Differential decoder over `n_` parallel lanes: out[i] = in[i] ^ state[i], and the state takes the input. */
template <class T> class ParallelDifferentialDecoder
{
public:
  explicit ParallelDifferentialDecoder(unsigned cap);
  ~ParallelDifferentialDecoder();
  int reset(unsigned n, T v);
  void process(T* in, T* out);

  T* state_;     ///< heap block of cap_ lanes
  unsigned cap_; ///< allocated lanes
  unsigned n_;   ///< lanes in use
};

/** Serial differential encoder: state = state ^ in, returned.  No constructor: the owner class initialises `state_`. */
template <class T> class SerialDifferentialEncoder
{
public:
  T process(T in);
  T state_;
};

/** Serial differential decoder: returns state ^ in, then state = in.  No constructor: the owner class initialises `state_`. */
template <class T> class SerialDifferentialDecoder
{
public:
  T process(T in);
  T state_;
};

/**
 * Queue<T>: ring buffer of (size_ - 1) usable elements (one slot is always kept free).
 *   buf_ .. last_ (= buf_ + size_ - 1); rd_ / wr_ in [buf_, last_]; used = (wr_ + size_ - rd_) % size_.
 * write() / read() return 0 on success and -1 when the queue has too little room / too few elements (nothing is transferred).  Elements are moved as 32-bit
 * words, so a signalling NaN is kept.
 * @warning A capacity of UINT_MAX gives size_ == 0, and every operation then divides by zero.  The allocation is not checked.
 */
template <class T> class Queue
{
public:
  explicit Queue(unsigned capacity);
  ~Queue();
  void reset(); ///< rd_ = wr_ = buf_ (empties the queue)
  int write(T v);
  int write(T* src, unsigned n);
  int read(T* dst, unsigned n);

  T* buf_;        ///< heap block of size_ elements
  T* last_;       ///< &buf_[size_ - 1]
  T* rd_;         ///< read position
  T* wr_;         ///< write position
  unsigned size_; ///< capacity + 1
};

/* 32-bit layout asserts.  The sizes are cross-checked against the owner classes (e.g. V92Modulator holds a Scrambler<int,uchar> and the next member is 0x20
 * bytes further). */
typedef Scrambler<unsigned char, unsigned char> P01_ScramblerHH;
typedef Scrambler<int, unsigned char> P01_ScramblerIH;
typedef Scrambler<unsigned char, int> P01_ScramblerHI;
typedef Descrambler<unsigned char, int> P01_DescramblerHI;
typedef Descrambler<int, int> P01_DescramblerII;
typedef ParallelDifferentialEncoder<unsigned char> P01_PDE;
typedef ParallelDifferentialDecoder<unsigned char> P01_PDD;
typedef SerialDifferentialEncoder<unsigned char> P01_SDE_H;
typedef SerialDifferentialDecoder<unsigned char> P01_SDD_H;
typedef SerialDifferentialDecoder<int> P01_SDD_I;
typedef Queue<float> P01_QueueF;

#define DSP_P01_SCR_LAYOUT(T)                                                                                                                                  \
  DSP_ASSERT_LAYOUT_I386(sizeof(T) == 0x20, "sizeof " #T);                                                                                                     \
  DSP_ASSERT_LAYOUT_I386(offsetof(T, buf_) == 0x00 && offsetof(T, head0_) == 0x04, "offsets " #T);                                                             \
  DSP_ASSERT_LAYOUT_I386(offsetof(T, t1_0_) == 0x08 && offsetof(T, end_) == 0x0c, "offsets " #T);                                                              \
  DSP_ASSERT_LAYOUT_I386(offsetof(T, wr_) == 0x10 && offsetof(T, t1_) == 0x14, "offsets " #T);                                                                 \
  DSP_ASSERT_LAYOUT_I386(offsetof(T, t2_) == 0x18 && offsetof(T, tail_) == 0x1c, "offsets " #T)
DSP_P01_SCR_LAYOUT(P01_ScramblerHH);
DSP_P01_SCR_LAYOUT(P01_ScramblerIH);
DSP_P01_SCR_LAYOUT(P01_ScramblerHI);
DSP_P01_SCR_LAYOUT(P01_DescramblerHI);
DSP_P01_SCR_LAYOUT(P01_DescramblerII);

#define DSP_P01_PAR_LAYOUT(T)                                                                                                                                  \
  DSP_ASSERT_LAYOUT_I386(sizeof(T) == 0x0c, "sizeof " #T);                                                                                                     \
  DSP_ASSERT_LAYOUT_I386(offsetof(T, state_) == 0 && offsetof(T, cap_) == 4 && offsetof(T, n_) == 8, "offsets " #T)
DSP_P01_PAR_LAYOUT(P01_PDE);
DSP_P01_PAR_LAYOUT(P01_PDD);

DSP_ASSERT_LAYOUT_I386(sizeof(P01_SDE_H) == 1 && sizeof(P01_SDD_H) == 1 && sizeof(P01_SDD_I) == 4, "sizeof SerialDifferential");
DSP_ASSERT_LAYOUT_I386(offsetof(P01_SDD_I, state_) == 0, "offset SerialDifferentialDecoder<int>");

DSP_ASSERT_LAYOUT_I386(sizeof(P01_QueueF) == 0x14, "sizeof Queue<float>");
DSP_ASSERT_LAYOUT_I386(offsetof(P01_QueueF, buf_) == 0x00 && offsetof(P01_QueueF, last_) == 0x04 && offsetof(P01_QueueF, rd_) == 0x08 &&
                           offsetof(P01_QueueF, wr_) == 0x0c && offsetof(P01_QueueF, size_) == 0x10,
                       "offsets Queue<float>");

#endif /* DSP_CLASSES_P01_H */
