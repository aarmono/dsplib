/**
 * @file cid_unused.c
 * @brief Caller-ID helpers that nothing in the library calls: two message-parameter searches and the byte framer step.
 *
 * _look_for() and _look_for_other_than() walk a caller-ID message (type, total length, then [type][length][data] parameters from index 2) and return the
 * index of a parameter (-1: none); the index and the end test are 16 bit.  pack_next_bit() is the byte framer of the FSK receiver as a function of its own
 * (CidFsk state 0 mark run, 3 start bit, 1 stop bit, 2 data bits least significant first; a finished byte goes to msg[count++]); cid_modem() (cid_modem.c)
 * contains the same state machine inline.  They are compiled and linked but no function in src/ references them.
 */
#include "cid.h"

/**
 * @brief Find the first parameter of a given type in a caller-ID message.
 * @param buf  message: buf[1] = length of the parameter area, parameters from index 2
 * @param c    parameter type to find
 * @return the index of the parameter's type byte, or -1 if there is none
 * @warning The length byte is a signed char.  A parameter length of -2 makes the walk stand still (an endless loop), a length below -2 steps it backwards
 *          and reads in front of the buffer.
 */
int _look_for(const signed char* buf, unsigned char c)
{
  int idx = 2, end = buf[1];
  while ((short)idx < (short)end)
  {
    if ((unsigned char)buf[idx] == c)
      return idx;
    idx = (short)(buf[idx + 1] + idx + 2);
  }
  return -1;
}

/**
 * @brief Find the first parameter at or after `start` that is not of type 1, 7 or 2 and is not at index `skip`.
 * @param buf    message (see _look_for())
 * @param start  index to start at (16 bit)
 * @param skip   index to ignore
 * @return the index of the parameter's type byte, or -1
 * @warning The same hazard with a parameter length of -2 or less as _look_for().
 */
int _look_for_other_than(const signed char* buf, short start, int skip)
{
  int idx = start, end = buf[1];
  while ((short)idx < (short)end)
  {
    unsigned char t = (unsigned char)buf[idx];
    if (t != 1 && t != 7 && t != 2 && idx != skip)
      return idx;
    idx = (short)(buf[idx + 1] + idx + 2);
  }
  return -1;
}

/**
 * @brief Advance the byte framer of an FSK receiver by one demodulated bit.
 * @param bit  the bit (0 or 1)
 * @param o    receiver state (state, ones, acc, bitIdx, count and msg[] are updated)
 * @warning `count` is not checked against the size of msg[]: more than 0x78 bytes write behind it.
 */
void pack_next_bit(short bit, CidFsk* o)
{
  int thr = o->cfgThr ? (int)(unsigned short)o->cfgThr : 0;
  switch (o->state)
  {
  case 0:
  {
    short v = (short)(o->ones + (short)(bit - 1 == 0 ? 1 : -1));
    o->ones = v;
    if (o->ones > 0xf)
    {
      o->state = 3;
      o->acc = 0;
      o->bitIdx = 0;
    }
    return;
  }
  case 3:
    if (bit != 0)
    {
      o->bitIdx = 0;
      return; /* 0 never equals thr + 1 */
    }
    o->bitIdx = (short)(o->bitIdx + 1);
    if (o->bitIdx == thr + 1)
    {
      o->bitIdx = (short)thr;
      o->state = 2;
    }
    return;
  case 1:
    if (bit == 0)
    {
      o->state = 2;
      o->acc = 0;
      o->bitIdx = 0;
    }
    return;
  default:
  {
    int sh = o->bitIdx;
    short acc = o->acc;
    short nb = (short)(o->bitIdx + 1);
    o->bitIdx = nb;
    o->acc = (short)(((unsigned)(int)bit << (sh & 31)) | (unsigned short)acc);
    if (nb == 8)
    {
      int k = o->count;
      o->state = 1;
      o->count = (short)(k + 1);
      o->msg[k] = (unsigned char)o->acc;
    }
    return;
  }
  }
}
