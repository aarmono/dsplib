/**
 * @file vpcm_split.h
 * @brief struct Split: the three arguments of a "%c%d.%0Nd" print of a float.
 *
 * Holds the sign character, the integer part and the fraction digits of a value.  splitValue() (vpcm_split_value.h) computes them.  The V.90 / V.92 / PCM
 * debug prints format their floats this way instead of with the C library's float printing.
 */
#ifndef VPCM_SPLIT_H
#define VPCM_SPLIT_H

/** @brief Arguments of a "%c%d.%0Nd" print. */
struct Split
{
  int sign, ip, fp; ///< sign character ('+' 0x2b or '-' 0x2d), integer part of |value|, fraction digits |trunc((value - trunc(value)) * scale)|
};

#endif /* VPCM_SPLIT_H */
