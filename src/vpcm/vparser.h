/**
 * @file vparser.h
 * @brief The parameter-file reader of the V.90 / V.92 modem: Vparser_read_int() / Vparser_read_float().
 *
 * Both are stubs that read nothing and return 0 (pcm_leaf_classes.cpp); pcm_parameters.cpp and pcm_parameters_load.cpp call them for every parameter
 * key, with the parameter-file path string as `parser`.
 */
#ifndef VPCM_VPARSER_H
#define VPCM_VPARSER_H

#include "core/dsp_cxx.h"

extern "C"
{
  /** Read the integer parameter `name` from the parameter file `parser` into *value. */
  int Vparser_read_int(void* parser, const char* name, int* value);
  /** Read the float parameter `name` from the parameter file `parser` into *value. */
  int Vparser_read_float(void* parser, const char* name, float* value);
}

#endif /* VPCM_VPARSER_H */
