/**
 * @file vpcm_getmp_alias.cpp
 * @brief C++-linkage forwarder getMPrecvdBits(tagV34Object *) for the phase 3 / 4 transmitters.
 *
 * vpcm_phase34_reneg.cpp calls getMPrecvdBits() with C++ linkage (mangled name _Z14getMPrecvdBitsP12tagV34Object) and, at 32 bits, the regparm(1)
 * convention; a test harness can redirect that symbol at run time.  The real work is done by the ordinary cdecl C function of v34_modem_init.c,
 * which this file calls.  At 64 bits the regparm attribute is ignored and this is a plain forwarder.
 */
#include "v34/v34_object.h"
extern "C" void getMPrecvdBits(void* obj);
void getMPrecvdBits(tagV34Object* obj) __attribute__((regparm(1)));

/**
 * @brief Forward to the C function getMPrecvdBits(void *) of v34_modem_init.c.
 * @param obj  session object
 */
void getMPrecvdBits(tagV34Object* obj) { ::getMPrecvdBits((void*)obj); }
