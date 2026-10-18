/**
 * @file fax_modem_views.h
 * @brief Accessors that let the scrambler / encoder wrappers of fax_modem_wrappers.c reach the sub-objects inside the V.17 / V.27ter / V.29 modem objects.
 *
 * The wrappers (ScrambleData*, DescrambleData*, SeedScrambler*, SetEncoder*, SetScramblerV27) receive the modem object as an opaque `void *`; each
 * sub-object they need (scrambler state, symbol-mapping encoder, encoder mode, rate) is found through ONE accessor per sub-object declared here.
 *
 * - 32-bit builds: the accessors are inline functions over the raw layout of the modem objects (the modem object holds a pointer to its transmit or
 *   receive state; the sub-object sits at a fixed offset inside that state):
 *     V.17 TX  object + 0x28 -> state: scrambler +0x1c, encoder +0x34 (SetEncoderV17 writes encoder.x = state + 0x3a), encoder mode (u16) +0x8c;
 *     V.17 RX  object + 0x60 -> state: scrambler +0x4f8c
 *     V.29 TX  object + 0x24 -> state: scrambler +0x1c, encoder select (int) +0x38;  V.29 RX  object + 0x50 -> state: scrambler +0x4f3c
 *     V.27ter  TX object + 0x28 -> state: scrambler +0x1c;  rate block object + 0x24 -> short +0xc (index into V27TX_SDM_NUM_BITS);
 *              RX object + 0x54 -> state: scrambler +0x3c
 * - 64-bit builds: the accessors are only declared; the modem files (fax_v17_tx.c, fax_v17_rx.c, fax_v27_tx.c, fax_v27_rx.c, fax_v29_tx.c, fax_v29_rx.c)
 *   define them over their typed members.
 * - The unit-test harness defines the accessors over its own typed stand-in objects.
 */
#ifndef DSPLIBS_PORT_FAX_VIEWS_H
#define DSPLIBS_PORT_FAX_VIEWS_H
#include "fax_common.h"

#if UINTPTR_MAX == 0xffffffffu
static inline char* faxw_at(void* obj, unsigned off) { return *(char**)((char*)obj + off); }
static inline FaxSdm* faxw_v17_txsdm(void* o) { return (FaxSdm*)(faxw_at(o, 0x28) + 0x1c); }
static inline FaxSmcV17* faxw_v17_txsmc(void* o) { return (FaxSmcV17*)(faxw_at(o, 0x28) + 0x34); }
static inline unsigned short* faxw_v17_encmode(void* o) { return (unsigned short*)(faxw_at(o, 0x28) + 0x8c); }
static inline FaxSdm* faxw_v17_rxsdm(void* o) { return (FaxSdm*)(faxw_at(o, 0x60) + 0x4f8c); }
static inline FaxSdm* faxw_v29_txsdm(void* o) { return (FaxSdm*)(faxw_at(o, 0x24) + 0x1c); }
static inline int* faxw_v29_encsel(void* o) { return (int*)(faxw_at(o, 0x24) + 0x38); }
static inline FaxSdm* faxw_v29_rxsdm(void* o) { return (FaxSdm*)(faxw_at(o, 0x50) + 0x4f3c); }
static inline FaxSdmV27* faxw_v27_txsdm(void* o) { return (FaxSdmV27*)(faxw_at(o, 0x28) + 0x1c); }
static inline short* faxw_v27_rate(void* o) { return (short*)(faxw_at(o, 0x24) + 0xc); }
static inline FaxSdmV27* faxw_v27_rxsdm(void* o) { return (FaxSdmV27*)(faxw_at(o, 0x54) + 0x3c); }
#else
FaxSdm* faxw_v17_txsdm(void* o);
FaxSmcV17* faxw_v17_txsmc(void* o);
unsigned short* faxw_v17_encmode(void* o);
FaxSdm* faxw_v17_rxsdm(void* o);
FaxSdm* faxw_v29_txsdm(void* o);
int* faxw_v29_encsel(void* o);
FaxSdm* faxw_v29_rxsdm(void* o);
FaxSdmV27* faxw_v27_txsdm(void* o);
short* faxw_v27_rate(void* o);
FaxSdmV27* faxw_v27_rxsdm(void* o);
#endif
#endif
