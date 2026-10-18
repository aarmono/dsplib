/**
 * @file fax_class1.h
 * @brief The fax class 1 layer (fax_class1.c) and the glue object behind the entry points slmodemd calls (fax_api.c).
 *
 * - fax_class1_create() / fax_class1_progress() / fax_class1_command() / fax_class1_delete() operate on the T.30 object (FaxT30, fax_t30.h) with its two
 *   V.21 HDLC channels and the state-function table fax_class1_state_functions.
 * - FAX_create() / FAX_process() / FAX_delete() / FAX_class1_command() are the host entry points; they work on the FaxGlue object (ring buffers, sample
 *   rate converters, DTE byte buffers) which owns the FaxT30.
 *
 * Types:
 * - FaxClass1Cfg: the argument of fax_class1_create().  `mode` is FAXC1_ROLE_ANSWER (state: answer tone) or anything else (originate: look for carrier
 *   and send the calling tone); `s7` the seconds a receiver waits for a carrier; `cdFlag` the initial carrier flag of the data receiver; `hpFilter`
 *   nonzero enables the input high-pass filter; `answerMs` the answer tone length in ms (frames of 20 ms; 0 keeps 150 frames); `cngOff` zero sends the
 *   calling tone.
 * - FaxGlue: what FAX_create() returns to slmodemd as an opaque `void *`: the modem, the class 1 object, the `online` flag that the progress status
 *   switches, the DTE buffers, the two RcFixed converters (NULL at 8000 Hz), two 160-sample staging buffers and two 384-sample rings.
 */
#ifndef DSPLIBS_PORT_FAX_CLASS1_H
#define DSPLIBS_PORT_FAX_CLASS1_H
#include <stddef.h>
#include <stdint.h>

#include "fax_t30.h"
#include "core/rcfixed_resampler.h"

struct modem;

/** Configuration of fax_class1_create(). */
typedef struct FaxClass1Cfg
{
  int mode;     ///< FAXC1_ROLE_ANSWER = answer side, anything else originate
  int s7;       ///< seconds to wait for a carrier
  int cdFlag;   ///< initial carrier flag of the data receiver (FaxT30.cd_flag)
  int hpFilter; ///< nonzero: high-pass filter on the input (FaxT30.hp_on)
  int answerMs; ///< answer tone length in ms (0: 150 frames)
  int cngOff;   ///< zero: the calling tone is sent (FaxT30.cng_on = cngOff == 0)
} FaxClass1Cfg;

/** One entry of a debug-name table: number and text. */
typedef struct FaxClass1Name
{
  int key;
  const char* name;
} FaxClass1Name;
extern const FaxClass1Name fax_class1_status_names[11]; ///< names of the class 1 status codes
extern const FaxClass1Name fax_class1_state_names[20];  ///< names of the class 1 states
extern const FaxClass1Name fax_class1_cmd_names[6];     ///< names of the class 1 commands
extern const short fax_hp_coeff[10];                    ///< input high-pass filter, two biquad sections
extern FaxT30StateFn fax_class1_state_functions[19];    ///< the state functions, indexed by FaxT30.state; filled by fax_class1_create()

/** The class 1 commands accepted by fax_class1_command(). */
enum
{
  FAXC1_TH = 0,
  FAXC1_TM = 1,
  FAXC1_RM = 2,
  FAXC1_RH = 3,
  FAXC1_TS = 4,
  FAXC1_RS = 5
};

FaxT30* fax_class1_create(FaxT30* o, const FaxClass1Cfg* cfg);
int fax_class1_progress(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
int fax_class1_command(FaxT30* o, int cmd, int param, int extra);
int fax_class1_delete(FaxT30* o);

/* ---------------------------------------------------------------- the glue object */
/** A ring of two blocks of samples (the input ring and the output ring of FAX_process()). */
typedef struct FaxRing
{
  unsigned int count; ///< samples held (the output ring starts with one block of silence)
  unsigned int wr;    ///< write position (input ring)
  unsigned int rd;    ///< read position (output ring)
  unsigned int half;  ///< offset of the half being processed / filled: 0 or blk
  short data[384];    ///< sample storage; holds two blocks, so it fits srate <= 9600 Hz only (2 * blk <= 384)
} FaxRing;

/** The fax session object behind the opaque pointer of FAX_create(). */
typedef struct FaxGlue
{
  struct modem* modem;           ///< the slmodemd modem object (tty transfers, S registers)
  FaxT30* c1;                    ///< the class 1 object
  int online;                    ///< nonzero between CONNECT and the end of the data connection: DTE bytes are exchanged
  int tty_cnt;                   ///< bytes the tty may send / the last input count
  unsigned char tty_out[0x1000]; ///< bytes for the DTE (the receive states fill it)
  unsigned char tty_in[0x1000];  ///< bytes from the DTE
  RcFixed* rcIn;                 ///< input converter to 8000 Hz (NULL at 8000 Hz)
  RcFixed* rcOut;                ///< output converter from 8000 Hz (NULL at 8000 Hz)
  short rxbuf[0xa0];             ///< resampled input block
  short txbuf[0xa0];             ///< the class 1 output block before rcOut
  unsigned int blk;              ///< samples per block = srate / 50
  FaxRing out;                   ///< output ring
  FaxRing in;                    ///< input ring
} FaxGlue;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(FaxClass1Cfg) == 0x18, "FaxClass1Cfg");
_Static_assert(sizeof(FaxRing) == 0x310 && offsetof(FaxRing, data) == 0x10, "FaxRing");
_Static_assert(sizeof(FaxGlue) == 0x28bc && offsetof(FaxGlue, c1) == 4 && offsetof(FaxGlue, online) == 8 && offsetof(FaxGlue, tty_cnt) == 0xc &&
                   offsetof(FaxGlue, tty_out) == 0x10 && offsetof(FaxGlue, tty_in) == 0x1010 && offsetof(FaxGlue, rcIn) == 0x2010 &&
                   offsetof(FaxGlue, rcOut) == 0x2014 && offsetof(FaxGlue, rxbuf) == 0x2018 && offsetof(FaxGlue, txbuf) == 0x2158 &&
                   offsetof(FaxGlue, blk) == 0x2298 && offsetof(FaxGlue, out) == 0x229c && offsetof(FaxGlue, in) == 0x25ac,
               "FaxGlue");
_Static_assert(sizeof(fax_class1_state_functions) == 76, "class1_state_functions");
#endif

/* the entry points slmodemd links (documented at their definitions in fax_api.c) */
void* FAX_create(struct modem* m, unsigned int caller, unsigned int srate);
int FAX_process(void* obj, void* in, void* out, int count);
void FAX_delete(void* obj);
int FAX_class1_command(void* obj, unsigned int cmd, unsigned int param);
#endif
