/**
 * @file cid.h
 * @brief Caller-ID receiver: state objects and prototypes (FSK and DTMF detectors, the cid_* object and the entry points of the host).
 *
 * Caller ID reaches the modem between the first and second ring as either an FSK burst (1200 baud, preceded by a 1200 Hz seizure tone and a run of mark bits)
 * or a string of DTMF digits.  The code is split in two files:
 *   - cid_detect.c: the signal detectors.  DTMF: band_pass() (is there a tone in this block?), DTMF_MTD_detect() (which digit?), dtmf_modem() (the digit state
 *     machine over blocks), reset_dtmf().  FSK: CID_FSD_demodulate() (samples to bits) and CID_MTD_detect() (is the 1200 Hz seizure tone present?).
 *   - cid_modem.c: the FSK receiver cid_modem() (tone gate, gain, demodulator, byte framer, message checksum), object creation / reset, the formatting of the
 *     result strings, the cid_* object API, and the three functions the host calls: CID_create(), CID_delete(), CID_process().
 *
 * Objects.
 *   CidDtmf   state of the DTMF detector (digits collected, filter states, block buffers).
 *   CidFsk    state of the FSK receiver: the polyphase filter (FpmMrf), the demodulator state, the byte framer and the received message.
 *   CidObj    the cid_* object: owns one CidDtmf and/or one CidFsk according to its mode (CidMode), a block buffer and the result strings.
 *   CidHandle what CID_create() returns: the host's modem object and the CidObj.
 * Data flow.  CID_process() cuts the host's samples into chunks of at most 192, cid_progress() collects them into frames of 160 (8000 Hz) or 192 (9600 Hz)
 * samples and runs cid_modem() and / or dtmf_modem() on every full frame; when a message is complete cid_get_strings() formats it and CID_process() sends the
 * strings to the host's tty.
 *
 * Layout.  Every member of CidDtmf and of the part of CidFsk behind the filter keeps the 32-bit layout asserted at the end of the structs (for a 32-bit build).
 * CidFsk contains an FpmMrf with pointers, so on a 64-bit build the members behind it move: nothing may address these objects by a raw byte offset.  The
 * members from `tc` to the end of CidFsk are 16-bit / byte only and contiguous on every host (the helper mb() in cid_modem.c).
 */
#ifndef DSPLIBS_PORT_CID_OBJ_H
#define DSPLIBS_PORT_CID_OBJ_H
#include <stddef.h>
#include <stdint.h>
#include "fpm/fpm.h"
#include "core/dsp_host.h"

/**
 * Which detector the CID object runs (CidObj::mode).  0 and 1 are the single-detector modes passed to cid_create(); any value above 1 selects the automatic
 * mode 5, which runs both.  In the automatic mode the value moves to 2 when the FSK message is complete and has a good checksum, and to 3 when the DTMF
 * detector has found the start of a message (or failed).  The value 4 is not used.  The tests `> 1` stay literal in the code (they accept 2, 3 and 5).
 */
enum CidMode
{
  CID_MODE_FSK = 0,
  CID_MODE_DTMF = 1,
  CID_MODE_FSK_DONE = 2,     ///< the FSK receiver returned 3 (message complete, checksum good)
  CID_MODE_DTMF_MESSAGE = 3, ///< the DTMF detector has taken over a message (also reached when it failed)
  CID_MODE_AUTO = 5
};

/**
 * State of the byte framer of the FSK receiver (CidFsk::state), stepped once per demodulated bit: a seizure of 16 mark bits, then the wait for the first
 * start bit, then the data bytes (8 data bits, least significant first, a stop bit, the start bit of the next byte).
 */
enum CidFskState
{
  CIDFSK_SEIZURE = 0,    ///< counting mark bits up and space bits down; 16 in a row go to CIDFSK_FIRST_START
  CIDFSK_STOP = 1,       ///< a stop bit was expected; the next space is the start bit of the next byte
  CIDFSK_DATA = 2,       ///< data bits of the current byte
  CIDFSK_FIRST_START = 3 ///< waiting for the run of space bits that is the first start bit
};

/**
 * Phase of the DTMF detector (CidDtmf::phase), switched by dtmf_modem() on the energy found in the two half blocks of a frame by band_pass().
 */
enum CidDtmfPhase
{
  CIDDTMF_WAIT_GAP = 0, ///< a digit was stored: wait until the tone has gone (one of the half blocks is quiet)
  CIDDTMF_LISTEN = 1,   ///< look for a tone: both half blocks must carry energy
  CIDDTMF_DECODE = 2,   ///< a tone is present: DTMF_MTD_detect() picks the digit (the same digit must be seen twice)
  CIDDTMF_DONE = 3      ///< more than 2 digits and a pause of 300 ms: nothing more is decoded
};

/** State of the DTMF caller-ID detector (cid_detect.c, created by create_cid_dtmf()). */
typedef struct CidDtmf
{
  short state0; ///< cleared at reset, otherwise unused
  short pad002;
  int w004;               ///< cleared at reset, otherwise unused
  short s008;             ///< cleared at reset, otherwise unused
  short blk[300];         ///< the frame handed to DTMF_MTD_detect() when it is assembled from the stash and the new frame
  short stash[100];       ///< second half of the previous frame, kept for the next call (see halfFlag)
  unsigned short lastDig; ///< the digit code seen in the previous detection (USHRT_MAX = none)
  short cnt;              ///< how many consecutive detections of lastDig
  short nDig;             ///< digits stored in digits[]; -1 = waiting for the start tone 'D' in mode 1
  short phase;            ///< CidDtmfPhase
  short halfFlag;         ///< non-zero: the previous call left half a frame in stash[]
  short fails;            ///< consecutive failed detections
  unsigned short minEn;   ///< band_pass() energy floor (adapts)
  short* blkp;            ///< the frame being decoded: dtmf_modem()'s local buffer or blk[]; only meaningful during a dtmf_modem() call
  unsigned short fs;      ///< sampling rate (8000 or 9600)
  short mode;             ///< band_pass() sensitivity: 0, 1 unscaled, 2 .. 4 lower the threshold (7/8, 3/4, 1/2); mode 1 also needs the start tone 'D'
  char digits[20];        ///< the digits, ASCII '0' .. '9'; the 'C' that ends the number is stored as 0.  Only the first 16 are cleared at reset or returned
  short s354, s356;       ///< cleared at reset, otherwise unused
  short iirBand[2];       ///< band_pass() filter state (one biquad)
  short s35c[2];          ///< cleared at reset, otherwise unused
  short iirLow0[2];       ///< DTMF_MTD_detect(): state of the pre-filter of the second filter group (coefficient set 4)
  short iirLow1[2];       ///< DTMF_MTD_detect(): state of the pre-filter of the first filter group (coefficient set 5)
  short iirMtd[8][2];     ///< DTMF_MTD_detect(): states of the eight tone filters, the first four are the first group
  int samples;            ///< samples since the last detection
} CidDtmf;

/** State of the FSK caller-ID receiver (cid_modem.c and cid_detect.c, created by create_cid()). */
typedef struct CidFsk
{
  int agc;                   ///< gain of cid_modem() (0 after reset, then 1 or the adapted value)
  short s04;                 ///< cleared at reset, otherwise unused
  short dc;                  ///< DC estimate of cid_modem()
  short s08;                 ///< cleared at reset, otherwise unused
  short s0a;                 ///< unused
  FpmMrf mrf;                ///< polyphase filter that converts 8000 Hz input to 7200 Hz (9 / 10); mrf.buf == NULL means "not allocated yet"
  unsigned short cfgThr;     ///< framer threshold (default 2): the first start bit is cfgThr + 1 space bits; the first byte's bit index starts at cfgThr
  unsigned short fs;         ///< sampling rate (8000 or 9600)
  short toneInc;             ///< increment of the tone counter `tc` per detected seizure-tone block (create_cid(): 9)
  short lpfPos;              ///< write position in lpf[] (0 .. 16)
  short lpf[17];             ///< history of the demodulator's low-pass filter input (17 taps)
  short pos;                 ///< write position in hist[] (0 .. 4)
  short hist[5];             ///< the last five input samples
  unsigned short cur;        ///< the bit the demodulator currently believes in
  short nPos;                ///< samples above the threshold counted so far (the first 128 are accumulated in posAcc)
  short nNeg;                ///< negative samples counted for the threshold estimate (once `mean` is known)
  int thr;                   ///< adaptive slicer threshold
  int mean;                  ///< mean of the first 128 values above the threshold
  int negAcc;                ///< accumulator of the negative values
  int posAcc;                ///< accumulator of the first 128 positive values
  unsigned char pad_0074[4]; ///< cleared at reset, otherwise unused
  short run;                 ///< samples counted since the last bit (same level as `cur`)
  short runAlt;              ///< samples of the opposite level counted since the last bit
  short quiet;               ///< consecutive samples near zero
  short iirMtd1[2];          ///< CID_MTD_detect(): state of the first biquad
  short iirMtd2[2];          ///< CID_MTD_detect(): state of the second biquad
  short z86[4];              ///< cleared at reset, otherwise unused
  /* from here to the end the members form a byte image that cid_modem.c addresses as a whole (cid_msg_byte) */
  unsigned short tc;       ///< seizure-tone counter: raised by `toneInc` per block with the 1200 Hz tone, reset by a block without
  unsigned short out[36];  ///< bits written by CID_FSD_demodulate() (reset_cid() clears only out[0] and out[1])
  unsigned char msg[0x78]; ///< the received message: type, length, data ..., checksum
  short s150, s152;        ///< cleared at reset, otherwise unused
  short ones;              ///< up / down counter of mark bits in the seizure (16 = done)
  short state;             ///< CidFskState: the byte framer
  short acc;               ///< byte being assembled
  short bitIdx;            ///< bit index in the byte (or the run length of space bits in CIDFSK_FIRST_START)
  short count;             ///< bytes stored in msg[]
  short s15e;              ///< unused
} CidFsk;

/** The cid_* object: one or two detectors, a frame buffer and the result strings.  Created by cid_create(). */
typedef struct CidObj
{
  CidDtmf* dtmf;  ///< DTMF detector (NULL in FSK-only mode)
  CidFsk* fsk;    ///< FSK receiver (NULL in DTMF-only mode)
  char str[600];  ///< the NUL-separated result strings (cid_get_strings())
  int mode;       ///< CidMode
  int fmt;        ///< 2 = unformatted (hexadecimal) strings, else formatted ("DATE = ..")
  short buf[200]; ///< the frame being collected (160 or 192 samples)
  int fill;       ///< samples in buf
} CidObj;

/** The handle CID_create() returns: the host's modem object and the CID object. */
typedef struct CidHandle
{
  void* modem; ///< the host's `struct modem *`, passed back to modem_send_to_tty()
  CidObj* cid;
} CidHandle;

#ifdef __i386__
_Static_assert(sizeof(CidDtmf) == 0x38c, "CidDtmf size");
_Static_assert(offsetof(CidDtmf, blk) == 0x00a, "blk");
_Static_assert(offsetof(CidDtmf, stash) == 0x262, "stash");
_Static_assert(offsetof(CidDtmf, lastDig) == 0x32a, "lastDig");
_Static_assert(offsetof(CidDtmf, nDig) == 0x32e, "nDig");
_Static_assert(offsetof(CidDtmf, phase) == 0x330, "phase");
_Static_assert(offsetof(CidDtmf, minEn) == 0x336, "minEn");
_Static_assert(offsetof(CidDtmf, blkp) == 0x338, "blkp");
_Static_assert(offsetof(CidDtmf, fs) == 0x33c, "fs");
_Static_assert(offsetof(CidDtmf, mode) == 0x33e, "mode");
_Static_assert(offsetof(CidDtmf, digits) == 0x340, "digits");
_Static_assert(offsetof(CidDtmf, iirBand) == 0x358, "iirBand");
_Static_assert(offsetof(CidDtmf, iirLow0) == 0x360, "iirLow0");
_Static_assert(offsetof(CidDtmf, iirMtd) == 0x368, "iirMtd");
_Static_assert(offsetof(CidDtmf, samples) == 0x388, "samples");
_Static_assert(sizeof(CidFsk) == 0x160, "CidFsk size");
_Static_assert(offsetof(CidFsk, agc) == 0, "agc");
_Static_assert(offsetof(CidFsk, dc) == 6, "dc");
_Static_assert(offsetof(CidFsk, mrf) == 0xc && sizeof(FpmMrf) == 0x1c, "mrf");
_Static_assert(offsetof(CidFsk, mrf.buf) == 0x24, "mrf.buf");
_Static_assert(offsetof(CidFsk, cfgThr) == 0x28, "cfgThr");
_Static_assert(offsetof(CidFsk, toneInc) == 0x2c, "toneInc");
_Static_assert(offsetof(CidFsk, z86) == 0x86, "z86");
_Static_assert(offsetof(CidFsk, tc) == 0x8e, "tc");
_Static_assert(offsetof(CidFsk, out) == 0x90, "out");
_Static_assert(offsetof(CidFsk, msg) == 0xd8, "msg");
_Static_assert(offsetof(CidFsk, s150) == 0x150, "s150");
_Static_assert(offsetof(CidFsk, ones) == 0x154, "ones");
_Static_assert(offsetof(CidFsk, state) == 0x156, "state");
_Static_assert(offsetof(CidFsk, count) == 0x15c, "count");
_Static_assert(offsetof(CidFsk, s15e) == 0x15e, "s15e");
_Static_assert(sizeof(CidObj) == 0x3fc && offsetof(CidObj, str) == 8 && offsetof(CidObj, mode) == 0x260 && offsetof(CidObj, fmt) == 0x264, "CidObj");
_Static_assert(offsetof(CidObj, buf) == 0x268 && offsetof(CidObj, fill) == 0x3f8, "CidObj buf");
_Static_assert(sizeof(CidHandle) == 8 && offsetof(CidHandle, cid) == 4, "CidHandle");
_Static_assert(offsetof(CidFsk, fs) == 0x2a, "fs");
_Static_assert(offsetof(CidFsk, lpfPos) == 0x2e, "lpfPos");
_Static_assert(offsetof(CidFsk, lpf) == 0x30, "lpf");
_Static_assert(offsetof(CidFsk, pos) == 0x52, "pos");
_Static_assert(offsetof(CidFsk, hist) == 0x54, "hist");
_Static_assert(offsetof(CidFsk, cur) == 0x5e, "cur");
_Static_assert(offsetof(CidFsk, thr) == 0x64, "thr");
_Static_assert(offsetof(CidFsk, posAcc) == 0x70, "posAcc");
_Static_assert(offsetof(CidFsk, run) == 0x78, "run");
_Static_assert(offsetof(CidFsk, quiet) == 0x7c, "quiet");
_Static_assert(offsetof(CidFsk, iirMtd1) == 0x7e, "iirMtd1");
_Static_assert(offsetof(CidFsk, iirMtd2) == 0x82, "iirMtd2");
_Static_assert(offsetof(CidFsk, iirMtd2) + sizeof(((CidFsk*)0)->iirMtd2) == 0x86, "iirMtd2 end");
#endif

/* cid_detect.c: DTMF detector */
int band_pass(short* x, short n, CidDtmf* d);
int DTMF_MTD_detect(const short* x, short n, CidDtmf* d);
int dtmf_modem(const short* in, unsigned short n, CidDtmf* d);
void reset_dtmf(CidDtmf* d);
/* cid_detect.c: FSK detector */
unsigned short CID_FSD_demodulate(const short* in, unsigned short* out, short n, CidFsk* d);
int CID_MTD_detect(const short* x, short n, CidFsk* d);

/* cid_modem.c */
int cid_modem(const short* in, unsigned short n, CidFsk* f);
CidDtmf* create_cid_dtmf(CidDtmf* d);
CidFsk* create_cid(CidFsk* f);
void reset_cid(CidFsk* f);
void data_formatted_output(const CidFsk* f, char* out);
void data_unformatted_output(const CidFsk* f, char* out);
void data_raw(const unsigned char* msg, char* out);
CidObj* cid_create(CidObj* c, int fmt, int mode);
void cid_delete(CidObj* c);
int cid_progress(CidObj* c, const short* in, int unused, short* pn);
char* cid_get_strings(CidObj* c);
void cid_freq_sampl(CidObj* c, int fs);
void cid_reset(CidObj* c);
void cid_threshold(CidObj* c, int v);
void cid_value(CidObj* c, int v);
CidHandle* CID_create(void* modem, int fs, int fmt);
void CID_delete(CidHandle* h);
int CID_process(CidHandle* h, const short* in, int n);

#endif
