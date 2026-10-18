/**
 * @file v8.h
 * @brief Interface of the V.8 handshake engine (v8.c): the primitive toolkit, the message coder, the state machine and the engine object API.
 *
 * The engine object is the flat V8Obj of v8_obj.h; most functions here take it (or one of its embedded sub-objects) as the first argument.  The
 * object API a data-pump glue uses is V8Create() / V8Process() / V8Control() / V8UpdateModemParameters() / V8Delete(); everything else is the
 * machinery behind it, exported so that it can be tested piece by piece.  Each function is documented at its definition in v8.c.
 * The dp glue that adapts the object API to the modem is dp_v8.c.
 */
#ifndef DSPLIBS_PORT_V8_H
#define DSPLIBS_PORT_V8_H
#include "v8_obj.h"
#include "v8_status.h" /* enum V8Status: the values V8Process returns */
#include "v8_phase.h"  /* enums V8TxPhase / V8RxPhase / V8RxSub: the dispatch words of v8handshak */

/* --- small shared math / bit helpers --- */
int v8_absfn(short value);
int v8_mpyint(short factorA, short factorB);
unsigned char charFlip(unsigned char octet);
void v8_copycoeff(short* dst, const short* src, short count);

/* --- message encode / decode (v8_getbit works on a message slot; V8SetMessage / V8GetMessage on the object, which selects one) --- */
unsigned int v8_getbit(V8Msg* slot);
unsigned int V8SetMessage(V8Obj* v8, int msgType, const void* octets, int length);
unsigned int V8GetMessage(V8Obj* v8, void* octetsOut, int* lengthIO);

/* --- receive / transmit ring plumbing --- */
unsigned int v8_rxinit(V8Obj* v8);
unsigned int v8_txinit(V8Obj* v8);
unsigned int v8_rxreadqueue(V8Obj* v8);
unsigned int v8_txwritequeue(V8Obj* v8);

/* --- ANSam detection, gain stability check, AGC --- */
void checkSignalStability(V8Obj* v8);
unsigned int V8agc(V8Obj* v8);
void v8_detectorinit(V8Obj* v8, V8Det* det, const void* coefTable, short mode, short hitTarget, short hitHeadStart, short threshold, short hiThreshold);
unsigned int v8_tone_detect(V8Obj* v8, V8Det* det, short* samples);
void v8_dftupdate(V8Dft* dft, short binCount, short* samples, short sampleCount);
void v8_dftenergy(V8Dft* dft, short binCount, unsigned char shift);
void v8_phase_rev_init(void* pr);
void v8_phase_rev_detect(void* pr, const short* samples, short count);
void v8_ansaminit(V8Obj* v8);
void v8_ansamgenerate(V8Obj* v8, short* out);
void v8_TONEq_init(V8Obj* v8);
void v8_TONEq_generate(V8Obj* v8, short* out);
short v8_cosread(unsigned char index);

/* --- V.21 FSK modulator / demodulator --- */
void v8_V21_Init(V8Obj* v8, short txBand, short rxBand);
void V8_setFilters(V8Obj* v8, const void* space1, const void* space2, const void* mark1, const void* mark2);
void V8_V21_reset(V8Obj* v8);
void v8_fskmodulate(V8Obj* v8, short bit);
short v8_fsktxfilter(V8Obj* v8, short sample);
void v8_fskdemodulate(V8Obj* v8);

/* --- handshake set-up and message matching --- */
unsigned int v8handshakinit(V8Obj* v8);
void initTxSequence(V8Obj* v8);

/* rebuildJMSequence (answerer: builds the JM from the received CM) and evaluateRxJMSequence (caller: evaluates the received JM) */
void rebuildJMSequence(V8Obj* v8);
void evaluateRxJMSequence(V8Obj* v8);

/* one block of the handshake state machine: returns 0, 1 (timeout) or 2 (finished) */
unsigned int v8handshak(V8Obj* v8);

/*
 * --- the engine object API ---
 * V8Create's configuration record (V8Cfg): side (0 = caller, 1 = answerer), opMode, signal-detect timeout and message-detect timeout in seconds, an
 * unused word, and the pointer to the caller-owned capability block.
 */
void* V8Create(const V8Cfg* cfg);
void V8Delete(void* v8);
unsigned int V8Process(V8Obj* v8, const short* samplesIn, short* samplesOut, int sampleCount);
unsigned int V8Control(V8Obj* v8, int ctrlType);
unsigned int V8UpdateModemParameters(V8Obj* v8, unsigned char* caps);

#endif
