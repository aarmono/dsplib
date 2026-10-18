/**
 * @file fax_t30.h
 * @brief The T.30 / HDLC fax class 1 object (FaxT30) and its state functions: transmit side (fax_t30_tx.c), receive side (fax_t30_rx.c), tables
 *   (fax_t30_tables.c).
 *
 * FaxT30 is the class 1 object that fax_class1_create() allocates (see fax_class1.c).  The class 1 layer (fax_class1_progress()) calls one state function per
 * block of samples through the table fax_class1_state_functions[state]; the states implement the T.30 phases of a fax call:
 * - transmit: silence before the preamble, preamble flags, HDLC frame transmission (V.21), scrambled ones / data / nulls on the V.27ter / V.29 / V.17 data
 *   modem;
 * - receive: HDLC reception (V.21) with command-mode buffering of the received frames (the "superframe"), data modem carrier search and data reception;
 * - shared: idle, silence, answer tone, calling tone.
 * The class 1 module also provides the DTE-side converters (_handle_data_input(), _handle_hdlc_input(): DLE un-stuffing into 16-bit words) and the
 * output converters (_handle_data_output(), cTOOLS_handle_hdlc_output(): words to DLE-stuffed bytes for the DTE).
 *
 * Layout: the object is an ordinary C struct with real pointers. In 32-bit builds every member sits at the offset the `_Static_assert`s pin (the object is
 * 0x12f4 bytes there); in 64-bit builds the members after a pointer move, so always use the member names.
 *
 * Work buffer: `buf` is the first member, so the object itself IS the work buffer ("obj + 0"): FIFO_write() / FIFO_read(), the FAXVMI_process() buffer
 * argument and the output of _handle_data_input() / _handle_hdlc_input() are all `o->buf`. A state function writes at most frameWords (<= 36) data words or
 * one HDLC frame ([len][bytes] words, 0x200 words maximum) into it. buf[0x800 .. 0x8ff] holds the superframe of the receive side.
 *
 * Channels: hdlc_tx is the V.21 HDLC transmit channel (FAXVMI mode 2), hdlc_rx the V.21 HDLC receive channel, data_vmi the data modem channel (V.27ter / V.29
 * / V.17 transmit or receive: _init_transmitter() creates the transmit one, _init_receiver() the receive one). hdlc_tx_cfg / hdlc_rx_cfg are the 0x18-byte
 * FaxVmiCfg blocks fax_class1_create() builds for the HDLC channels; data_cfg is the 0x18-byte block the init_vmi_data_*_modem function fills (the VMI view
 * FaxVmiM = the config FaxVmiCfg of the data channel; its `u10.cfg` is the modem's config block); both are freed by _delete_data_tx_modem() /
 * _delete_data_rx_modem().
 *
 * State: `state` indexes class1_state_functions (0 _t30_silence_before_tx_state, 1 _t30_preabmle_state, 2 _send_hdlc_buffer_state, 3
 * _send_hdlc_between_buffer_state,
 * 4 _hdlc_receive_look_carrier_state, 5 _hdlc_receive_state, 6 _hdlc_receive_between_buffers_state, 7 _hdlc_emulate_receive_state, 8 _idle_state,
 * 9 _tx_scrambled_ones_state, 10 _tx_data_state, 11 _tx_nulls_state, 12 _rx_look_carrier_state, 13 _rx_data_state, 14 _answer_tone_state, 15
 * _send_silence_state,
 * 16 _recieve_silence_state, 17 cHDLCtx_off, 18 _tx_silence_before_scrm_ones); state_prev is the previous one.  `tmr` is the state timer (every state
 * function counts it, the init functions clear it); `status` is the result code fax_class1_progress() returns (a FaxClass1Status value) when the command
 * ends; `sec` / `ms` are the clock the debug messages print ("%2d.%02d[sec]"). The transmit FIFO `fifo` (0x800 words) feeds the data states: frameWords are
 * the words per 20 ms frame (rate / 400), enableDelay / txEnable / txConnected / fifoReady the scrambled-ones / data hand-over flags, silenceTime the silence
 * time, dataRate the data rate. alignDone / alignHist / alignShift / alignMask are the bit-alignment state of _handle_data_output() (reset by
 * cTOOLS_handle_data_output_reset()).
 *
 * Members documented "medium" are named from the behaviour and the debug strings ("ENABLE_TRANSMIT") only. w1230 / w1234 are written by _init_transmitter()
 * per rate and read by nothing; w12ac is never accessed.
 */
#ifndef DSPLIBS_PORT_FAX_T30_H
#define DSPLIBS_PORT_FAX_T30_H
#include <stddef.h>
#include <stdint.h>

#include "fax_common.h"
#include "fax_vmi.h"
#include "fpm/fpm.h"

/**
 * The modem selection of the class 1 commands (FaxT30.modem_sel, the parameter of +FTM / +FRM and of _init_transmitter() / _init_receiver()).
 * The values are the T.31 / T.32 rate numbers in hex (24 = 0x18 V.27ter 2400, 48 = 0x30, 72 = 0x48 V.29 7200, 73 = 0x49 V.17 7200 long train,
 * 74 = 0x4a V.17 7200 short train, 96 = 0x60, 97 / 98, 121 / 122, 145 / 146); the debug line "Short train in V.17 mode is selected" is printed for exactly
 * the short-train values 0x4a / 0x62 / 0x7a / 0x92.
 */
enum FaxModemSel
{
  FAXMOD_V27_2400 = 0x18,
  FAXMOD_V27_4800 = 0x30,
  FAXMOD_V29_7200 = 0x48,
  FAXMOD_V17_7200 = 0x49,
  FAXMOD_V17_7200_SHORT = 0x4a,
  FAXMOD_V29_9600 = 0x60,
  FAXMOD_V17_9600 = 0x61,
  FAXMOD_V17_9600_SHORT = 0x62,
  FAXMOD_V17_12000 = 0x79,
  FAXMOD_V17_12000_SHORT = 0x7a,
  FAXMOD_V17_14400 = 0x91,
  FAXMOD_V17_14400_SHORT = 0x92
};

/** FaxT30.modemClass: the data modem family of a modem selection; the index of fax_init_vmi_data_tx_modem[] / fax_init_vmi_data_rx_modem[]. */
enum FaxModemClass
{
  FAXMC_V27 = 0,
  FAXMC_V29 = 1,
  FAXMC_V17 = 2
};

/** FaxT30.ans_org (set by fax_class1_create()) and FaxClass1Cfg.mode: the side of the call. */
enum FaxClass1Role
{
  FAXC1_ROLE_ORIGINATE = 1,
  FAXC1_ROLE_ANSWER = 2
};

/** FaxT30.chan: which data modem the object holds (fax_class1_command() / _delete pick the delete function by it). */
enum FaxClass1Chan
{
  FAXC1_CHAN_RX = 1,
  FAXC1_CHAN_TX = 2
};

/**
 * FaxT30.state: the index of the state function in fax_class1_state_functions.  The names follow the table fax_class1_state_names (printed in
 * "FAX_CLASS1_STATE: %s ===> %s"): FAXC1_ST_ plus the table string; the functions are listed in the file comment.
 */
enum FaxClass1State
{
  FAXC1_ST_T30_SILENCE_BEFORE_PREAMBLE_STATE = 0,
  FAXC1_ST_T30_PREAMBLE_STATE = 1,
  FAXC1_ST_SEND_HDLC_BUFFER_STATE = 2,
  FAXC1_ST_SEND_HDLC_BETWEEN_BUFFER_STATE = 3,
  FAXC1_ST_HDLC_RECEIVE_LOOK_CARRIER_STATE = 4,
  FAXC1_ST_HDLC_RECEIVE_STATE = 5,
  FAXC1_ST_HDLC_RECEIVE_BETWEEN_BUFFERS_STATE = 6,
  FAXC1_ST_HDLC_EMULATE_RECEIVE_STATE = 7,
  FAXC1_ST_IDLE_STATE = 8,
  FAXC1_ST_TX_SCRAMBLED_ONES_STATE = 9,
  FAXC1_ST_TX_DATA_STATE = 10,
  FAXC1_ST_TX_NULLS_STATE = 11,
  FAXC1_ST_RX_LOOK_CARRIER = 12,
  FAXC1_ST_RX_DATA_STATE = 13,
  FAXC1_ST_ANSWER_TONE_STATE = 14,
  FAXC1_ST_SEND_SILENCE_STATE = 15,
  FAXC1_ST_RECIEVE_SILENCE_STATE = 16,
  FAXC1_ST_CHDLCTX_OFF_STATE = 17,
  FAXC1_ST_TX_SILENCE_BEFORE_SCRM_ONES = 18,
  FAXC1_ST_MAX_STATES = 19
};

/** The status codes fax_class1_progress() returns (FaxT30.status, rx_event); the names follow the table fax_class1_status_names. */
enum FaxClass1Status
{
  FAX_CLASS1_NO_MESSAGE = 0,
  FAX_CLASS1_OK = 1,
  FAX_CLASS1_ERROR = 2,
  FAX_CLASS1_OK_NO_CARRIER = 3,
  FAX_CLASS1_ERROR_NO_CARRIER = 4,
  FAX_CLASS1_ERROR_ON_HOOK = 5,
  FAX_CLASS1_CONNECT = 6,
  FAX_CLASS1_NO_CARRIER = 7,
  FAX_CLASS1_NO_CARRIER_NO_MESSAGE = 8,
  FAX_CLASS1_OTHER_CARRIER = 9,
  FAX_CLASS1_ACCEPT_RATE = 10
};

/** The class 1 fax object (see the file comment for the roles of the groups of members). */
typedef struct FaxT30
{
  unsigned short buf[0x900]; ///< work buffer (the object itself); buf[0x800 .. 0x8ff] is the superframe of the receive side
  FaxVmi* hdlc_tx;           ///< V.21 HDLC transmit channel
  FaxVmi* hdlc_rx;           ///< V.21 HDLC receive channel
  FaxVmi* data_vmi;          ///< data modem channel (transmit or receive)
  FaxVmiCfg* hdlc_tx_cfg;    ///< config block of hdlc_tx
  FaxVmiCfg* hdlc_rx_cfg;    ///< config block of hdlc_rx
  FaxVmiM* data_cfg;         ///< config block of data_vmi
  int ans_org;               ///< 1 = created as the originating side (calling tone), 2 = answering side (answer tone)
  int state;                 ///< current state (FaxClass1State)
  int state_prev;            ///< previous state
  int lastFrame;             ///< set when the frame just collected was the last one (HDLC control byte buf[2] bit 4, the T.30 final-frame bit); the
                             ///<        transmit side then goes idle after sending it
  int tmr;                   ///< state timer (read as unsigned by most states)
  int status;                ///< FaxClass1Status: the result code fax_class1_progress() returns (cleared per call; the states set it)
  int w1230;                 ///< written by _init_transmitter(): 0x30 / 0x18 / 0xc / 6; read by nothing
  int w1234;                 ///< written by _init_transmitter(): 0x8000 / 0x75a2 / 0x4000; read by nothing
  int lastByte;              ///< last byte of the DTE input block (_handle_data_input()); _tx_data_state() on a queue under-run: nonzero (DLE ETX seen)
                             ///<        stops the transmission, zero continues with nulls
  int rx_event;              ///< receive result code the AT layer reports: 1 frame OK, 2 frame with errors, 7 carrier lost (data / HDLC receive states)
  int rx_kind;               ///< 2 = a receive result is pending (set with rx_event), 0 cleared by _cHDLCrx_init_from_idle()
  int chan;                  ///< the data channel kind (FaxClass1Chan): 1 = a data receiver exists, 2 = a data transmitter
  int modemClass;            ///< data modem class (FaxModemClass): index into fax_init_vmi_data_*_modem (the debug line prints it as MODEM_IDX)
  int dlePending;            ///< the previous input byte was DLE (_handle_data_input() / _handle_hdlc_input())
  int hdlcWr;                ///< HDLC input write index into buf (_handle_hdlc_input*(); 1 after _handle_hdlc_input_open())
  int modem_sel;             ///< the modem selection of the class 1 command (FaxModemSel; 3 after fax_class1_create())
  FpmTone* tone;             ///< tone generator (answer tone, calling tone)
  int cng_tone;              ///< _hdlc_receive_look_carrier_state(): 1 = the calling tone is on (0.5 s), 0 = off (3 s)
  int cng_ctr;               ///< samples of the current calling tone phase
  int cng_on;                ///< nonzero: the receiver sends the calling tone while it looks for the carrier
  int ans_len;               ///< answer tone length (frames of 20 ms; 150 or the configured ms / 20)
  int dataRate;              ///< data rate (bit/s)
  int enableDelay;           ///< scrambled-ones state: calls left (2 after the connect) before txEnable is set ("ENABLE_TRANSMIT"; medium)
  int sec;                   ///< clock printed by the debug messages: seconds
  int ms;                    ///< clock printed by the debug messages: hundredths
  int rx_act;                ///< count of non-zero input samples among the first n / 8 (fax_class1_progress(), debug dump of the input; 0 = not watching)
  int frameDone;             ///< _t30_preabmle_state(): result of the last _handle_hdlc_input() (nonzero: a frame is complete)
  int frameCount;            ///< _t30_preabmle_state(): frames collected ("send %d buffers")
  FaxFifo* fifo;             ///< transmit FIFO of the data states
  int txEnable;              ///< scrambled-ones state: the transmission may switch to data (set by the enableDelay countdown, "ENABLE_TRANSMIT"; medium)
  int frameWords;            ///< data words per 20 ms frame (dataRate / 400)
  int txConnected;           ///< set once the transmitter reported the connect ("Tx connect", status bit 0x100)
  int fifoReady;             ///< the FIFO holds at least one frame (count >= frameWords)
  int alignDone;             ///< _handle_data_output(): the byte alignment of the stream has been found
  unsigned int alignHist;    ///< _handle_data_output(): bit history of the previous bytes (all ones after the reset)
  int alignShift;            ///< _handle_data_output(): right shift of the extraction window
  unsigned int alignMask;    ///< _handle_data_output(): mask of the extraction window
  int w12ac;                 ///< never accessed
  int dataEnd;               ///< the DTE sent DLE ETX (end of the data stream): _handle_data_input() then accepts nothing; cleared by _tx_scrambled_ones_init()
  int s7;                    ///< S7 (seconds to wait for a carrier)
  int silenceTime;           ///< silence time before the transmission (ms, set by the TM command); the receive silence state reuses it as its counter (medium)
  int silence_rms;           ///< the low 16 bits: FPM_rms() of the last block of _recieve_silence_state() (the high half is kept)
  int agc_rcp;               ///< the V.21 receiver's AGC reciprocal gain after the last good frame (given to the data receiver's AGC by _init_receiver())
  int agc_shift;             ///< the AGC shift that goes with agc_rcp
  int emu_delay;             ///< _hdlc_emulate_receive_state(): calls before the next frame is handed out
  int emu_idx;               ///< index of the next frame of the superframe
  int sf_fill;               ///< words used in the superframe buffer (buf[0x800 .. 0x8ff]: [len][bytes] records)
  int cd_flag;               ///< value of the carrier detector flag of the data receiver (V.27ter hdx.s14, V.29 hdx.s1c, V.17 hdx.s20)
  int gain_att;              ///< the gain attenuation (dB) _hdlc_receive_look_carrier_state() asks for
  const void* hpCoef;        ///< the high-pass filter coefficient table (fax_hp_coeff)
  unsigned char hpState[16]; ///< the high-pass filter state (cleared by fax_class1_create())
  int hp_on;                 ///< nonzero: fax_class1_progress() high-pass filters the input (FPM_iir_filt_II() with hpCoef / hpState)
} FaxT30;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(FaxT30) == 0x12f4 && offsetof(FaxT30, hdlc_tx) == 0x1200 && offsetof(FaxT30, hdlc_rx) == 0x1204 && offsetof(FaxT30, data_vmi) == 0x1208 &&
                   offsetof(FaxT30, hdlc_tx_cfg) == 0x120c && offsetof(FaxT30, hdlc_rx_cfg) == 0x1210 && offsetof(FaxT30, data_cfg) == 0x1214 &&
                   offsetof(FaxT30, state) == 0x121c && offsetof(FaxT30, lastFrame) == 0x1224 && offsetof(FaxT30, tmr) == 0x1228 &&
                   offsetof(FaxT30, status) == 0x122c && offsetof(FaxT30, w1230) == 0x1230 && offsetof(FaxT30, lastByte) == 0x1238 &&
                   offsetof(FaxT30, modemClass) == 0x1248 && offsetof(FaxT30, hdlcWr) == 0x1250 && offsetof(FaxT30, tone) == 0x1258 &&
                   offsetof(FaxT30, ans_len) == 0x1268 && offsetof(FaxT30, dataRate) == 0x126c && offsetof(FaxT30, sec) == 0x1274 &&
                   offsetof(FaxT30, ms) == 0x1278 && offsetof(FaxT30, frameDone) == 0x1280 && offsetof(FaxT30, fifo) == 0x1288 &&
                   offsetof(FaxT30, frameWords) == 0x1290 && offsetof(FaxT30, fifoReady) == 0x1298 && offsetof(FaxT30, alignDone) == 0x129c &&
                   offsetof(FaxT30, alignMask) == 0x12a8 && offsetof(FaxT30, dataEnd) == 0x12b0 && offsetof(FaxT30, silenceTime) == 0x12b8 &&
                   offsetof(FaxT30, cd_flag) == 0x12d4 && offsetof(FaxT30, hpCoef) == 0x12dc && offsetof(FaxT30, hpState) == 0x12e0 &&
                   offsetof(FaxT30, hp_on) == 0x12f0,
               "FaxT30 32-bit layout");
/* members used by the class 1 layer (fax_class1.c) */
_Static_assert(offsetof(FaxT30, ans_org) == 0x1218 && offsetof(FaxT30, chan) == 0x1244 && offsetof(FaxT30, modem_sel) == 0x1254 &&
                   offsetof(FaxT30, ans_len) == 0x1268 && offsetof(FaxT30, rx_act) == 0x127c && offsetof(FaxT30, hp_on) == 0x12f0,
               "FaxT30 class 1 layer members");
/* members used by the HDLC / data receive states (fax_t30_rx.c) */
_Static_assert(offsetof(FaxT30, rx_event) == 0x123c && offsetof(FaxT30, rx_kind) == 0x1240 && offsetof(FaxT30, cng_tone) == 0x125c &&
                   offsetof(FaxT30, cng_ctr) == 0x1260 && offsetof(FaxT30, cng_on) == 0x1264 && offsetof(FaxT30, s7) == 0x12b4 &&
                   offsetof(FaxT30, silence_rms) == 0x12bc && offsetof(FaxT30, agc_rcp) == 0x12c0 && offsetof(FaxT30, agc_shift) == 0x12c4 &&
                   offsetof(FaxT30, emu_delay) == 0x12c8 && offsetof(FaxT30, emu_idx) == 0x12cc && offsetof(FaxT30, sf_fill) == 0x12d0 &&
                   offsetof(FaxT30, cd_flag) == 0x12d4 && offsetof(FaxT30, gain_att) == 0x12d8,
               "FaxT30 receive state members");
#endif

/**
 * Signature of every state function.  fax_class1_progress() calls the current state through fax_class1_state_functions[state] with its own arguments:
 * - o        the object
 * - rx       the received samples (n_in = 160 of them; cHDLCtx_off() reads them through the receive channel)
 * - tx       the buffer the transmit samples are written to
 * - tty_out  the fax object's buffer for data bytes TO the DTE (the receive states fill it)
 * - tty_in   the bytes the DTE sent
 * - n_in     the number of received samples (160)
 * - n_out    in: the number of samples wanted (0xa0); out: the number written
 * - p7       the number of bytes written to tty_out (cleared by fax_class1_progress() before the call; the receive states write it)
 * - p8       in: the number of tty_in bytes available; out (transmit states): free room of the FIFO - 1 / 0x200 / 0
 * The transmit states hand the DTE bytes to _handle_data_input() / _handle_hdlc_input(), which write words into o->buf and update *p8.
 * @return 0 (every state function returns 0; the result is reported through o->status).
 */
typedef int (*FaxT30StateFn)(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);

/* State functions of the transmit side (fax_t30_tx.c); the signature is FaxT30StateFn. */
int _t30_silence_before_tx_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
int _t30_preabmle_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
int _send_hdlc_buffer_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
int _send_hdlc_between_buffer_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7,
                                    int* p8);
int _tx_scrambled_ones_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
int _tx_data_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
int _tx_nulls_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
int cHDLCtx_off(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
int _tx_silence_before_scrm_ones(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
/* Shared states (fax_t30_tx.c) under library-private names: idle (_idle_state), send silence (_send_silence_state), answer tone (_answer_tone_state). */
int fax_idle_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
int fax_send_silence_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
int fax_answer_tone_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);

/* The state initialisers: (o [, argument]). */
int _idle_state_init(FaxT30* o);
int _init_tx_nulls_state(FaxT30* o);               ///< no caller in this library
int _send_silence_state_init(FaxT30* o, int n);    ///< no caller in this library
int _tx_scrambled_ones_init(FaxT30* o, int modem); ///< calls _init_transmitter(); (re)creates the FIFO and clears the hand-over flags
int cHDLCtx_preamble_state_init(FaxT30* o);
int _send_hdlc_between_buffer_state_init(FaxT30* o); ///< no caller in this library
int cHDLCtx_off_init(FaxT30* o);                     ///< no caller in this library
/**
 * @brief Create the data modem transmit channel and select the first transmit state.
 * @param o     the object
 * @param modem a FaxModemSel value; any other value leaves the rate and the modem class at 0 (V.27ter, rate 0)
 */
void _init_transmitter(FaxT30* o, int modem);
void _delete_data_tx_modem(FaxT30* o); ///< needs o->data_cfg (NULL is dereferenced)
void _put_silence(short* buf, int n);
void cTOOLS_handle_data_output_reset(FaxT30* o);
int _handle_data_output(FaxT30* o, const unsigned short* in, unsigned char* out, int n, int flag);
int cTOOLS_handle_hdlc_output(FaxT30* o, const unsigned short* in, unsigned char* out, int n, int flag);

/** One entry of the T.30 frame name table. */
typedef struct FaxT30FrameName
{
  unsigned int id;
  const char* name;
} FaxT30FrameName;
extern const FaxT30FrameName fax_t30_frame_names[36]; ///< frame names searched by GetT30FrameNameByID()
extern const unsigned char aReversedCharsArray[256];  ///< bit-reversal table
unsigned int GetT30FrameIDFromBuffer(unsigned char b0, unsigned char b1, unsigned char b2);
const char* GetT30FrameNameByID(unsigned int id);

/** Counts the calls of _tx_scrambled_ones_state() / _tx_data_state() since the last _tx_scrambled_ones_init(). */
extern int fax_cDATAtx_counter;

/* State functions of the receive side (fax_t30_rx.c) and the input side that converts the DTE bytes into words; the signature is FaxT30StateFn. */
int _hdlc_receive_look_carrier_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7,
                                     int* p8);
int _hdlc_receive_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
int _hdlc_receive_between_buffers_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7,
                                        int* p8);
int _hdlc_emulate_receive_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
int _rx_look_carrier_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
int _rx_data_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
/** The receive silence state (state 16, _recieve_silence_state) under its library-private name. */
int fax_recieve_silence_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8);
int _recieve_silence_state_init(FaxT30* o, int n); ///< no caller in this library: state 16, timer n / 2 (at least 1), silenceTime = 0, silence_rms low half 0
int _hdlc_receive_state_init(FaxT30* o);
int _rx_look_carrier_init(FaxT30* o, int modem); ///< _init_receiver() + cTOOLS_handle_data_output_reset(), timer 0
/**
 * @brief Create or restart the data modem receive channel and select the carrier-search state.
 * @param o     the object
 * @param modem a FaxModemSel value (0x18 / 0x30 V.27ter, 0x48 / 0x60 V.29, 0x49 .. 0x92 V.17); any other value leaves the class and the rate at 0
 */
void _init_receiver(FaxT30* o, int modem);
void _cHDLCrx_init_from_idle(FaxT30* o, int mode); ///< mode 3: select state 4 (_hdlc_receive_look_carrier_state)
void _delete_data_rx_modem(FaxT30* o);             ///< needs o->data_cfg and o->data_vmi (NULL is dereferenced)
extern const short fax_gain_threshold_table[4];    ///< carrier gain thresholds of the HDLC receive
extern int fax_handle_data_input_count;            ///< bytes seen by _handle_data_input() (written, never read)
int _handle_hdlc_input_close(FaxT30* o);           ///< no caller in this library: buf[0] = hdlcWr - 1, lastFrame = 1 when buf[2] bit 4
int _handle_data_input(FaxT30* o, const unsigned char* in, unsigned short* out,
                       int* n); ///< DLE (0x10) un-stuffing; ETX (0x03) sets dataEnd; writes words, *n = count
int _handle_hdlc_input(FaxT30* o, const unsigned char* in, unsigned short* out, int* n); ///< returns 1 when the frame is complete (DLE ETX), else 0 and *n = 0
int _handle_hdlc_input_open(FaxT30* o);                                                  ///< o->hdlcWr = 1; returns 0
#endif
