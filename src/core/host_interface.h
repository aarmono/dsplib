/**
 * @file host_interface.h
 * @brief The library's view of its host: data pump ids, status codes, the data pump registration records, the call-control and voice records, and
 *        the host parameter ids.
 *
 * The library is a set of data pumps that a host modem program drives.  This header holds the data-only half of that contract (the function half,
 * sysdep_* / modem_* / dsplibs_debug_*, is core/dsp_host.h):
 * - enum DP_ID and the DPSTAT_* codes: which data pump a session runs, and the status a pump reports back to the host;
 * - struct dp, struct dp_operations: the record a data pump registers with modem_dp_register() and the per-session object the host hands to it;
 * - struct dsp_info: the connection summary a pump writes for the host;
 * - FAX_CMD / FAX_STATUS_*, struct voice_info / VOICE_CMD / VOICE_STATUS_* / VOICE_STATE_*: the class 1 fax and voice command sets of the host;
 * - enum MODEM_PARAMETER_NAMES: the ids accepted by modem_get_param() / modem_set_param() (settings and a few object pointers of the host).
 *
 * The declarations mirror the slmodemd host interface (its modem_defs.h, modem_dp.h and modem_param.h): numeric values, member order and therefore
 * structure layout (i386 and x86-64) are part of the contract, because the host reads and writes the same objects.  Another host maps its own
 * definitions onto these (docs/host-interface.md).  tools/hostiface_check.sh compares this file with a copy of the slmodemd headers when one is
 * available; the library does not need them.
 *
 * Freestanding: no #include.  Usable from C and C++; the two dp records are C only (see below).  Guarded against repeated inclusion.
 */
#ifndef DSPLIBS_PORT_HOST_INTERFACE_H
#define DSPLIBS_PORT_HOST_INTERFACE_H

struct modem;

/**
 * Data pump identifiers.  A session is created for one of these; automatic mode (DP_AUTOMODE) and the V.8 / V.8bis negotiation select the pump that
 * actually runs.  The numbers are the host's wire values: they are stored in S-registers and compared across the host, so they are not an index.
 */
enum DP_ID
{
  DP_CALLPROG = 2,      ///< call progress (dialing and tone detection)
  DP_DUMMY = 3,         ///< placeholder pump with no signal processing
  DP_AUTOMODE = 4,      ///< automatic mode: the host picks the pump after the handshake
  DP_V8 = 8,            ///< V.8 handshake
  DP_V17 = 17,          ///< V.17 fax
  DP_V21 = 21,          ///< V.21
  DP_V22 = 22,          ///< V.22
  DP_V23 = 23,          ///< V.23
  DP_V22BIS = 122,      ///< V.22bis
  DP_V32 = 32,          ///< V.32
  DP_V32BIS = 132,      ///< V.32bis
  DP_V34 = 34,          ///< V.34
  DP_V34BIS = 134,      ///< V.34bis
  DP_B103 = 103,        ///< Bell 103
  DP_B212 = 212,        ///< Bell 212A
  DP_FAX = 100,         ///< class 1 fax (V.27ter / V.29 / V.17 and T.30 / HDLC)
  DP_K56 = 56,          ///< K56flex
  DP_K56_OR_V90 = 65,   ///< K56flex or V.90, decided during the handshake
  DP_V8BIS = 108,       ///< V.8bis
  DP_V90 = 90,          ///< V.90
  DP_V90_NO_V8BIS = 91, ///< V.90 without V.8bis
  DP_V92 = 92,          ///< V.92
  DP_TEST = 126,        ///< pump used by the host's test mode
  DP_SINUS = 127,       ///< single tone generator pump
  DP_MAX = 128,         ///< one past the largest id
  MAX_DP = 128          ///< same as DP_MAX
};

/** @name Data pump status codes (struct dp::status) @{ */
#define DPSTAT_OK 0         /**< running, nothing to report */
#define DPSTAT_CONNECT 1    /**< a connection is established */
#define DPSTAT_ERROR 4      /**< the session failed */
#define DPSTAT_NODIALTONE 6 /**< call progress: no dial tone */
#define DPSTAT_BUSY 7       /**< call progress: busy tone */
#define DPSTAT_NOANSWER 8   /**< call progress: the remote side did not answer */
#define DPSTAT_CHANGEDP 10  /**< the host is to continue with another pump (the requested id is in the host's DP_REQUESTED parameter) */
/** @} */

/**
 * Connection summary of a session: the data pumps write it, the host reads it (through MDMPRM_DSPINFO).
 * Layout: unsigned, long, unsigned, unsigned; `long` is 4 bytes on i386 and 8 on x86-64, so the offsets differ between the two.
 */
struct dsp_info
{
  unsigned connection_type; ///< connection type code
  long clock_deviation;     ///< measured clock deviation of the far end
  unsigned qc_lapm;         ///< 1 when the quick-connect data asks for LAPM
  unsigned qc_index;        ///< quick-connect table index
};

/** @name Class 1 fax @{ */
#define FAX_STATUS_OK 1        /**< command accepted / done */
#define FAX_STATUS_ERROR 2     /**< command failed */
#define FAX_STATUS_CONNECT 3   /**< carrier found, data follows */
#define FAX_STATUS_NOCARRIER 4 /**< carrier lost */

/** Class 1 fax commands, in the order of the host's table. */
enum FAX_CMD
{
  FAX_CMD_FTS, ///< AT+FTS: transmit silence
  FAX_CMD_FRS, ///< AT+FRS: receive silence
  FAX_CMD_FTM, ///< AT+FTM: transmit data
  FAX_CMD_FRM, ///< AT+FRM: receive data
  FAX_CMD_FTH, ///< AT+FTH: transmit HDLC
  FAX_CMD_FRH  ///< AT+FRH: receive HDLC
};
/** @} */

/** @name Voice @{ */

/** Commands of the voice engine, in the order of the host's table. */
enum VOICE_CMD
{
  VOICE_CMD_STATE_COMMAND, ///< go to command state
  VOICE_CMD_STATE_RX,      ///< start receiving (record)
  VOICE_CMD_STATE_TX,      ///< start transmitting (play)
  VOICE_CMD_STATE_DUPLEX,  ///< receive and transmit
  VOICE_CMD_STATE_SPEAKER, ///< route to the speaker
  VOICE_CMD_BEEP,          ///< play a beep
  VOICE_CMD_DTMF,          ///< play a DTMF symbol
  VOICE_CMD_ABORT          ///< abort the running operation
};

#define VOICE_STATUS_OK 1      /**< command accepted / done */
#define VOICE_STATUS_ERROR 2   /**< command failed */
#define VOICE_STATUS_CONNECT 3 /**< the voice path is up */

#define VOICE_STATE_COMMAND 0x00                             /**< idle, command state */
#define VOICE_STATE_RX 0x01                                  /**< receiving */
#define VOICE_STATE_TX 0x02                                  /**< transmitting */
#define VOICE_STATE_DUPLEX (VOICE_STATE_TX | VOICE_STATE_RX) /**< both */
#define VOICE_STATE_SPEAKER 0x08                             /**< speaker routed */

/** Voice settings, filled in by the host (read through MDMPRM_VOICEINFO); eleven consecutive `unsigned` members. */
struct voice_info
{
  unsigned comp_method;                ///< compression method of the sample stream
  unsigned sample_rate;                ///< sample rate of the stream, samples per second
  unsigned rx_gain;                    ///< receive gain
  unsigned tx_gain;                    ///< transmit gain
  unsigned dtmf_symbol;                ///< symbol of a VOICE_CMD_DTMF command
  unsigned tone1_freq;                 ///< first tone of a VOICE_CMD_BEEP / VOICE_CMD_DTMF command
  unsigned tone2_freq;                 ///< second tone
  unsigned tone_duration;              ///< tone duration
  unsigned inactivity_timer;           ///< inactivity timeout
  unsigned silence_detect_sensitivity; ///< silence detector sensitivity
  unsigned silence_detect_period;      ///< silence detector period
};
/** @} */

/**
 * Parameter ids of modem_get_param() / modem_set_param() / the host's settings.  The values are the positions in this list (MDMPRM_NONE is 0), so
 * the order is part of the contract.  The first group are host objects and plain settings (several return a pointer through the `long` result:
 * MDMPRM_DSPINFO, MDMPRM_DPRUNTIME, MDMPRM_VOICEINFO, MDMPRM_DP_ADDR); the Get* group are the call progress / dialer / voice-tone settings, read as
 * numbers.
 */
enum MODEM_PARAMETER_NAMES
{
  MDMPRM_NONE,                                      ///< no parameter
  MDMPRM_RX_RATE,                                   ///< receive rate
  MDMPRM_RATE = MDMPRM_RX_RATE,                     ///< same as MDMPRM_RX_RATE
  MDMPRM_TX_RATE,                                   ///< transmit rate
  MDMPRM_MIN_RATE,                                  ///< lowest rate to negotiate
  MDMPRM_MAX_RATE,                                  ///< highest rate to negotiate
  MDMPRM_IODELAY,                                   ///< delay of the host's sample path
  MDMPRM_CODECTYPE,                                 ///< codec type code
  MDMPRM_DIALSTR,                                   ///< the dial string
  MDMPRM_AUTOMODE,                                  ///< automatic mode setting
  MDMPRM_DP_REQUESTED,                              ///< data pump requested (set with DPSTAT_CHANGEDP)
  MDMPRM_DPRUNTIME,                                 ///< pointer to the shared data pump parameter block
  MDMPRM_DSPINFO,                                   ///< pointer to the host's struct dsp_info
  MDMPRM_VOICEINFO,                                 ///< pointer to the host's struct voice_info
  MDMPRM_UPDATE_DELAY,                              ///< sample path delay adjustment (write)
  MDMPRM_DP_ADDR,                                   ///< pointer to the running struct dp
  MDMPRM_HOOK_ON,                                   ///< hook state
  MDMPRM_PULSE_DIAL,                                ///< pulse dialing selected
  GetPulseDialMakeTime,                             ///< pulse dialing: make time
  GetPulseDialBreakTime,                            ///< pulse dialing: break time
  GetPulseDialDigitPattern,                         ///< pulse dialing: pulse pattern of a digit
  GetDTMFHighToneLevel,                             ///< DTMF: level of the high tone
  GetDTMFDialSpeed,                                 ///< DTMF: dial speed
  GetMinBusyCadenceOnTime,                          ///< busy tone: shortest on time
  GetMaxBusyCadenceOnTime,                          ///< busy tone: longest on time
  GetBusyDetectionCyclesNumber,                     ///< busy tone: cycles needed for detection
  GetMinBusyCadenceOffTime,                         ///< busy tone: shortest off time
  GetMaxBusyCadenceOffTime,                         ///< busy tone: longest off time
  GetCallingToneFlag,                               ///< send a calling tone
  GetHookFlashTime,                                 ///< hook flash duration
  GetBlindDialPause,                                ///< pause before blind dialing
  GetNoAnswerTimeOut,                               ///< no answer timeout
  GetDialPauseTime,                                 ///< duration of the dial string pause character
  GetTransmitLevel,                                 ///< transmit level
  GetDialModifierValidation,                        ///< validate the dial string modifiers
  GetDialToneValidationTime,                        ///< time a dial tone must persist
  GetBusyToneDiffTime,                              ///< busy tone: tolerance between on and off times
  GetDTMFHighAndLowToneLevelDifference,             ///< DTMF: level difference between the high and the low tone
  GetPulseDialingFlag,                              ///< pulse dialing enabled
  GetDialToneCallProgressFilterIndex,               ///< dial tone: call progress filter table index
  GetDialToneDetectionThreshold,                    ///< dial tone: detection threshold
  GetABCDDialingPermittedFlag,                      ///< DTMF digits A to D permitted
  GetComaPauseDurationLimit,                        ///< longest pause the comma may stand for
  GetPulseAndToneDialInSameDialStringPermittedFlag, ///< pulse and tone digits may be mixed in one dial string
  GetBusyToneCallProgressFilterIndex,               ///< busy tone: call progress filter table index
  GetPulseBetweenDigitsInterval,                    ///< pulse dialing: interval between digits
  GetDialToneWaitTime,                              ///< time to wait for a dial tone
  GetMinRingbackCadenceOnTime,                      ///< ringback: shortest on time
  GetMaxRingbackCadenceOnTime,                      ///< ringback: longest on time
  GetRingbackDetectionCyclesNumber,                 ///< ringback: cycles needed for detection
  GetMinRingbackCadenceOffTime,                     ///< ringback: shortest off time
  GetMaxRingbackCadenceOffTime,                     ///< ringback: longest off time
  GetRingbackToneCallProgressFilterIndex,           ///< ringback: call progress filter table index
  GetMinCongestionCadenceOnTime,                    ///< congestion tone: shortest on time
  GetMaxCongestionCadenceOnTime,                    ///< congestion tone: longest on time
  GetCongestionDetectionCyclesNumber,               ///< congestion tone: cycles needed for detection
  GetMinCongestionCadenceOffTime,                   ///< congestion tone: shortest off time
  GetMaxCongestionCadenceOffTime,                   ///< congestion tone: longest off time
  GetCongestionToneCallProgressFilterIndex,         ///< congestion tone: call progress filter table index
  GetCallProgressSamplesBufferLength,               ///< length of the call progress sample buffer
  MustNoiseFilterBeApplied,                         ///< apply the noise filter to the call progress input
  GetAdditAttenToBeepgenVoice,                      ///< additional attenuation of the voice beep generator
  GetDialToneFilterSubindex,                        ///< dial tone: filter sub-table index
  GetBusyToneLooseDetectionEnabled,                 ///< busy tone: relaxed detection
  MDMPRM_LAST                                       ///< one past the last id
};

#ifndef __cplusplus

/**
 * Operations record of a data pump, registered once per pump id with modem_dp_register().  The host calls the members; `use_count` is the host's
 * reference count of the registration.
 */
struct dp_operations
{
  const char* name; ///< pump name (diagnostics)
  int use_count;    ///< registrations / sessions currently using the record (maintained by the host)
  /** create a session: caller is non-zero for the calling side, srate the sample rate, max_frag the largest block process() is given; NULL on failure */
  struct dp* (*create)(struct modem*, enum DP_ID id, int caller, int srate, int max_frag, struct dp_operations*);
  int (*delete)(struct dp*);                                ///< destroy a session
  int (*process)(struct dp*, void* in, void* out, int cnt); ///< run `cnt` samples from `in` to `out`; returns a DPSTAT_* code or 0
  int (*hangup)(struct dp*);                                ///< end the session gracefully
};

/** Per-session object of a running data pump; the host owns the record, the pump owns what `dp_data` points to. */
struct dp
{
  enum DP_ID id;            ///< pump id
  struct modem* modem;      ///< host modem object
  unsigned status;          ///< last DPSTAT_* code
  struct dp_operations* op; ///< operations record the session was created from
  void* dp_data;            ///< the pump's own state
};

/** Register / unregister a data pump (host functions). */
extern int modem_dp_register(enum DP_ID id, struct dp_operations* op);
extern void modem_dp_deregister(enum DP_ID id, struct dp_operations* op);

#endif /* !__cplusplus */

#endif /* DSPLIBS_PORT_HOST_INTERFACE_H */
