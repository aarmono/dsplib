/**
 * @file pcm_modem.cpp
 * @brief V90Modem and V92Modem: the owners of one side of the PCM (V.90 / V.92) modem.
 *
 * Each object builds, on the heap, the parameter block (V90Parameters / V92Parameters), the phase-2 info, the Jd objects (V.90) or the CP / mapping
 * parameters (V.92), and the active side: the modulator (V.90 side 0 "Digital", V.92 side 1 "Analog") or the demodulator (V.90 side 1 "Analog").  They
 * forward setSessionFlag() / reset() / progress() to the active side and print the version banner (printTitle()).  VPcmFloModem embeds one V90Modem and
 * one V92Modem (layout: pcm_modem.h, asserts in pcm_modem_layout.inc).
 * - V90Modem: constructor, destructor, printTitle(), reset(qc), setSessionFlag(), progress().
 * - V92Modem: constructor, destructor, printTitle(), reset(), progress().  Side 0 does nothing; the V.92 ComputationalMode argument is ignored.
 * Every callee is another file's function (the parameter, phase-2, MP / CP, DIL, Jd, modulator and demodulator classes) or edprintf() /
 * dsplibs_debug_printf().
 *
 * Construction and destruction order:
 * - The V90Modem constructor runs the V90MP and V90CP member constructors (in declaration order) before anything else; the destructor runs them last
 *   (implicit member destructors in reverse order).
 * - The heap objects are sysdep_malloc()ed and constructed in place; every pointer member is stored after its constructor returned (V90: params_, info_,
 *   jd_, jd92_, mod_ / dem_; V92: params_, info_, cp_), except the V92 mapping parameters (stored before V92createConstellations() /
 *   V92createFilterCoefficients(), which read it back from the member).
 * - V90: side 0 stores dem_ = NULL after the modulator constructor; side 1 stores mod_ = NULL before the demodulator allocation and passes the
 *   hardware codec type of the modem parameter block (params->codecType).
 * - V92: side 1 builds the modulator with the DIL constructor ARGUMENT (not the member); side 0 only stores NULL.
 * - The V92Modem destructor ends with `info_ = NULL` after freeing it; KEEP_STORES_TO() keeps -flifetime-dse from deleting that store.
 * - The debug level is re-read before every print (a print may change it): every `if (dbgOn())` is an unsigned compare against the memory word.
 *
 * Hazards:
 * - None of the sysdep_malloc() results is checked for NULL.
 * - A side value other than 0 / 1 prints a diagnostic and leaves the pointer members (V90: mod_, dem_; V92: mod_) unset, and the destructor then frees
 *   them; callers pass 0 or 1 only.
 */

#include "pcm_modem.h"
#include "core/dsp_asm.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

/* ---- message and banner strings ---- */
#define S_STARS "*********************************************************\r\n"
#define S_PRIVATE "*******         This is a PRIVATE version         *******\r\n"
#define S_FORSL "*******     for the use of SL DSP group only      *******\r\n"
#define S_PCT_S "%s\r\n"
#define S_DIGITAL "Digital"
#define S_ANALOG "Analog"
#define S90_CONSTRUCTION "V90Modem Construction (as %s Modem)\r\n"
#define S90_CTOR_ILLEGAL "V90Modem Constructor: Illegal modemSide\r\n"
#define S90_DESTRUCTION "V90Modem Destruction\r\n"
#define S90_DTOR_ILLEGAL "V90Modem Destructor: Illegal modemSide\r\n"
#define S90_VERSION "V90Modem Version: %s  (%s)\r\n"
#define S90_VERSION_NO "2.98"
#define S90_VERSION_DATE "25-Mar-04"
#define S90_DESC_HDR "V90Modem Version Description:\r\n"
#define S90_DESC "Modified Quick Connect without Memory + Train Time + Constel Power"
#define S90_COMPONENTS "Components: Floreat, ADI, ACD, New BLL\r\n"
#define S90_RESET "V90Modem Reset, qcFlag = %d\r\n"
#define S90_RESET_ILLEGAL "V90Modem Reset: Illegal modemSide\r\n"
#define S90_QC_MASKED "due to probe mode quick connect is masked !!!\r\n"
#define S90_PROGRESS_ILLEGAL "V90Modem progress: Illegal modemSide\r\n"
#define S92_CONSTRUCTION "V92Modem Construction (as %s Modem)\r\n"
#define S92_CTOR_ILLEGAL "V92Modem Constructor: Illegal modemSide\r\n"
#define S92_DESTRUCTION "V92Modem Destruction\r\n"
#define S92_DTOR_ILLEGAL "V92Modem Destructor: Illegal modemSide\r\n"
#define S92_VERSION "V92Modem Version: %s  (%s)\r\n"
#define S92_VERSION_NO "1.1"
#define S92_VERSION_DATE "9-Apr-01"
#define S92_DESC_HDR "V92Modem Version Description:\r\n"
#define S92_DESC "Memory cleanups + dil descriptor crash fix"
#define S92_RESET "V92Modem Reset\r\n"
#define S92_RESET_ILLEGAL "V92Modem Reset: Illegal modemSide\r\n"
#define S92_PROGRESS_ILLEGAL "V92Modem progress: Illegal modemSide\r\n"

#include "core/dsp_debug.h"

/* ---- helpers (static inline) ---- */

/* The version line both printTitle() methods print between the star lines: nested debug-level tests, the level re-read after every print. */
static inline void bannerVersion(const char* stars, const char* fmt, const char* no, const char* date)
{
  if (dbgOn())
  {
    dsplibs_debug_printf(stars);
    if (dbgOn())
    {
      dsplibs_debug_printf(fmt, no, date);
      if (dbgOn())
        dsplibs_debug_printf(stars);
    }
  }
}

/* ---- V90Modem ---- */

/** @brief Print the version banner and description (edprintf() lines; the closing star line only at debug level above 1). */
void V90Modem::printTitle()
{
  edprintf(S_STARS);
  edprintf(S_PRIVATE);
  edprintf(S_FORSL);
  bannerVersion(S_STARS, S90_VERSION, S90_VERSION_NO, S90_VERSION_DATE);
  edprintf(S90_DESC_HDR);
  edprintf(S_PCT_S, S90_DESC);
  edprintf(S90_COMPONENTS);
  if (dbgOn())
    dsplibs_debug_printf(S_STARS);
}

/* Constructor body: build the parameters, phase-2 info and Jd objects, then the modulator (side 0) or the demodulator (side 1). */
static inline void ctorBody90(V90Modem* m, V90ModemSide side, _tagModemParameters* p, tagV90DILdescriptor* dil, unsigned blk, V90ComputationalMode mode,
                              unsigned session)
{
  if (dbgOn())
    dsplibs_debug_printf(S90_CONSTRUCTION, side == 0 ? S_DIGITAL : S_ANALOG);
  m->printTitle();

  m->side_ = (unsigned)side;
  m->dil_ = dil;
  m->session_ = session;

  /* helper objects shared by both sides */
  {
    V90Parameters* pr = (V90Parameters*)sysdep_malloc(sizeof(V90Parameters));
    new (pr) V90Parameters(p);
    m->params_ = pr;
  }
  {
    V90Phase2Info* i = (V90Phase2Info*)sysdep_malloc(sizeof(V90Phase2Info));
    new (i) V90Phase2Info(m->params_);
    m->info_ = i;
  }
  {
    V90Jd* j = (V90Jd*)sysdep_malloc(sizeof(V90Jd));
    new (j) V90Jd(m->params_);
    m->jd_ = j;
  }
  {
    V92Jd* j = (V92Jd*)sysdep_malloc(sizeof(V92Jd));
    new (j) V92Jd(m->params_);
    m->jd92_ = j;
  }

  /* the active side */
  if (m->side_ == 0)
  {
    V90Modulator* mo = (V90Modulator*)sysdep_malloc(sizeof(V90Modulator));
    new (mo) V90Modulator(blk, m->info_, m->jd_, m->jd92_, m->dil_, &m->mp1_, &m->mp2_, &m->addCp_, &m->cp_, &m->mp_, m->params_, m->session_);
    m->mod_ = mo;
    m->dem_ = 0;
  }
  else if (m->side_ == 1)
  {
    V90Demodulator* d;
    m->mod_ = 0;
    d = (V90Demodulator*)sysdep_malloc(sizeof(V90Demodulator));
    new (d) V90Demodulator(blk, m->info_, m->jd_, m->jd92_, m->dil_, &m->mp1_, &m->mp2_, &m->addCp_, &m->cp_, &m->mp_,
                           (__tHardwareCodecTypes__)(unsigned)p->codecType, m->params_, mode, m->session_);
    m->dem_ = d;
  }
  else if (dbgOn())
  {
    dsplibs_debug_printf(S90_CTOR_ILLEGAL);
  }
}

/**
 * @brief Construct a V.90 modem side.
 * @param side     V90_SIDE_DIGITAL (builds the modulator) or V90_SIDE_ANALOG (builds the demodulator)
 * @param params   modem parameter block
 * @param dil      DIL descriptor owned by the caller
 * @param blk      block size passed to the modulator / demodulator
 * @param mode     computational mode (demodulator only)
 * @param session  session flag
 * @warning The sysdep_malloc() results are not checked; any other `side` value prints a diagnostic and leaves mod_ / dem_ unset.
 */
V90Modem::V90Modem(V90ModemSide side, _tagModemParameters* params, tagV90DILdescriptor* dil, unsigned blk, V90ComputationalMode mode, unsigned session)
    : mp_(), cp_()
{
  ctorBody90(this, side, params, dil, blk, mode, session);
}

/* Destructor body: free the active side and the helper objects (info_ has no destructor to run). */
static inline void dtorBody90(V90Modem* m)
{
  if (dbgOn())
    dsplibs_debug_printf(S90_DESTRUCTION);
  if (m->side_ > 1)
  {
    if (dbgOn())
      dsplibs_debug_printf(S90_DTOR_ILLEGAL);
  }
  if (m->mod_)
  {
    m->mod_->~V90Modulator();
    sysdep_free(m->mod_);
  }
  if (m->dem_)
  {
    m->dem_->~V90Demodulator();
    sysdep_free(m->dem_);
  }
  if (m->info_)
    sysdep_free(m->info_);
  if (m->jd_)
  {
    m->jd_->~V90Jd();
    sysdep_free(m->jd_);
  }
  if (m->jd92_)
  {
    m->jd92_->~V92Jd();
    sysdep_free(m->jd92_);
  }
  if (m->params_)
  {
    m->params_->~V90Parameters();
    sysdep_free(m->params_);
  }
}

/** @brief Destroy the modulator / demodulator, the Jd objects, the phase-2 info and the parameter block. */
V90Modem::~V90Modem() { dtorBody90(this); }

/**
 * @brief Reset the active side.
 * @param qc  quick-connect flag (demodulator side only; forced to 0 in probing mode, and selects the ADI_QC DIL option)
 */
void V90Modem::reset(unsigned qc)
{
  if (dbgOn())
    dsplibs_debug_printf(S90_RESET, qc);
  printTitle();
  if (side_ == 0)
  {
    mod_->reset();
  }
  else if (side_ == 1)
  {
    if (params_->probingMode != 0)
    {
      edprintf(S90_QC_MASKED);
      qc = 0;
    }
    setDilDescriptor(dil_, qc != 0 ? DIL_ADI_QC : DIL_ADI);
    dem_->reset(qc);
  }
  else if (dbgOn())
  {
    dsplibs_debug_printf(S90_RESET_ILLEGAL);
  }
}

/** @brief Store the session flag and forward it to the modulator or demodulator. */
void V90Modem::setSessionFlag(unsigned f)
{
  unsigned side = side_;
  session_ = f;
  if (side == 0)
    mod_->setSessionFlag(f);
  else if (side == 1)
    dem_->setSessionFlag(f);
}

/**
 * @brief Run the active side over one block.
 * @param bits    side 0: input bits; side 1: output bits
 * @param nBits   side 0: number of input bits; side 1: number of output bits
 * @param samples side 0: output samples; side 1: input samples
 * @param n       sample count
 */
void V90Modem::progress(int* bits, unsigned& nBits, float* samples, unsigned n)
{
  if (side_ == 0)
  {
    mod_->progress(bits, nBits, samples, n);
  }
  else if (side_ == 1)
  {
    dem_->progress(bits, nBits, samples, n);
  }
  else if (dbgOn())
  {
    dsplibs_debug_printf(S90_PROGRESS_ILLEGAL);
  }
}

/* ---- V92Modem ---- */

/** @brief Print the version banner and description. */
void V92Modem::printTitle()
{
  edprintf(S_STARS);
  edprintf(S_PRIVATE);
  edprintf(S_FORSL);
  bannerVersion(S_STARS, S92_VERSION, S92_VERSION_NO, S92_VERSION_DATE);
  edprintf(S92_DESC_HDR);
  edprintf(S_PCT_S, S92_DESC);
  edprintf(S_STARS);
}

/* Constructor body: build the parameters, phase-2 info, CP and mapping arrays, then the modulator (side 1). */
static inline void ctorBody92(V92Modem* m, V92ModemSide side, _tagModemParameters* p, unsigned blk, tagV90DILdescriptor* dil)
{
  if (dbgOn())
    dsplibs_debug_printf(S92_CONSTRUCTION, side == 0 ? S_DIGITAL : S_ANALOG);
  m->printTitle();

  m->side_ = (unsigned)side;
  {
    V92Parameters* pr = (V92Parameters*)sysdep_malloc(sizeof(V92Parameters));
    new (pr) V92Parameters(p);
    m->params_ = pr;
  }
  m->dil_ = dil;
  {
    V92Phase2Info* i = (V92Phase2Info*)sysdep_malloc(sizeof(V92Phase2Info));
    new (i) V92Phase2Info(m->params_);
    m->info_ = i;
  }
  {
    V92CP* c = (V92CP*)sysdep_malloc(sizeof(V92CP));
    new (c) V92CP();
    m->cp_ = c;
  }

  m->mapping_ = (V92MappingParams*)sysdep_malloc(sizeof(V92MappingParams));
  V92createConstellations(m->mapping_);
  V92createFilterCoefficients(m->mapping_);

  /* the active side */
  if (m->side_ == 0)
  {
    m->mod_ = 0;
  }
  else if (m->side_ == 1)
  {
    V92Modulator* mo = (V92Modulator*)sysdep_malloc(sizeof(V92Modulator));
    new (mo) V92Modulator(blk, m->info_, (V92Ja*)&m->ja_, dil, m->cp_, m->mapping_, m->params_);
    m->mod_ = mo;
  }
  else if (dbgOn())
  {
    dsplibs_debug_printf(S92_CTOR_ILLEGAL);
  }
}

/**
 * @brief Construct a V.92 modem side.
 * @param side    V92_SIDE_ANALOG builds the modulator; side 0 builds nothing
 * @param params  modem parameter block
 * @param blk     block size passed to the modulator
 * @param dil     DIL descriptor owned by the caller
 * @param mode    ignored
 * @warning The sysdep_malloc() results are not checked; any other `side` value leaves mod_ unset.
 */
V92Modem::V92Modem(V92ModemSide side, _tagModemParameters* params, unsigned blk, tagV90DILdescriptor* dil, V92ComputationalMode mode)
{
  (void)mode;
  ctorBody92(this, side, params, blk, dil);
}

/* Destructor body: free the mapping arrays, the active side, the CP, the parameter block and the phase-2 info. */
static inline void dtorBody92(V92Modem* m)
{
  if (dbgOn())
    dsplibs_debug_printf(S92_DESTRUCTION);
  if (m->side_ > 1)
  {
    if (dbgOn())
      dsplibs_debug_printf(S92_DTOR_ILLEGAL);
  }

  V92deleteConstellations(m->mapping_);
  V92deleteFilterCoefficients(m->mapping_);
  if (m->mapping_)
    sysdep_free(m->mapping_);
  if (m->mod_)
  {
    m->mod_->~V92Modulator();
    sysdep_free(m->mod_);
  }
  if (m->cp_)
  {
    m->cp_->~V92CP();
    sysdep_free(m->cp_);
  }
  if (m->params_)
  {
    m->params_->~V92Parameters();
    sysdep_free(m->params_);
  }
  if (m->info_)
  {
    sysdep_free(m->info_);
    m->info_ = 0;
    KEEP_STORES_TO(m); /* keeps -flifetime-dse from deleting the store */
  }
}

/**
 * @brief Destroy the mapping arrays, the modulator, the CP, the parameter block and the phase-2 info.
 * @warning V92deleteConstellations() dereferences `mapping_` without a NULL test.
 */
V92Modem::~V92Modem() { dtorBody92(this); }

/** @brief Reset the active side: pack the DIL frame into the Ja block, reset the modulator and enter phase 3 (side 1); side 0 does nothing. */
void V92Modem::reset()
{
  if (dbgOn())
    dsplibs_debug_printf(S92_RESET);
  printTitle();
  if (side_ == 0)
    return;
  if (side_ == 1)
  {
    V92DILdescriptorPacker(dil_, &ja_.bits_[0], (int*)&ja_.len_);
    mod_->reset();
    mod_->enterPhase3();
  }
  else if (dbgOn())
  {
    dsplibs_debug_printf(S92_RESET_ILLEGAL);
  }
}

/** @brief Run the modulator over one block (side 1); side 0 does nothing.  Arguments as V90Modem::progress(). */
void V92Modem::progress(int* bits, unsigned& nBits, float* samples, unsigned n)
{
  if (side_ == 0)
    return;
  if (side_ == 1)
    mod_->progress(bits, nBits, samples, n);
  else if (dbgOn())
    dsplibs_debug_printf(S92_PROGRESS_ILLEGAL);
}
