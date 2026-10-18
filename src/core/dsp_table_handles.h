/**
 * @file dsp_table_handles.h
 * @brief Table-pointer words of the 32-bit configuration blocks: CFGP() handles that stay 32 bits wide on a 64-bit host.
 *
 * The configuration blocks of the modem objects (FPM_TONE_CFG_raw, AGCv32_CFG_raw, SREv32_CFG_raw ...) are arrays of 32-bit WORDS, some of which
 * are the address of another table (ToneLPF, V32_AGC_DEF_ALPHA ...).  The consumers copy these words into 4-byte slots of the 32-bit-image objects,
 * so the slot has to stay 32 bits wide.
 *
 * - 32-bit host (pointer == 32 bits): CFGP(sym) is `(int)(intptr_t)sym` and cfgp_ptr(w) is `(const void *)(intptr_t)w`; the word is the address.
 * - x86-64: an address cannot be a 32-bit initializer.  CFGP(sym) is a small HANDLE (the enum constant CFGH_<sym>, 1 .. CFGH_COUNT-1; 0 = NULL)
 *   and cfgp_ptr(w) maps a handle back to the REAL pointer (src/core/dsp_table_handles.c).  Every table file registers its symbols from a
 *   constructor, so a table file that is not linked leaves its handles NULL.
 *
 * A word that holds a handle must never be used as an address directly: the producer writes CFGP(sym) / cfgp_word(p), the consumer reads
 * cfgp_ptr(word).  Comparing two words (`*(int*)mtd == CFGP(V32_S_DATA_COEF)`) is valid on both hosts.
 *
 * Adding a table symbol: put it in the CFGP_SYMS_<file> list below (the file that DEFINES it) and call CFGP_REGISTER(<file>) at the end of that file.
 */
#ifndef DSPLIBS_PORT_CFGPTR_H
#define DSPLIBS_PORT_CFGPTR_H
#include <stdint.h>

/* Symbol lists, one per defining file (X-macros: X(sym) is applied to each symbol). */
#define CFGP_SYMS_fpm_tables(X) X(ToneLPF) X(MTDb103_COEF) X(DEF_COEFS)
#define CFGP_SYMS_v32_tables(X) \
  X(V32_S_DATA_COEF) X(V32_AGC_DEF_ALPHA) X(V32_AGC_DEF_BETA) X(MRFv32_COFFS) X(ECCv32_IMAP) X(ECCv32_QMAP) \
  X(SREv32_COFFS) X(SREv32_XB_COFFS) X(SREv32_xCLOCK) X(SREv32_yCLOCK) X(SREv32_PLL_K1) X(SREv32_PLL_K2) \
  X(FSEv32_ICOFF) X(FSEv32_QCOFF) X(CRRv32_CLK) X(CRRv32_PLL_K1) X(CRRv32_PLL_K2) X(PPSv32_ICOFFS) X(PPSv32_QCOFFS)
#define CFGP_SYMS_v32_tables_vtb(X) \
  X(SMCv32_IMAP16) X(SMCv32_QMAP16) X(VTBv32_IMAP16T) X(VTBv32_QMAP16T) X(VTBv32_IMAP32) X(VTBv32_QMAP32) X(VTBv32_IMAP64) X(VTBv32_QMAP64) \
  X(VTBv32_IMAP128) X(VTBv32_QMAP128) X(VTB_BOUND_7200) X(VTB_BOUND_9600) X(VTB_BOUND_12000) X(VTB_BOUND_14400) \
  X(VTB_REGION_7200) X(VTB_REGION_9600) X(VTB_REGION_12000) X(VTB_REGION_14400)
#define CFGP_SYMS_b103_v23_tables(X) \
  X(B103_AGC_DEF_ALPHA) X(B103_AGC_DEF_BETA) X(V23_AGC_DEF_ALPHA) X(V23_AGC_DEF_BETA) X(V23RX_IIR_LPF) X(V23RX_ANSWER_INTRP) \
  X(V23_MRF_FILT) X(B103_MRF_FILT_TX) X(B103_MRF_FILT_RX) X(B103_IIR_LPF) X(B103_CHAN_INTRP) X(B103_BPF_ANSWER) X(B103_BPF_CALLER)
/* the V.8 tables whose address is stored in a 4-byte slot of the V.8 object (the detector's first slot and the filter-coefficient slots) */
#define CFGP_SYMS_v8_tables(X) \
  X(v8_c2100) X(LM_filter1_coef) X(LM_filter2_coef) X(LS_filter1_coef) X(LS_filter2_coef) X(HM_filter1_coef) X(HM_filter2_coef) \
  X(HS_filter1_coef) X(HS_filter2_coef)
#define CFGP_SYMS_v23_bwch_tables(X) X(V23_BWCH_AGC_DEF_ALPHA) X(V23_BWCH_AGC_DEF_BETA) X(fw_ch_samp_per_bit_table) X(bw_ch_samp_per_bit_table)
/* the fax V.21 channel tables (src/fax/fax_v21_tables.c) */
#define CFGP_SYMS_fax_v21_tables(X) \
  X(V21_MRF_FILT) X(AGC_DEF_ALPHA_v21) X(AGC_DEF_BETA_v21) X(V21RX_CHAN1_INTRP) X(V21RX_CHAN2_INTRP) X(V21RX_IIR_LPF) X(V21_CHAN1_MTD_COEFF) X(V21_CHAN2_MTD_COEFF)

/* the fax V.27ter receive tables (src/fax/fax_v27_rx_tables.c): every per-rate table whose address is stored in a cfg word of the MTD / MRF / SRE / FSE / AGC */
#define CFGP_SYMS_fax_v27rx_tables(X) \
  X(V27RX_FSE_IFILT_2400) X(V27RX_FSE_IFILT_4800) X(V27RX_FSE_QFILT_2400) X(V27RX_FSE_QFILT_4800) X(V27RX_CRR_TABLE_2400) X(V27RX_CRR_TABLE_4800) X(V27RX_FSE_PLLK1_2400) \
  X(V27RX_FSE_PLLK1_4800) X(V27RX_FSE_PLLK2_2400) X(V27RX_FSE_PLLK2_4800) X(V27RX_MRF_FILT_2400) X(V27RX_MRF_FILT_4800) X(V27RX_SRE_FILT_2400) X(V27RX_SRE_FILT_4800) \
  X(V27RX_XB_COFFS_2400) X(V27RX_XB_COFFS_4800) X(V27RX_XCLOCK_2400) X(V27RX_XCLOCK_4800) X(V27RX_YCLOCK_2400) X(V27RX_YCLOCK_4800) X(V27RX_SRE_PLLK1_2400) \
  X(V27RX_SRE_PLLK1_4800) X(V27RX_SRE_PLLK2_2400) X(V27RX_SRE_PLLK2_4800) X(V27_MTD_COEFF_2400) X(V27_MTD_COEFF_4800) X(V27RX_DEC_PMAP_2400) X(V27RX_DEC_PMAP_4800) \
  X(AGC_DEF_ALPHA_v27) X(AGC_DEF_BETA_v27)

/* the fax V.29 receive tables (src/fax/fax_v29_rx_tables.c): every table whose address is stored in a cfg word of the MTD / MRF / SRE / FSE / AGC */
#define CFGP_SYMS_fax_v29rx_tables(X) \
  X(V29RX_FSE_IFILT) X(V29RX_FSE_QFILT) X(V29RX_CRR_TABLE) X(V29RX_FSE_PLLK1) X(V29RX_FSE_PLLK2) X(V29RX_MRF_FILT) X(V29RX_SRE_FILT) X(V29RX_XB_COFFS) X(V29RX_XCLOCK) \
  X(V29RX_YCLOCK) X(V29RX_SRE_PLLK1) X(V29RX_SRE_PLLK2) X(V29_MTD_COEFF) X(AGC_DEF_ALPHA_v29) X(AGC_DEF_BETA_v29)

/* the fax V.17 receive tables (src/fax/fax_v17_rx_tables.c): every table whose address is stored in a cfg word of the MTD / MRF / SRE / FSE / AGC */
#define CFGP_SYMS_fax_v17rx_tables(X) \
  X(V17_MTD_COEFF) X(MRFv17_COFFS) X(SREv17_COFFS) X(SREv17_XB_COFFS) X(SREv17_xCLOCK) X(SREv17_yCLOCK) X(SREv17_PLL_K1) X(SREv17_PLL_K1_S) X(SREv17_PLL_K2) X(FSEv17_ICOFF) \
  X(FSEv17_QCOFF) X(CRRv17_CLK) X(CRRv17_PLL_K1) X(CRRv17_PLL_K1_S) X(CRRv17_PLL_K2) X(AGC_DEF_ALPHA_v17) X(AGC_DEF_BETA_v17)
/* the V.17 trellis maps (defined in src/fax/fax_v17_tx_tables.c, registered from src/fax/fax_v17_rx_tables.c): the receiver's Viterbi decoder (FaxVtb) holds them as handles */
#define CFGP_SYMS_fax_v17rx_vtb(X) \
  X(VTBv17_IMAP16T) X(VTBv17_QMAP16T) X(VTBv17_IMAP32) X(VTBv17_QMAP32) X(VTBv17_IMAP64) X(VTBv17_QMAP64) X(VTBv17_IMAP128) X(VTBv17_QMAP128)

#if UINTPTR_MAX == 0xffffffffu

/* ---- 32-bit host: the word is the address ---- */
/* CFGP(sym): the table word for symbol `sym`; cfgp_ptr(w): the table pointer for word `w`; cfgp_word(p): the word for pointer `p`.
 * Macros, not inline functions, so the -O0 code is the plain cast. */
#define CFGP(sym) ((int)(intptr_t)(sym))
#define cfgp_ptr(w) ((const void *)(intptr_t)(w))
#define cfgp_word(p) ((int)(intptr_t)(p))
/* CFGP_WORD(w): a table word as an integer base address (`*(short *)(CFGP_WORD(w) + i * 2)`): the word itself here; the resolved pointer as intptr_t on x86-64.
 * cfgp_int_t is the type for a local that holds one (int here). */
typedef int cfgp_int_t;
#define CFGP_WORD(w) (w)
/* CFGP_SLOT(T, addr): load the table pointer (type T, e.g. `const short *`) from the 4-byte object slot at addr: `*(T *)(addr)` here; on x86-64 the slot holds
 * a handle, which is resolved. */
#define CFGP_SLOT(T, addr) (*(T *)(addr))
/* CFGP_SLOT_IS(T, addr, sym): does the slot at addr hold table sym?   CFGP_SLOT_SET(T, addr, sym): store table sym into the slot (T as above). */
#define CFGP_SLOT_IS(T, addr, sym) (*(T *)(addr) == (T)(sym))
#define CFGP_SLOT_SET(T, addr, sym) (*(T *)(addr) = (T)(sym))
#define CFGP_SLOT_PUT(T, addr, ptr) (*(T *)(addr) = (T)(ptr)) /* store an arbitrary table pointer (x86-64: its handle) */
#define CFGP_REGISTER(file) /* nothing: the word IS the address */

#else

/* ---- x86-64: the word is a handle; the registry (dsp_table_handles.c) maps it to the pointer ---- */

#define CFGP_ALL(X) \
  CFGP_SYMS_fpm_tables(X) CFGP_SYMS_v32_tables(X) CFGP_SYMS_v32_tables_vtb(X) CFGP_SYMS_b103_v23_tables(X) CFGP_SYMS_v23_bwch_tables(X) CFGP_SYMS_v8_tables(X) CFGP_SYMS_fax_v21_tables(X) CFGP_SYMS_fax_v27rx_tables(X) CFGP_SYMS_fax_v29rx_tables(X) CFGP_SYMS_fax_v17rx_tables(X) CFGP_SYMS_fax_v17rx_vtb(X)
#define CFGP_ENUM_(s) CFGH_##s,
enum { CFGH_NONE = 0, CFGP_ALL(CFGP_ENUM_) CFGH_COUNT };
#undef CFGP_ENUM_
#define CFGP(sym) ((int)CFGH_##sym)
const void *cfgp_ptr(int handle);        /* handle -> pointer (NULL for 0 / unknown / table file not linked) */
int cfgp_word(const void *p);
#define CFGP_SLOT(T, addr) ((T)cfgp_ptr(*(int *)(addr)))
typedef intptr_t cfgp_int_t;
#define CFGP_WORD(w) ((intptr_t)cfgp_ptr(w))
#define CFGP_SLOT_IS(T, addr, sym) (*(int *)(addr) == CFGP(sym))
#define CFGP_SLOT_SET(T, addr, sym) (*(int *)(addr) = CFGP(sym))
#define CFGP_SLOT_PUT(T, addr, ptr) (*(int *)(addr) = cfgp_word(ptr))            /* pointer -> handle (0 if p is not a registered table) */
void cfgp_set(int handle, const void *p);
#define CFGP_REG_(s) cfgp_set(CFGH_##s, (const void *)(s));
#define CFGP_REGISTER(file) \
  static void __attribute__((constructor)) cfgp_register_##file(void) { CFGP_SYMS_##file(CFGP_REG_) }

#endif
#endif
