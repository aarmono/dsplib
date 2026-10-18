/**
 * @file dp_registry.h
 * @brief The entry points of the data pumps that prop_dp_init() / prop_dp_exit() (core/dp_registry.c) call, and dp_param_get().
 *
 * Each of the seven data pumps (call progress, Bell 103 / V.21, V.22, V.23, V.32, V.8, and the V.34 / PCM group) defines one init / exit pair:
 * dp_<name>_init() registers the pump's struct dp_operations with slmodemd (modem_dp_register), dp_<name>_exit() deregisters it.  This header holds
 * the only declaration of each.  dp_param_get() (src/vpcm/dp_vpcm.cpp) is modem_get_param(m, 0xa), the modem parameter block, which the C glue of
 * core/dp_glue.h calls.
 *
 * Freestanding and usable from the C++ half (the pumps in src/vpcm are C++): no includes, extern "C" guards, include-guarded.
 */
#ifndef DSPLIBS_PORT_DP_REGISTRY_H
#define DSPLIBS_PORT_DP_REGISTRY_H

#ifdef __cplusplus
extern "C"
{
#endif

  extern int dp_call_init(void);
  extern void dp_call_exit(void);
  extern int dp_b103_init(void);
  extern void dp_b103_exit(void);
  extern int dp_v22_init(void);
  extern void dp_v22_exit(void);
  extern int dp_v23_init(void);
  extern void dp_v23_exit(void);
  extern int dp_v32_init(void);
  extern void dp_v32_exit(void);
  extern int dp_v8_init(void);
  extern void dp_v8_exit(void);
  extern int dp_vpcm_init(void);
  extern void dp_vpcm_exit(void);

  /** modem_get_param(m, 0xa): the modem parameter block.  The argument is the host's struct modem *, kept opaque. */
  extern long dp_param_get(void* modem);

#ifdef __cplusplus
}
#endif

#endif /* DSPLIBS_PORT_DP_REGISTRY_H */
