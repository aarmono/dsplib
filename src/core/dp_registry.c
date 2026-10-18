/**
 * @file dp_registry.c
 * @brief prop_dp_init() / prop_dp_exit(): register and deregister every data pump of the library with the host.
 *
 * These are the two entry points slmodemd's modem_main.c and modem_test.c call at start-up and shutdown.  Each calls the seven pump init (exit)
 * functions in a fixed order - call progress, Bell 103 / V.21, V.22, V.23, V.32, V.8, V.34 / PCM group - and discards their return values.  The exit
 * order is the same as the init order (not the reverse).  No argument, no state; nothing here depends on the word size.  Fax has no data-pump glue
 * (slmodemd calls FAX_create() / FAX_process() directly), so it is not registered.
 */
#include "dp_registry.h"

/**
 * @brief Register all data pumps with the host.
 * @return always 0; the results of the individual dp_*_init() calls are ignored
 * @note   slmodemd declares this function as returning int and ignores the value.
 */
int prop_dp_init(void)
{
  dp_call_init();
  dp_b103_init();
  dp_v22_init();
  dp_v23_init();
  dp_v32_init();
  dp_v8_init();
  dp_vpcm_init();
  return 0;
}

/**
 * @brief Deregister all data pumps from the host, in the same order as prop_dp_init() registered them.
 * @return always 0
 * @warning slmodemd (modem_main.c, modem_test.c) declares this function `void`, the library defines it `int`; the value is ignored on both sides, which
 *          is harmless on the supported ABIs but the two declarations differ.
 */
int prop_dp_exit(void)
{
  dp_call_exit();
  dp_b103_exit();
  dp_v22_exit();
  dp_v23_exit();
  dp_v32_exit();
  dp_v8_exit();
  dp_vpcm_exit();
  return 0;
}
