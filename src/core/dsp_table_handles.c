/**
 * @file dsp_table_handles.c
 * @brief x86-64 only: the handle <-> table pointer registry behind CFGP() (see core/dsp_table_handles.h); empty on a 32-bit build.
 *
 * `g_cfgp[handle]` holds the table pointer of each handle.  The table files fill it from their constructors (CFGP_REGISTER()), before main().
 */
#include "dsp_table_handles.h"
#if UINTPTR_MAX != 0xffffffffu
static const void* g_cfgp[CFGH_COUNT];

/** Register the table pointer `p` under `handle`; a handle outside 1 .. CFGH_COUNT-1 is ignored. */
void cfgp_set(int handle, const void* p)
{
  if (handle > 0 && handle < CFGH_COUNT)
    g_cfgp[handle] = p;
}

/** The table pointer registered for `handle`; NULL for 0, an unknown handle, or a table file that is not linked (nothing registered). */
const void* cfgp_ptr(int handle) { return (handle > 0 && handle < CFGH_COUNT) ? g_cfgp[handle] : (const void*)0; }

/** The handle of the table pointer `p` (linear search of the registry); 0 for NULL or for a pointer that is not a registered table. */
int cfgp_word(const void* p)
{
  int i;
  if (p == (const void*)0)
    return 0;
  for (i = 1; i < CFGH_COUNT; i++)
    if (g_cfgp[i] == p)
      return i;
  return 0;
}
#endif
