/**
 * @file tone_table_words.c
 * @brief Table-word registry of the tone-detect core: the lookup behind tone_word(), tone_ptr() and tone_tab_register().
 *
 * Only built for 64-bit hosts; on a 32-bit host a table word is the address itself and this file is empty (see tone_detect.h).
 * A word is (id << 16) | byte offset.  Ids start at 1: ids 1 .. tone_ntabs are the built-in tables tone_tabs[] (tone_tables.c), the following ids are
 * the tables registered at run time with tone_tab_register() (up to 64).  The registry is process-global and not locked.
 */
#include "tone_detect.h"

#if UINTPTR_MAX != 0xffffffffu

#define TONE_MAX_EXTRA 64
static ToneTab g_extra[TONE_MAX_EXTRA]; /* tables registered at run time */
static unsigned g_nextra;               /* number of entries used in g_extra */

/* registry entry of a 1-based table id, or NULL if the id is unknown */
static const ToneTab* tab_by_id(unsigned id)
{
  if (id >= 1 && id <= tone_ntabs)
    return &tone_tabs[id - 1];
  if (id > tone_ntabs && id - tone_ntabs <= g_nextra)
    return &g_extra[id - tone_ntabs - 1];
  return NULL;
}

/* the table holding p: its id (1 based) and, in *off, the byte offset of p inside it; 0 when p is in none */
static unsigned tab_find(const void* p, unsigned* off)
{
  unsigned i, n = tone_ntabs + g_nextra;

  for (i = 1; i <= n; i++)
  {
    const ToneTab* t = tab_by_id(i);
    const char* b = (const char*)t->base;
    if ((const char*)p >= b && (const char*)p < b + t->bytes)
    {
      *off = (unsigned)((const char*)p - b);
      return i;
    }
  }
  return 0;
}

/**
 * @brief Convert a pointer into a registered table to its table word.
 * @param p  address inside one of the built-in or registered tables, or NULL
 * @return (table id << 16) | byte offset; 0 for NULL or an address in no registered table
 * @note  The byte offset must fit in 16 bits (the largest built-in table is 168 bytes).
 */
int tone_word(const void* p)
{
  unsigned off, id;

  if (!p)
    return 0;

  id = tab_find(p, &off);
  return id ? (int)((id << 16) | off) : 0;
}

/**
 * @brief Convert a table word back to a pointer.
 * @param w  table word from tone_word()
 * @return base of the table named by the high 16 bits plus the byte offset in the low 16 bits; NULL if the id is unknown (including 0)
 * @note  The offset is not checked against the size of the table.
 */
const short* tone_ptr(int w)
{
  const ToneTab* t = tab_by_id((unsigned)w >> 16);
  return t ? (const short*)((const char*)t->base + ((unsigned)w & 0xffffu)) : NULL;
}

/**
 * @brief Make an array addressable by a table word.
 * @param name   label for diagnostics (the pointer is stored, not copied)
 * @param base   first element of the array
 * @param bytes  size of the array in bytes
 * @return the table id; 0 if the registry (64 run-time entries) is full.  Registering the same base again updates the name and keeps the larger size.
 */
int tone_tab_register(const char* name, const void* base, unsigned bytes)
{
  unsigned i;

  /* an already registered array: update its entry */
  for (i = 0; i < g_nextra; i++)
    if (g_extra[i].base == base)
    {
      g_extra[i].name = name;
      g_extra[i].bytes = bytes > g_extra[i].bytes ? bytes : g_extra[i].bytes;
      return (int)(tone_ntabs + i + 1);
    }

  /* a new entry */
  if (g_nextra >= TONE_MAX_EXTRA)
    return 0;
  g_extra[g_nextra].name = name;
  g_extra[g_nextra].base = (const short*)base;
  g_extra[g_nextra].bytes = bytes;
  return (int)(tone_ntabs + ++g_nextra);
}
#else
typedef int tone_words_empty_on_32bit; /* a table word is the address on a 32-bit host: nothing to define (keeps the translation unit non-empty) */
#endif
