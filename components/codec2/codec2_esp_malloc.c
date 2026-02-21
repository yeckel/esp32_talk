/* codec2_esp_malloc.c
 * Provides codec2_malloc / codec2_calloc / codec2_free for the __EMBEDDED__
 * build path on ESP-IDF.  These are declared as extern in debug_alloc.h when
 * __EMBEDDED__ is defined; we just forward them to the standard heap.
 */
#include <stdlib.h>

void *codec2_malloc(size_t size)             { return malloc(size);        }
void *codec2_calloc(size_t nmemb, size_t sz) { return calloc(nmemb, sz);  }
void  codec2_free(void *ptr)                 { free(ptr);                  }
