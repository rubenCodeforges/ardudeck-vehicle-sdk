/*
 * The entire C library the SDK uses, so it can be cross-compiled for a bare target
 * with no sysroot. If this file ever needs a fourth function, that is a change worth
 * arguing about: every addition is something a vendor's toolchain has to provide.
 */

#ifndef AD_FREESTANDING_STRING_H
#define AD_FREESTANDING_STRING_H

#include <stddef.h>

void *memcpy(void *dst, const void *src, size_t n);
void *memset(void *dst, int c, size_t n);
int strncmp(const char *a, const char *b, size_t n);

#endif
