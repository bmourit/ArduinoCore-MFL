#pragma once

#include <stdint.h>
#include <stdarg.h>
#ifdef CORE_DEBUG
#include <stdio.h>
#endif

// Arduino core version number
#define VERSION_MAJOR   1	// Major version
#define VERSION_MINOR   0	// Minor version
#define VERSION_PATCH   1	// Patch version
#define CORE_VERSION    (VERSION_MAJOR << 16) | (VERSION_MINOR << 8) | (VERSION_PATCH)

// libc porting layers for GCC
#if defined(__GNUC__)
#define WEAK  __attribute__((weak))
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Error handler
void _ErrorHandler(const char* msg, int value);
#define ErrorHandler() _ErrorHandler(__FILE__, __LINE__)

// Core debug
inline void core_debug(const char* format, ...) {
#ifdef CORE_DEBUG
    va_list arglist;
    va_start(arglist, format);
    vfprintf(stderr, format, arglist);
    va_end(arglist);
#else
    (void)(format);
#endif
}

#ifdef __cplusplus
}
#endif
