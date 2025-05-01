#pragma once

#include <stdint.h>
#include <stdarg.h>
#ifdef CORE_DEBUG
    #include <stdio.h>
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
}   // extern "C"
#endif

