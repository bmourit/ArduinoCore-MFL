
#include <string_view>

#include "CoreHandler.h"

extern void core_debug(const char *format, ...);

// Default weak implementation of _Error_Handler
WEAK void _Error_Handler(const char *msg, int value) {
    // User can override this implementation to handle errors in their own way
    core_debug("Error: %s (%i)\n", msg, value);

    // Infinite loop to halt execution
    while (true) {
    }
}
