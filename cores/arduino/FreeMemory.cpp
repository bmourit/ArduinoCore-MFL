
#include "Arduino.h"

#if defined(__GNUC__)
    #include <sys/stat.h>
#endif

#include <stdint.h>
#include <errno.h>

#undef errno
extern int errno;

extern "C" {

    /**
     * @brief Weak alias of newlib's _sbrk (memory allocation for heap).
     *
     * This function is a weak alias of newlib's _sbrk function. It is used by
     * newlib to allocate memory for the heap. The function is not implemented
     * in the Arduino core, so it must be implemented by the user if the
     * Arduino program uses dynamic memory allocation.
     *
     * The function takes the number of bytes to increment the heap as an
     * argument and returns the address of the new heap end. If the heap
     * allocation fails because of heap and stack collision, or if the minimum
     * stack size is not maintained, the function sets errno to ENOMEM and
     * returns -1.
     *
     * @param[in]  incr  The number of bytes to increment the heap.
     * @return          The address of the new heap end, or -1 if the heap
     *                  allocation fails.
     */
    __attribute__((weak)) void* _sbrk(int incr) {
        extern char _estack;          // Defined in the linker script
        extern char _Min_Stack_Size;  // Defined in the linker script
        extern char _end;             // Defined by the linker

        static char* heap_end = &_end;
        char* prev_heap_end = heap_end;

        // Avoid heap and stack collision by checking MSP (Main Stack Pointer)
        if (heap_end + incr > (char *)__get_MSP()) {
            errno = ENOMEM;
            return (void*)-1;
        }

        // Ensure that a minimum stack size is maintained
        if (heap_end + incr >= (char *)(&_estack - &_Min_Stack_Size)) {
            errno = ENOMEM;
            return (void*)-1;
        }

        heap_end += incr;
        return (void*)prev_heap_end;
    }

} // extern "C"
