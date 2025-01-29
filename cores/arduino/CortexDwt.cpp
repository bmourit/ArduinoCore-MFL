
#include "CortexDwt.h"

CortexDWT& CortexDWT::get_instance() {
    static CortexDWT instance;
    return instance;
}

CortexDWT::CortexDWT() {}

/**
 * @brief Initializes the DWT to be used as a cycle counter.
 *
 * Enables tracing, resets the cycle counter, and enables the cycle counter.
 * @return Clear if the initialization failed, Set if it succeeded.
 */
uint32_t CortexDWT::init() {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    __DSB();

    return (DWT->CYCCNT) ? Clear : Set;
}
