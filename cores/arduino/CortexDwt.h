
#pragma once

#include <stdbool.h>
#include <limits>

#include "mfl_api.h"

class CortexDWT {
public:
    static CortexDWT& get_instance();

    uint32_t init();
    inline uint32_t getCycleCount() {
        return (DWT->CYCCNT);
    }
    inline uint32_t microsecondsMax() {
        return (std::numeric_limits<uint32_t>::max() / (RCU_I.get_system_clock() / 1'000'000U));
    }
    inline uint32_t millisecondsMax() {
        return (std::numeric_limits<uint32_t>::max() / (RCU_I.get_system_clock() / 1'000U));
    }
    inline uint32_t secondsMax() {
        return (std::numeric_limits<uint32_t>::max() / RCU_I.get_system_clock());
    }

private:
    CortexDWT();
};
