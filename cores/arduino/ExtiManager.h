#pragma once

#include <array>

#include "Arduino.h"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"

#ifndef EXTI_IRQ_PRIORITY
    #define EXTI_IRQ_PRIORITY       6U
#endif
#ifndef EXTI_IRQ_SUBPRIORITY
    #define EXTI_IRQ_SUBPRIORITY    0U
#endif

enum {
    MAX_EXTI_LINES = 16
};

inline constexpr uint8_t maxExtiLines_ = MAX_EXTI_LINES;

struct exti_to_irq {
    uint8_t line_number;
    IRQn_Type irq_type;
};

class ExtiManager {
public:
    static auto get_instance() -> ExtiManager&;

    using EXTICallback = void (*)();

    void enablePinExtiInterrupt(pin_size_t pin, EXTICallback callback, exti::EXTI_Trigger type);
    void disablePinExtiInterrupt(pin_size_t pin);
    void handleCallback(gpio::Pin_Number pin);

private:
    static std::array<exti_to_irq, maxExtiLines_> irq_index;

    ExtiManager();

    exti::EXTI& exti_;
    std::array<EXTICallback, maxExtiLines_> callbacks_;

    inline auto extiToIrq(uint8_t extiIndex) -> IRQn_Type {
        if (extiIndex < maxExtiLines_) {
            return irq_index[extiIndex].irq_type;
        }
        return INVALID_IRQ;
    }
};
