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

    using EXTICallback = voidFuncPtrParam;

    void enablePinExtiInterrupt(pin_size_t pin, EXTICallback callback, void* param, exti::EXTI_Trigger type);
    void disablePinExtiInterrupt(pin_size_t pin);
    void handleCallback(gpio::Pin_Number pin);

private:
    inline static constexpr std::array<exti_to_irq, maxExtiLines_> irq_index {{
        {0U, EXTI0_IRQn},       {1U, EXTI1_IRQn},       {2U, EXTI2_IRQn},       {3U, EXTI3_IRQn},
        {4U, EXTI4_IRQn},       {5U, EXTI5_9_IRQn},     {6U, EXTI5_9_IRQn},     {7U, EXTI5_9_IRQn},
        {8U, EXTI5_9_IRQn},     {9U, EXTI5_9_IRQn},     {10U, EXTI10_15_IRQn},  {11U, EXTI10_15_IRQn},
        {12U, EXTI10_15_IRQn},  {13U, EXTI10_15_IRQn},  {14U, EXTI10_15_IRQn},  {15U, EXTI10_15_IRQn}
    }};

    ExtiManager();

    exti::EXTI& exti_;
    std::array<EXTICallback, maxExtiLines_> callbacks_;
    std::array<void*, maxExtiLines_> params_;
    std::array<bool, maxExtiLines_> active_;

    inline auto extiToIrq(uint8_t extiIndex) -> IRQn_Type {
        if (extiIndex < maxExtiLines_) {
            return irq_index[extiIndex].irq_type;
        }
        return INVALID_IRQ;
    }
};
