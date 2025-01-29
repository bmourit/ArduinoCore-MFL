#pragma once

#include "Arduino.h"

enum class SysTick_Error : uint8_t {
    TICK_OK,
    TICK_ERROR,
    TICK_TIMEOUT
};

extern uint32_t msTickPriority;

SysTick_Error tickInit(uint32_t priority);
uint32_t getTickPriority();
void tickInc();

uint32_t getCurrentMillis();
uint32_t getCurrentMicros();
