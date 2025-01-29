/*
  Copyright (c) 2025 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino.h"
#include "TickTime.h"

volatile uint32_t msTickCount_;
uint32_t msTickPriority = (1UL << __NVIC_PRIO_BITS);

/**
 * @brief Returns the number of milliseconds since the Arduino board began running the current program.
 *
 * This value will overflow (go back to zero), after approximately 50 days.
 * To avoid overflow, use unsigned long for variables that store the result.
 *
 * @return The number of milliseconds since the program started.
 */
unsigned long millis(void) {
    // TODO: ensure no interrupts
    return getCurrentMillis();
}

/**
 * @brief Returns the number of microseconds since the Arduino board began running the current program.
 *
 * This value will overflow (go back to zero), after approximately 70 minutes.
 * To avoid overflow, use unsigned long for variables that store the result.
 *
 * There are 1,000 microseconds in a millisecond and 1,000,000 microseconds in a second.
 *
 * @return The number of microseconds since the program started.
 */
unsigned long micros(void) {
    return getCurrentMicros();
}

/**
 * @brief Pauses the program for the specified number of milliseconds.
 *
 * The delay is accurate to within 1-2 milliseconds. The function is interrupt-compatible.
 *
 * @param ms The number of milliseconds to pause.
 */
void delay(unsigned long ms) {
    if (ms != 0U) {
        uint32_t start = getCurrentMillis();
        do {
            yield();
        } while (getCurrentMillis() - start < ms);
    }
}

/**
 * @brief Delays program execution for the specified number of microseconds.
 *
 * The delay is accurate to within 1-2 microseconds. The function is interrupt-compatible.
 *
 * @param usec The number of microseconds to delay.
 */
void delayMicroseconds(unsigned int usec) {
    volatile uint32_t tickCount = SysTick->VAL;
    const uint32_t msTicks = SysTick->LOAD + 1U;
    const uint32_t numTicks = ((usec - ((usec > 0U) ? 1U : 0U)) * msTicks) / 1000U;
    uint32_t elapsedTicks = 0U;
    volatile uint32_t prevTicks = tickCount;
    do {
        tickCount = SysTick->VAL;
        elapsedTicks += (prevTicks < tickCount) ? msTicks + prevTicks - tickCount : prevTicks - tickCount;
        prevTicks = tickCount;
    } while (numTicks > elapsedTicks);
}

void tickIncrement() { 
    msTickCount_ = msTickCount_ + 1U;
}

/**
 * @brief Initializes the system tick.
 *
 * Initializes the system tick to generate interrupts at a period of 1ms.
 * The priority of the interrupt is set to the provided parameter.
 *
 * @param[in] priority The priority of the system tick interrupt.
 * @return TICK_OK if the initialization was successful, TICK_ERROR otherwise.
 */
SysTick_Error tickInit(uint32_t priority) {
    uint32_t clock = rcu::RCU::get_instance().get_system_clock();
    if (SysTick_Config(clock / 1000U) > 0U) {
        return SysTick_Error::TICK_ERROR;
    }

    if (priority < (1UL << __NVIC_PRIO_BITS)) {
        uint32_t priority_group = NVIC_GetPriorityGrouping();
        NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(priority_group, priority, 0U));
        msTickPriority = priority;
    } else {
        return SysTick_Error::TICK_ERROR;
    }

    return SysTick_Error::TICK_OK;
}

/**
 * @brief Retrieves the current priority level for the SysTick interrupt.
 *
 * @return The current priority level of the SysTick interrupt as a uint32_t.
 */
uint32_t getTickPriority() {
    return msTickPriority;
}

/**
 * @brief Retrieves the current milliseconds count.
 *
 * @return The current value of msTickCount_ as a uint32_t.
 */
uint32_t getCurrentMillis() {
    return msTickCount_;
}

/**
 * @brief Retrieves the current time in microseconds.
 *
 * This function calculates the current microsecond count by combining
 * the current milliseconds and the system tick value, handling potential rollovers.
 *
 * @return Current time in microseconds.
 */
uint32_t getCurrentMicros() {
    uint32_t ms0 = getCurrentMillis();
    volatile uint32_t usec0 = SysTick->VAL;
    uint32_t ms1 = getCurrentMillis();
    volatile uint32_t usec1 = SysTick->VAL;
    const uint32_t load = SysTick->LOAD + 1U;

    // If milliseconds rolled over during read, use the later values
    if (ms0 != ms1) {
        ms0 = ms1;
        usec0 = usec1;
    }

    // Calculate current time in microseconds
    return (ms0 * 1000U + ((load - usec0) * 1000U) / load);
}

extern "C" {

    /**
     * @brief Handler for system tick interrupts.
     *
     * This function is the interrupt service routine (ISR) for the system tick.
     * It simply calls the tickIncrement() function to increment the tick count.
     *
     * @note This function is called automatically by the MCU when the system tick
     *       interrupt is triggered.
     */
    void SysTick_Handler(void) {
        tickIncrement();
    }

} // extern "C"
