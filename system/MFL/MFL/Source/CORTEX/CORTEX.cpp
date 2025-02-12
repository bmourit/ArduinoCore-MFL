//
// MFL gd32f30x CORTEX peripheral register access in C++
//
// Copyright (C) 2025 B. Mouritsen <bnmguy@gmail.com>. All rights reserved.
//
// This file is part of the Microcontroller Firmware Library (MFL).
//
// MFL is free software: you can redistribute it and/or modify it under the terms of the
// GNU Lesser General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// MFL is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License along with MFL.
// If not, see <https://www.gnu.org/licenses/>.
//

#include "F303RE.hpp"
#include "CORTEX.hpp"

namespace cortex {

CORTEX& CORTEX::get_instance() {
    static CORTEX instance;
    return instance;
}

CORTEX::CORTEX() {}

/**
 * @brief Set low power mode enable
 *
 * @param[in] mode The low power mode to enable/disable
 * @param[in] enable If true, enable the low power mode. Otherwise, disable it.
 *
 * This function sets the corresponding bit in the System Control Register
 * (SCB->SCR) to enable or disable the specified low power mode. If the
 * parameter enable is true, the bit is set, and if it is false, the bit is
 * cleared.
 */
void CORTEX::set_low_power_mode_enable(Low_Power_Mode mode, bool enable) {
    enable ? (SCB->SCR |= (1U << static_cast<uint32_t>(mode))) : (SCB->SCR &= (~(1U << static_cast<uint32_t>(mode))));
}

/**
 * @brief Configures the SysTick clock source.
 *
 * @param[in] source The SysTick clock source to be set.
 *
 * This function sets the clock source for the SysTick timer. If the specified
 * source is SYSTICK_SOURCE_HCLK, the SysTick is driven by the processor clock (HCLK).
 * Otherwise, the SysTick is driven by HCLK divided by 8.
 */
void CORTEX::set_systick_source(Systick_Source source) {
    if (source == Systick_Source::SYSTICK_SOURCE_HCLK) {
        SysTick->CTRL |= SysTickSourceHClk;
    } else {
        SysTick->CTRL &= SysTickSourceHClkDiv8;
    }
}

/**
 * @brief Configures the NVIC vector table base and offset
 *
 * @param[in] base The base address of the vector table. If set to
 *                  Vector_Table::NVIC_VECTTAB_RAM, the vector table is
 *                  stored in SRAM. Otherwise, the vector table is stored in
 *                  flash.
 *
 * @param[in] offset The offset of the vector table from the base address.
 *
 * This function sets the vector table base and offset for the NVIC. The
 * offset is masked with VectorTableOffsetMask to ensure that the offset is
 * a multiple of 256 bytes (as required by the NVIC). After setting the
 * vector table, the __DSB() instruction is executed to ensure that any
 * pending data writes are completed before any interrupts are enabled.
 */
void CORTEX::set_nvic_vector_table(Vector_Table base, uint32_t offset) {
    SCB->VTOR = (base == Vector_Table::NVIC_VECTTAB_RAM) ? VectorTableSram : VectorTableFlash | (offset & VectorTableOffsetMask);
    __DSB();
}

/**
 * @brief Configures the NVIC priority group
 *
 * @param[in] group The priority group to configure
 *
 * This function sets the NVIC priority group using the specified parameter.
 * The priority group is set by writing the value of the parameter group to the
 * SCB->AIRCR register, with the VectKeyMask also set to ensure that the write
 * is successful. The priority group determines how the NVIC interprets the
 * priority value for each interrupt. See the datasheet for more information on
 * the priority group options.
 */
void CORTEX::set_nvic_priority_group(Priority_Group group) {
    NVIC_SetPriorityGrouping(static_cast<uint32_t>(group));
}

/**
 * @brief Set the priority of an interrupt request
 *
 * @param[in] irq The interrupt request number to set the priority for
 * @param[in] preemption_priority The preemption priority for the interrupt request
 * @param[in] sub_priority The sub-priority for the interrupt request
 *
 * This function sets the priority of an interrupt request. The priority is
 * computed by calling NVIC_EncodePriority() with the current priority group
 * (obtained from NVIC_GetPriorityGrouping()) and the specified preemption
 * priority and sub-priority. The resulting value is then passed to
 * NVIC_SetPriority() to set the priority of the interrupt request.
 */
void CORTEX::set_nvic_priority(uint8_t irq, uint8_t preemption_priority, uint8_t sub_priority) {
    uint32_t priority_group = NVIC_GetPriorityGrouping();
    NVIC_SetPriority(static_cast<IRQn_Type>(irq), NVIC_EncodePriority(priority_group, preemption_priority, sub_priority));
}

/**
 * @brief Enable an interrupt request
 *
 * @param[in] irq The interrupt request number to enable
 *
 * This function enables the specified interrupt request by setting the
 * corresponding bit in the NVIC->ISER register. It also executes the
 * __DSB() and __ISB() instructions to ensure that any pending data writes
 * are completed before the interrupt is enabled. If the specified interrupt
 * request is invalid (i.e. >= 0), the function returns without action.
 */
void CORTEX::nvic_irq_enable(uint8_t irq) {
    if (static_cast<uint32_t>(irq) >= 0) {
        __asm volatile("":::"memory");
        NVIC->ISER[(static_cast<uint32_t>(irq) >> 5UL)] = (uint32_t)(1UL << (static_cast<uint32_t>(irq) & 0x1FUL));
        __asm volatile("":::"memory");
    }
}

/**
 * @brief Disable an interrupt request
 *
 * @param[in] irq The interrupt request number to disable
 *
 * This function disables the specified interrupt request by setting the
 * corresponding bit in the NVIC->ICER register. It also executes the
 * __DSB() and __ISB() instructions to ensure that any pending data writes
 * are completed before disabling the interrupt, and to flush the processor
 * pipeline after disabling the interrupt.
 */
void CORTEX::nvic_irq_disable(uint8_t irq) {
    if (irq >= 0) {
        NVIC->ICER[(static_cast<uint32_t>(irq) >> 5UL)] = static_cast<uint32_t>(1UL << (static_cast<uint32_t>(irq) & 0x1FUL));
        __DSB();
        __ISB();
    }
}


} // namespace cortex

cortex::CORTEX& CORTEX_I = cortex::CORTEX::get_instance();
