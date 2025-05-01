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

#include "mcu_common.hpp"
#include "CORTEX.hpp"

namespace cortex {

auto CORTEX::get_instance() -> CORTEX& {
    static CORTEX instance;
    return instance;
}

CORTEX::CORTEX() = default;

/**
 * @brief Enables or disables low power mode.
 *
 * This function sets or clears the specified low power mode in the
 * System Control Register (SCR). When set, the processor enters low
 * power mode when the WFE (Wait for Event) instruction is executed.
 *
 * @param mode The low power mode to enable or disable, as defined by the
 *             Low_Power_Mode enum.
 * @param enable Set to true to enable low power mode, false to disable it.
 */
 void CORTEX::set_low_power_mode_enable(Low_Power_Mode mode, bool enable) {
    uint32_t bitmask = 1U << static_cast<uint32_t>(mode);
    if (enable) {
        SCB->SCR |= bitmask;
    } else {
        SCB->SCR &= ~bitmask;
    }
}

/**
 * @brief Sets the clock source for the SysTick timer.
 *
 * This function sets the clock source for the SysTick timer to either the
 * HCLK (HCLK divided by 8) or the reference clock (HCLK divided by 256). The
 * SysTick timer is used to generate interrupts for the delay and wait functions.
 *
 * @param source The clock source to use, as defined by the Systick_Source enum.
 */
void CORTEX::set_systick_source(Systick_Source source) {
    if (source == Systick_Source::SYSTICK_SOURCE_HCLK) {
        SysTick->CTRL |= SysTickSourceHClk;     // Set bit 2 to select HCLK
    } else {
        SysTick->CTRL &= SysTickSourceHClkDiv8; // Clear bit 2 to select HCLK/8
    }
}

/**
 * @brief Sets the NVIC vector table base address and offset.
 *
 * This function sets the NVIC vector table base address and offset in the
 * Vector Table Offset Register (VTOR). The base address is set to either
 * SRAM or flash, and the offset is set to the specified value.
 *
 * @param base The base address for the NVIC vector table, as defined by the
 *             Vector_Table enum.
 * @param offset The offset from the base address, in bytes. The offset must
 *               be a multiple of 256 (0x100).
 */
void CORTEX::set_nvic_vector_table(Vector_Table base, uint32_t offset) {
    const uint32_t masked_offset = offset & VectorTableOffsetMask;
    uint32_t base_addr = (base == Vector_Table::NVIC_VECTTAB_RAM) ? VectorTableSram : VectorTableFlash;
    SCB->VTOR = base_addr | masked_offset;
    __DSB();
}

/**
 * @brief Sets the NVIC priority group.
 *
 * This function sets the NVIC priority group using the
 * NVIC_SetPriorityGrouping() function. The priority group is
 * set to the specified value, which must be one of the values
 * defined in the Priority_Group enum.
 *
 * @param group The priority group to set, as defined by the
 *              Priority_Group enum.
 */
void CORTEX::set_nvic_priority_group(Priority_Group group) {
    NVIC_SetPriorityGrouping(static_cast<uint32_t>(group));
}

/**
 * @brief Sets the NVIC priority for a given IRQ.
 *
 * This function sets the NVIC priority for a given IRQ using the
 * NVIC_SetPriority() function. The priority is set to the encoded
 * value of the preemption priority and sub priority, using the
 * NVIC_EncodePriority() function.
 *
 * @param irq The IRQ for which to set the priority.
 * @param preemption_priority The preemption priority for the IRQ.
 * @param sub_priority The sub priority for the IRQ.
 */
void CORTEX::set_nvic_priority(uint8_t irq, uint8_t preemption_priority, uint8_t sub_priority) {
    uint32_t priority_group = NVIC_GetPriorityGrouping();
    NVIC_SetPriority(static_cast<IRQn_Type>(irq), NVIC_EncodePriority(priority_group, preemption_priority, sub_priority));
}

/**
 * @brief Enables an NVIC interrupt.
 *
 * This function enables a given NVIC interrupt, as specified by the irq
 * parameter. The interrupt is enabled by setting the corresponding bit in
 * the NVIC_ISERx register. The function also includes memory barriers before
 * and after the operation to ensure that the operation is not optimized away
 * by the compiler.
 *
 * @param irq The IRQ to enable, as defined by the IRQn_Type enum.
 */
void CORTEX::nvic_irq_enable(uint8_t irq) {
    __asm volatile("":::"memory");  // Memory barrier before

    auto irq_num = static_cast<uint32_t>(irq);
    uint32_t iser_index = irq_num >> 5U;            // Divide by 32
    uint32_t iser_bit = 1U << (irq_num & 0x1FU);    // Modulo 32

    NVIC->ISER[iser_index] = iser_bit;

    __asm volatile("":::"memory");  // Memory barrier after
}

/**
 * @brief Disables an NVIC interrupt.
 *
 * This function disables a given NVIC interrupt, as specified by the irq
 * parameter. The interrupt is disabled by setting the corresponding bit in
 * the NVIC_ICERx register. The function also includes data and instruction
 * synchronization barriers before and after the operation to ensure that the
 * operation is not optimized away by the compiler.
 *
 * @param irq The IRQ to disable, as defined by the IRQn_Type enum.
 */
void CORTEX::nvic_irq_disable(uint8_t irq) {
    auto irq_num = static_cast<uint32_t>(irq);
    uint32_t icer_index = irq_num >> 5U;            // Divide by 32
    uint32_t icer_bit = 1U << (irq_num & 0x1FU);    // Modulo 32

    NVIC->ICER[icer_index] = icer_bit;
    __DSB();
    __ISB();
}

} // namespace cortex

cortex::CORTEX& CORTEX_I = cortex::CORTEX::get_instance();
