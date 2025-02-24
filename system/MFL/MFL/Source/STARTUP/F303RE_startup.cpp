//
// MFL gd32f30x STARTUP peripheral register access in C++
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

#include <stdint.h>

#include "CONFIG.hpp"
#include "RCU.hpp"

extern int main();

extern "C" {
    extern uint32_t _sbss;
    extern uint32_t _ebss;
    extern uint32_t __bss_start__;
    extern uint32_t __bss_end__;
    extern uint32_t _etext;
    extern uint32_t end;
    extern uint32_t _end;
    extern uint32_t _sidata;
    extern uint32_t _sdata;
    extern uint32_t _edata;
}

// Minimal RCU register structure for startup
struct RCUMin {
    volatile uint32_t CTL;
    volatile uint32_t CFG0;
    volatile uint32_t INTR;
    volatile uint32_t APB2RST;
    volatile uint32_t APB1RST;
    volatile uint32_t AHBEN;
    volatile uint32_t APB2EN;
    volatile uint32_t APB1EN;
    volatile uint32_t BDCTL;
    volatile uint32_t RSTSCK;
    volatile uint32_t CFG1;
};

static RCUMin* const regs = reinterpret_cast<RCUMin*>(0x40021000U);

/**
 * @brief Perform minimal system startup operations.
 *
 * This function performs the following tasks:
 * 1. Enables FPU access if present and used.
 * 2. Enables clock on HXTAL.
 * 3. Set the HXTAL clock as the system clock.
 * 4. Enables interrupts.
 * 5. Disables all peripheral clocks.
 * 6. Sets the vector table offset to 0x08007000.
 */
extern "C" void system_startup() {
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    // Set CP10 and CP11 full access
    SCB->CPACR |= ((3U << 10 * 2) | (3U << 11 * 2));
#endif
    regs->CTL |= 0x00000001U;
    regs->CFG0 &= 0xFFFFFFFCU;
    regs->CTL &= 0xFEF6FFFFU;
    regs->INTR = 0x009F0000U;
    regs->CTL &= 0xFFFBFFFFU;
    regs->CFG0 = 0x00000000U;
    regs->CFG1 = 0x00000000U;
    // Vector table offset
    SCB->VTOR = VTOR_ADDRESS;
}

/**
 * @brief Default reset handler.
 *
 * This function is called by the startup code and performs the following tasks:
 * 1. Copies the initialized data section into RAM.
 * 2. Clears the BSS section.
 * 3. Calls the `system_startup` function to perform minimal system startup operations.
 * 4. Initializes static objects by calling the constructors.
 * 5. Calls the `main` function.
 * 6. Calls the destructors of static objects.
 * 7. Enters an infinite loop.
 */
extern "C" void Reset_Handler() {
    // Initialize data section
    uint32_t *src = &_sidata;
    uint32_t *dest = &_sdata;
    while (dest < &_edata) *dest++ = *src++;

    // Clear BSS
    for (uint32_t *bss = &_sbss; bss < &_ebss; ++bss) *bss = 0;

    // Call system startup function
    system_startup();

    // Initialize static objects
    extern void (*__preinit_array_start[])(void), (*__preinit_array_end[])(void);
    extern void (*__init_array_start[])(void), (*__init_array_end[])(void);
    extern void (*__fini_array_start[])(void), (*__fini_array_end[])(void);

    // Call constructors
    for (void (**p)() = __preinit_array_start; p < __preinit_array_end; ++p) (*p)();
    for (void (**p)() = __init_array_start; p < __init_array_end; ++p) (*p)();

    // Jump to main
    main();

    // Call destructors
    for (void (**p)() = __fini_array_start; p < __fini_array_end; ++p) (*p)();

    while (1);
}
