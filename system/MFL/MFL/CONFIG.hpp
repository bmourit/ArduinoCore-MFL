//
// MFL gd32f30x CONFIG definitions
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

#pragma once

#include <stdlib.h>

#include "F303RE.hpp"

/**
 * @brief Encodes register bit definitions.
 *
 * Combines the start bit and width into a single uint32_t value.
 *
 * @param start The starting bit position.
 * @param end The ending bit position.
 * @return uint32_t Encoded bit definition.
 */
inline constexpr uint32_t REG_BIT_DEF(uint32_t start, uint32_t end) {
    return (start << 16U) | (end - start + 1U);
}

// Uncomment if you need the vector table in SRAM
//#define VECTOR_TABLE_SRAM

// Enabling this bit causes extra signal noise which may have implicatiions for things like ADC/DAC precision
// If you need to use this, use the appropriate class methods directly.
// NOTE: This define is now deprecated and CEE will nver be set in startup code.
#define	DISABLE_CEE_ENHANCE

// Set the oppropriate offset here.
// User can change this by defining VECT_TAB_OFFSET.
// This should match the offset expected by the bootloader.
// If no bootloader exists, use 0x00000000
#ifndef VECT_TAB_OFFSET
    inline constexpr uintptr_t VECT_TABLE_OFFSET = 0x00007000U;
#else
    inline constexpr uintptr_t VECT_TABLE_OFFSET = VECT_TAB_OFFSET
#endif

// DO NOT CHANGE THESE
inline constexpr uintptr_t SRAM_BASE = 0x20000000U;
inline constexpr uintptr_t FLASH_BASE = 0x08000000U;

#ifdef VECT_TABLE_SRAM
    inline constexpr uintptr_t VTOR_ADDRESS = SRAM_BASE | VECT_TABLE_OFFSET;
#else
    inline constexpr uintptr_t VTOR_ADDRESS = FLASH_BASE | VECT_TABLE_OFFSET;
#endif
