//
// MFL gd32f30x FWDGT peripheral register access in C++
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
#include <stdint.h>

#include "CONFIG.hpp"

namespace fwdgt {


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class FWDGT_Regs : uint32_t {
    CTL = 0x00U,
    PSC = 0x04U,
    RLD = 0x08U,
    STAT = 0x0CU
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class CTL_Bits : uint32_t {
    CMD = REG_BIT_DEF(0, 15)
};

enum class PSC_Bits : uint32_t {
    PSC = REG_BIT_DEF(0, 2)
};

enum class RLD_Bits : uint32_t {
    RLD = REG_BIT_DEF(0, 11)
};

enum class STAT_Bits : uint8_t {
    PUD = 0,
    RUD = 1
};


///////////////////////////// ENUMS /////////////////////////////

enum class Prescaler_Value : uint8_t {
    DIV4,
    DIV8,
    DIV16,
    DIV32,
    DIV64,
    DIV128,
    DIV256A,
    DIV256
};

enum class Status_Flags : uint8_t {
    FLAG_PUD = 0,
    FLAG_RUD = 1
};


///////////////////////////// CONSTANTS /////////////////////////////

static inline constexpr uint32_t EnableValue = 0x0000CCCCU;
static inline constexpr uint32_t WriteEnable = 0x00005555U;
static inline constexpr uint32_t ReloadValue = 0x0000AAAAU;


} // namespace fwdgt
