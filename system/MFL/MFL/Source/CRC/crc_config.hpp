//
// MFL gd32f30x CRC functionality in C++
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

namespace crc {


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class CRC_Regs : uint32_t {
    DATA = 0x00U,
    FDATA = 0x04U,
    CTL = 0x08U
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class FDATA_Bits : uint32_t {
    FDATA = REG_BIT_DEF(0, 7)
};

enum class CTL_Bits : uint8_t {
    RST = 0
};


///////////////////////////// CONSTANTS /////////////////////////////

inline constexpr uint32_t DataResetValue = 0xFFFFFFFFU;
inline constexpr uint8_t FreeDataResetValue = 0U;


} // namespace crc
