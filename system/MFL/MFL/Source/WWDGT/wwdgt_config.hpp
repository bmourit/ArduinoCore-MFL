//
// MFL gd32f30x WWDGT peripheral register access in C++
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

namespace wwdgt {


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class WWDGT_Regs : uint32_t {
    CTL = 0x00U,
    CFG = 0x04U,
    STAT = 0x08U
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class CTL_Bits : uint32_t {
    CNT = REG_BIT_DEF(0, 6),
    WDGTEN = 7
};

enum class CFG_Bits : uint32_t {
    WIN = REG_BIT_DEF(0, 6),
    PSC = REG_BIT_DEF(7, 8),
    EWIE = 9
};

enum class STAT_Bits : uint32_t {
    EWIF = 0
};


///////////////////////////// ENUMS /////////////////////////////

enum class Prescaler_Values : uint8_t {
    PSC_DIV1,
    PSC_DIV2,
    PSC_DIV4,
    PSC_DIV8
};

} // namespace wwdgt
