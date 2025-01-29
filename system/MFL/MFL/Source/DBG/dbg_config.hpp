//
// MFL gd32f30x DBG debug in C++
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

namespace armdbg {


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class DBG_Regs : uint32_t {
    ID = 0x00U,
    CTL0 = 0x04U
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class CTL0_Bits : uint8_t {
    SLP_HOLD = 0U,
    DSLP_HOLD = 1U,
    STB_HOLD = 2U,
    TRACE_IOEN = 5U,
    FWDGT_HOLD = 8U,
    WWDGT_HOLD = 9U,
    TIMER0_HOLD = 10U,
    TIMER1_HOLD = 11U,
    TIMER2_HOLD = 12U,
    TIMER3_HOLD = 13U,
    CAN0_HOLD = 14U,
    I2C0_HOLD = 15U,
    I2C1_HOLD = 16U,
    TIMER7_HOLD = 17U,
    TIMER4_HOLD = 18U,
    TIMER5_HOLD = 19U,
    TIMER6_HOLD = 20U
};


///////////////////////////// ALIASES /////////////////////////////

enum class Low_Power_Debug : uint8_t {
    DEBUG_SLEEP = 0U,
    DEBUG_DEEPSLEEP = 1U,
    DEBUG_STANDBY = 2U
};

enum class Debug_Peripheral : uint8_t {
    FWDGT = 8U,
    WWDGT = 9U,
    TIMER0 = 10U,
    TIMER1 = 11U,
    TIMER2 = 12U,
    TIMER3 = 13U,
    CAN0 = 14U,
    I2C0 = 15U,
    I2C1 = 16U,
    TIMER7 = 17U,
    TIMER4 = 18U,
    TIMER5 = 19U,
    TIMER6 = 20U
};


} // namespace armdbg
