//
// MFL gd32f30x PMU peripheral register access in C++
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

namespace pmu {


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class PMU_Regs : uint32_t {
    CTL = 0x00U,
    CS = 0x04U
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class CTL_Bits : uint32_t {
    LDOLP = 0,
    STBMOD = 1,
    WURST = 2,
    STBRST = 3,
    LVDEN = 4,
    LVDT = REG_BIT_DEF(5, 7),
    BKPWEN = 8,
    LDLP = 10,
    LDNP = 11,
    LDOVS = REG_BIT_DEF(14, 15),
    HDEN = 16,
    HDS = 17,
    LDEN = REG_BIT_DEF(18, 19)
};

enum class CS_Bits : uint32_t {
    WUF = 0,
    STBF = 1,
    LVDF = 2,
    WUPEN = 8,
    LDOVSRF = 14,
    HDRF = 16,
    HDSRF = 17,
    LDRF = REG_BIT_DEF(18, 19)
};


///////////////////////////// ENUMS /////////////////////////////

enum class LVD_Threshold : uint8_t {
    LVDT_2_1,   // 2.1V
    LVDT_2_3,   // 2.3V
    LVDT_2_4,   // 2.4V
    LVDT_2_6,   // 2.6V
    LVDT_2_7,   // 2.7V
    LVDT_2_8,   // 2.8V
    LVDT_3_0,   // 3.0V
    LVDT_3_1    // 3.1V
};

enum class Power_Driver : uint8_t {
    NORMAL_DRIVER,
    LOW_DRIVER
};

enum class Output_Voltage : uint8_t {
    LDO_VOLTAGE_LOW = 1,
    LDO_VOLTAGE_MID = 2,
    LDO_VOLTAGE_HIGH = 3
};

enum class Low_Driver : uint8_t {
    LOW_DRIVER_DEEP_SLEEP_DISABLE = 0,
    LOW_DRIVER_DEEP_SLEEP_ENABLE = 3
};

// Alias for CS_Bits
enum class Status_Flags : uint32_t {
    WAKEUP_FLAG = 0,
    STANDBY_FLAG = 1,
    LVD_FLAG = 2,
    FDOVSR_FLAG = 14,
    HDR_FLAG = 16,
    HDSR_FLAG = 17,
    LDR_FLAG = REG_BIT_DEF(18, 19)
};

enum class Clear_Flags : uint8_t {
    RESET_WAKEUP_FLAG,
    RESET_STANDBY_FLAG
};

enum class PMU_Commands : uint8_t {
    WFI_CMD,
    WFE_CMD
};

} // namespace pmu
