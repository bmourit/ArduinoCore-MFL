//
// MFL gd32f30x RTC peripheral register access in C++
//
// Copyright (C) 2024 B. Mouritsen <bnmguy@gmail.com>. All rights reserved.
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

namespace rtc {


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class RTC_Regs : uint32_t {
    INTEN = 0x00U,
    CTL = 0x04U,
    PSCH = 0x08U,
    PSCL = 0x0CU,
    DIVH = 0x10U,
    DIVL = 0x14U,
    CNTH = 0x18U,
    CNTL = 0x1CU,
    ALRMH = 0x20U,
    ALRML = 0x24U
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class INTEN_Bits : uint8_t {
    SCIE = 0,
    ALRMIE = 1,
    OVIE = 2
};

enum class Interrupt_Type : uint8_t {
    INTR_SCIE = 0,
    INTR_ALRMIE = 1,
    INTR_OVIE = 2
};

enum class CTL_Bits : uint8_t {
    SCIF = 0,
    ALRMIF = 1,
    OVIF = 2,
    RSYNF = 3,
    CMF = 4,
    LWOFF = 5
};

enum class Status_Flags : uint8_t {
    FLAG_SCIF = 0,
    FLAG_ALRMIF = 1,
    FLAG_OVIF = 2,
    FLAG_RSYNF = 3,
    FLAG_LWOFF = 5
};

enum class PSCH_Bits : uint32_t {
    HIGH_PSC = REG_BIT_DEF(0, 3)
};

enum class PSCL_Bits : uint32_t {
    LOW_PSC = REG_BIT_DEF(0, 15)
};

enum class DIVH_Bits : uint32_t {
    HIGH_DIV = REG_BIT_DEF(0, 3)
};

enum class DIVL_Bits : uint32_t {
    LOW_DIV = REG_BIT_DEF(0, 15)
};

enum class CNTH_Bits : uint32_t {
    HIGH_CNT = REG_BIT_DEF(0, 15)
};

enum class CNTL_Bits : uint32_t {
    LOW_CNT = REG_BIT_DEF(0, 15)
};

enum class ALRMH_Bits : uint32_t {
    HIGH_ALRM = REG_BIT_DEF(0, 15)
};

enum class ALRML_Bits : uint32_t {
    LOW_ALRM = REG_BIT_DEF(0, 15)
};

} // namespace rtc
