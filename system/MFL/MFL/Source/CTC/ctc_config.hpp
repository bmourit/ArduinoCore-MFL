//
// MFL gd32f30x CTC peripheral register access in C++
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
#include "rcu_config.hpp"

namespace ctc {


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class CTC_Regs : uint32_t {
    CTL0 = 0x00U,
    CTL1 = 0x04U,
    STAT = 0x08U,
    INTC = 0x0CU
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class CTL0_Bits : uint32_t {
    CKOKIE = 0U,
    CKWARNIE = 1U,
    ERRIE = 2U,
    EREFIE = 3U,
    CNTEN = 5U,
    AUTOTRIM = 6U,
    SWREFPUL = 7U,
    TRIMVALUE = REG_BIT_DEF(8, 13)
};

enum class CTL1_Bits : uint32_t {
    RLVALUE = REG_BIT_DEF(0, 15),
    CKLIM  = REG_BIT_DEF(16, 23),
    REFPSC = REG_BIT_DEF(24, 26),
    REFSEL = REG_BIT_DEF(28, 29),
    REFPOL = 31
};

enum class STAT_Bits : uint32_t {
    CKOKIF = 0U,
    CKWARNIF = 1U,
    ERRIF = 2U,
    EREFIF = 3U,
    CKERR = 8U,
    REFMISS = 9U,
    TRIMERR = 10U,
    REFDIR = 15U,
    REFCAP = REG_BIT_DEF(16, 31)
};

enum class INTC_Bits : uint8_t {
    CKOKIC = 0U,
    CKWARNIC = 1U,
    ERRIC = 2U,
    EREFIC = 3U
};


///////////////////////////// ENUMS /////////////////////////////

enum class Reference_Polarity : uint8_t {
    RISING,
    FALLING
};

enum class Reference_Select : uint8_t {
    GPIO,
    LXTAL
};

enum class Reference_Prescaler : uint8_t {
    OFF,
    DIV2,
    DIV4,
    DIV8,
    DIB16,
    DIV32,
    DIV64,
    DIV128
};

enum class Status_Flags : uint8_t {
    FLAG_CKOK = 0U,
    FLAG_CKWARN = 1U,
    FLAG_ERR = 2U,
    FLAG_EREF = 3U,
    FLAG_CKERR = 8U,
    FLAG_REFMISS = 9U,
    FLAG_TRIMERR = 10U
};

enum class Clear_Flags : uint8_t {
    INTC_CKOKIC = 0U,
    INTC_CKWARNIC = 1U,
    INTC_ERRIC = 2U,
    INTC_EREFIC = 3U
};

enum class Interrupt_Flags : uint32_t {
    INTR_FLAG_CKOK = 0U,
    INTR_FLAG_CKWARN = 1U,
    INTR_FLAG_ERR = 2U,
    INTR_FLAG_EREF = 3U,
    INTR_FLAG_CKERR = 8U,
    INTR_FLAG_REFMISS = 9U,
    INTR_FLAG_TRIMERR = 10U
};

enum class Interrupt_Type : uint8_t {
    INTR_CKOK = 0U,
    INTR_CKWARN = 1U,
    INTR_ERR = 2U,
    INTR_EREF = 3U
};

inline constexpr uint32_t INTR_ERRIC_FLAG_MASK = 0x00000700U;

} // namespace ctc
