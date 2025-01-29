//
// MFL gd32f30x DAC peripheral register access in C++
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

namespace dac {


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class DAC_Regs : uint32_t {
    CTL = 0x00U,
    SWT = 0x04U,
    DAC0_R12DH = 0x08U,
    DAC0_L12DH = 0x0CU,
    DAC0_R8DH  = 0x10U,
    DAC1_R12DH = 0x14U,
    DAC1_L12DH = 0x18U,
    DAC1_R8DH  = 0x1CU,
    DACC_R12DH = 0x20U,
    DACC_L12DH = 0x24U,
    DACC_R8DH  = 0x28U,
    DAC0_DO = 0x2CU,
    DAC1_DO = 0x30U
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class CTL_Bits : uint32_t {
    DEN0 = 0U,
    DBOFF0 = 1U,
    DTEN0 = 2U,
    DTSEL0 = REG_BIT_DEF(3, 5),
    DWM0 = REG_BIT_DEF(6, 7),
    DWBW0 = REG_BIT_DEF(8, 11),
    DDMAEN0 = 12U,
    DEN1 = 16U,
    DBOFF1 = 17U,
    DTEN1 = 18U,
    DTSEL1 = REG_BIT_DEF(19, 21),
    DWM1 = REG_BIT_DEF(22, 23),
    DWBW1 = REG_BIT_DEF(24, 27),
    DDMAEN1 = 28U
};

enum class SWT_Bits : uint8_t {
    SWTR0 = 0U,
    SWTR1 = 1U
};

enum class DAC0_R12DH_Bits : uint32_t {
    DAC0_DH = REG_BIT_DEF(0, 11)
};

enum class DAC0_L12DH_Bits : uint32_t {
    DAC0_DH = REG_BIT_DEF(4, 15)
};

enum class DAC0_R8DH_Bits : uint32_t {
    DAC0_DH = REG_BIT_DEF(0, 7)
};

enum class DAC1_R12DH_Bits : uint32_t {
    DAC1_DH = REG_BIT_DEF(0, 11)
};

enum class DAC1_L12DH_Bits : uint32_t {
    DAC1_DH = REG_BIT_DEF(4, 15)
};

enum class DAC1_R8DH_Bits : uint32_t {
    DAC1_DH = REG_BIT_DEF(0, 7)
};

enum class DACC_R12DH_Bits : uint32_t {
    DAC0_DH = REG_BIT_DEF(0, 11),
    DAC1_DH = REG_BIT_DEF(16, 27)
};

enum class DACC_L12DH_Bits : uint32_t {
    DAC0_DH = REG_BIT_DEF(4, 15),
    DAC1_DH = REG_BIT_DEF(20, 31)
};

enum class DACC_R8DH_Bits : uint32_t {
    DAC0_DH = REG_BIT_DEF(0, 7),
    DAC1_DH = REG_BIT_DEF(8, 15)
};

enum class DAC0_DO_Bits : uint32_t {
    DAC0_DO = REG_BIT_DEF(0, 11)
};

enum class DAC1_DO_Bits : uint32_t {
    DAC1_DO = REG_BIT_DEF(0, 11)
};


///////////////////////////// ENUMS /////////////////////////////

enum class Internal_Device : uint8_t {
    DAC0_BASE,
    DAC1_BASE,
    INVALID
};

enum class Trigger_Source : uint8_t {
    TIMER5_TRIGGER,
    TIMER7_TRIGGER,
    TIMER6_TRIGGER,
    TIMER4_TRIGGER,
    TIMER1_TRIGGER,
    TIMER3_TRIGGER,
    EXTI_9_TRIGGER,
    SOFTWARE_TRIGGER
};

enum class Wave_Type : uint8_t {
    DISABLE,
    LFSR,
    TRIANGLE
};

enum class Bit_Width : uint8_t {
    WIDTH_1,
    WIDTH_2,
    WIDTH_3,
    WIDTH_4,
    WIDTH_5,
    WIDTH_6,
    WIDTH_7,
    WIDTH_8,
    WIDTH_9,
    WIDTH_10,
    WIDTH_11,
    WIDTH_12
};

enum class LFSR_Noise : uint8_t {
    LFSR_BIT0,
    LFSR_BIT1_0,
    LFSR_BIT2_0,
    LFSR_BIT3_0,
    LFSR_BIT4_0,
    LFSR_BIT5_0,
    LFSR_BIT6_0,
    LFSR_BIT7_0,
    LFSR_BIT8_0,
    LFSR_BIT9_0,
    LFSR_BIT10_0,
    LFSR_BIT11_0
};

enum class Data_Align : uint8_t {
    RIGHT_12B,
    LEFT_12B,
    RIGHT_8B
};

enum class Triangle_Amplitude : uint8_t {
    AMP_1,
    AMP_3,
    AMP_7,
    AMP_15,
    AMP_31,
    AMP_63,
    AMP_127,
    AMP_255,
    AMP_511,
    AMP_1023,
    AMP_2047,
    AMP_4095
};

} // namespace dac
