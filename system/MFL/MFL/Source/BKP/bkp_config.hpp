//
// MFL gd32f30x BKP peripheral register access in C++
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

namespace bkp {


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class BKP_Regs : uint32_t {
    DATA0 = 0x04,
    DATA1 = 0x08,
    DATA2 = 0x0C,
    DATA3 = 0x10,
    DATA4 = 0x14,
    DATA5 = 0x18,
    DATA6 = 0x1C,
    DATA7 = 0x20,
    DATA8 = 0x24,
    DATA9 = 0x28,
    OCTL = 0x2C,
    TPCTL = 0x30,
    TPCS = 0x34,
    DATA10 = 0x40,
    DATA11 = 0x44,
    DATA12 = 0x48,
    DATA13 = 0x4C,
    DATA14 = 0x50,
    DATA15 = 0x54,
    DATA16 = 0x58,
    DATA17 = 0x5C,
    DATA18 = 0x60,
    DATA19 = 0x64,
    DATA20 = 0x68,
    DATA21 = 0x6C,
    DATA22 = 0x70,
    DATA23 = 0x74,
    DATA24 = 0x78,
    DATA25 = 0x7C,
    DATA26 = 0x80,
    DATA27 = 0x84,
    DATA28 = 0x88,
    DATA29 = 0x8C,
    DATA30 = 0x90,
    DATA31 = 0x94,
    DATA32 = 0x98,
    DATA33 = 0x9C,
    DATA34 = 0xA0,
    DATA35 = 0xA4,
    DATA36 = 0xA8,
    DATA37 = 0xAC,
    DATA38 = 0xB0,
    DATA39 = 0xB4,
    DATA40 = 0xB8,
    DATA41 = 0xBC
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class DATA_Bits : uint32_t {
    DATAX = REG_BIT_DEF(0, 15)
};

enum class OCTL_Bits : uint32_t {
    RCCV = REG_BIT_DEF(0, 6),
    COEN = 7U,
    ASOEN = 8U,
    ROSEL = 9U,
    CCOSEL = 14U,
    CALDIR = 15U
};

enum class TPCTL_Bits : uint8_t {
    TPEN = 0U,
    TPAL = 1U
};

enum class TPCS_Bits : uint8_t {
    TER = 0U,
    TIR = 1U,
    TPIE = 2U,
    TEF = 8U,
    TIF = 9U
};


///////////////////////////// ENUMS /////////////////////////////

enum class Output_Pulse : uint8_t {
    ALARM_PULSE,
    SECOND_PULSE
};

enum class Clock_Divider : uint8_t {
    DIV_64,
    DIV_1
};

enum class Calibration_Type : uint8_t {
    SLOW_DOWN,
    SPEED_UP
};

enum class Tamper_Level : uint8_t {
    ACTIVE_HIGH,
    ACTIVE_LOW
};

enum class Status_Flags : uint8_t {
    TAMPER_FLAG = 8U
};

enum class Clear_Flags : uint8_t {
    TAMPER_FLAG_CLEAR = 0U,
    TAMPER_INTR_FLAG_CLEAR = 1U
};

enum class Interrupt_Flags : uint8_t {
    TAMPER_INTR_FLAG = 9
};

enum class Backup_Data : uint8_t {
    DATA_0 = 1,
    DATA_1,
    DATA_2,
    DATA_3,
    DATA_4,
    DATA_5,
    DATA_6,
    DATA_7,
    DATA_8,
    DATA_9,
    DATA_10,
    DATA_11,
    DATA_12,
    DATA_13,
    DATA_14,
    DATA_15,
    DATA_16,
    DATA_17,
    DATA_18,
    DATA_19,
    DATA_20,
    DATA_21,
    DATA_22,
    DATA_23,
    DATA_24,
    DATA_25,
    DATA_26,
    DATA_27,
    DATA_28,
    DATA_29,
    DATA_30,
    DATA_31,
    DATA_32,
    DATA_33,
    DATA_34,
    DATA_35,
    DATA_36,
    DATA_37,
    DATA_38,
    DATA_39,
    DATA_40,
    DATA_41
};


} // namespace backup
