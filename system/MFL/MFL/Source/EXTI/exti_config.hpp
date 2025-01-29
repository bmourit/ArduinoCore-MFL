//
// MFL EXTI peripheral register access in C++
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

namespace exti {


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class EXTI_Regs : uint32_t {
    INTEN = 0x00U,
    EVEN = 0x04U,
    RTEN = 0x08U,
    FTEN = 0x0CU,
    SWIEV = 0x10U,
    PD = 0x14U
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class EXTI_Line : uint8_t {
    EXTI0 = 0,
    EXTI1 = 1,
    EXTI2 = 2,
    EXTI3 = 3,
    EXTI4 = 4,
    EXTI5 = 5,
    EXTI6 = 6,
    EXTI7 = 7,
    EXTI8 = 8,
    EXTI9 = 9,
    EXTI10 = 10,
    EXTI11 = 11,
    EXTI12 = 12,
    EXTI13 = 13,
    EXTI14 = 14,
    EXTI15 = 15,
    EXTI16 = 16,
    EXTI17 = 17,
    EXTI18 = 18,
    EXTI19 = 19,
    INVALID = 20
};

enum class Status_Flags : uint8_t {
    FLAG_EXTI0 = 0,
    FLAG_EXTI1 = 1,
    FLAG_EXTI2 = 2,
    FLAG_EXTI3 = 3,
    FLAG_EXTI4 = 4,
    FLAG_EXTI5 = 5,
    FLAG_EXTI6 = 6,
    FLAG_EXTI7 = 7,
    FLAG_EXTI8 = 8,
    FLAG_EXTI9 = 9,
    FLAG_EXTI10 = 10,
    FLAG_EXTI11 = 11,
    FLAG_EXTI12 = 12,
    FLAG_EXTI13 = 13,
    FLAG_EXTI14 = 14,
    FLAG_EXTI15 = 15,
    FLAG_EXTI16 = 16,
    FLAG_EXTI17 = 17,
    FLAG_EXTI18 = 18,
    FLAG_EXTI19 = 19,
    INVALID = 20
};

enum class Interrupt_Flags : uint8_t {
    INTR_FLAG_EXTI0 = 0,
    INTR_FLAG_EXTI1 = 1,
    INTR_FLAG_EXTI2 = 2,
    INTR_FLAG_EXTI3 = 3,
    INTR_FLAG_EXTI4 = 4,
    INTR_FLAG_EXTI5 = 5,
    INTR_FLAG_EXTI6 = 6,
    INTR_FLAG_EXTI7 = 7,
    INTR_FLAG_EXTI8 = 8,
    INTR_FLAG_EXTI9 = 9,
    INTR_FLAG_EXTI10 = 10,
    INTR_FLAG_EXTI11 = 11,
    INTR_FLAG_EXTI12 = 12,
    INTR_FLAG_EXTI13 = 13,
    INTR_FLAG_EXTI14 = 14,
    INTR_FLAG_EXTI15 = 15,
    INTR_FLAG_EXTI16 = 16,
    INTR_FLAG_EXTI17 = 17,
    INTR_FLAG_EXTI18 = 18,
    INTR_FLAG_EXTI19 = 19,
    INVALID = 20
};

enum class Interrupt_Type : uint8_t {
    INTR_EXTI0 = 0,
    INTR_EXTI1 = 1,
    INTR_EXTI2 = 2,
    INTR_EXTI3 = 3,
    INTR_EXTI4 = 4,
    INTR_EXTI5 = 5,
    INTR_EXTI6 = 6,
    INTR_EXTI7 = 7,
    INTR_EXTI8 = 8,
    INTR_EXTI9 = 9,
    INTR_EXTI10 = 10,
    INTR_EXTI11 = 11,
    INTR_EXTI12 = 12,
    INTR_EXTI13 = 13,
    INTR_EXTI14 = 14,
    INTR_EXTI15 = 15,
    INTR_EXTI16 = 16,
    INTR_EXTI17 = 17,
    INTR_EXTI18 = 18,
    INTR_EXTI19 = 19,
    INVALID = 20
};


///////////////////////////// ENUMS /////////////////////////////

enum class EXTI_Mode : uint8_t {
    EXTI_INTERRUPT,
    EXTI_EVENT
};

enum class EXTI_Trigger : uint8_t {
    TRIG_RISING = 0,
    TRIG_FALLING,
    TRIG_BOTH,
    TRIG_NONE
};


///////////////////////////// UNUSED DOCUMENTATION ONLY /////////////////////////////

/*
enum class INTEN_Bits : uint32_t {
    INTEN0 = REG_BIT_DEF(0, 0),
    INTEN1 = REG_BIT_DEF(1, 1),
    INTEN2 = REG_BIT_DEF(2, 2),
    INTEN3 = REG_BIT_DEF(3, 3),
    INTEN4 = REG_BIT_DEF(4, 4),
    INTEN5 = REG_BIT_DEF(5, 5),
    INTEN6 = REG_BIT_DEF(6, 6),
    INTEN7 = REG_BIT_DEF(7, 7),
    INTEN8 = REG_BIT_DEF(8, 8),
    INTEN9 = REG_BIT_DEF(9, 9),
    INTEN10 = REG_BIT_DEF(10, 10),
    INTEN11 = REG_BIT_DEF(11, 11),
    INTEN12 = REG_BIT_DEF(12, 12),
    INTEN13 = REG_BIT_DEF(13, 13),
    INTEN14 = REG_BIT_DEF(14, 14),
    INTEN15 = REG_BIT_DEF(15, 15),
    INTEN16 = REG_BIT_DEF(16, 16),
    INTEN17 = REG_BIT_DEF(17, 17),
    INTEN18 = REG_BIT_DEF(18, 18),
    INTEN19 = REG_BIT_DEF(19, 19)
};

enum class EVEN_Bits : uint32_t {
    EVEN0 = REG_BIT_DEF(0, 0),
    EVEN1 = REG_BIT_DEF(1, 1),
    EVEN2 = REG_BIT_DEF(2, 2),
    EVEN3 = REG_BIT_DEF(3, 3),
    EVEN4 = REG_BIT_DEF(4, 4),
    EVEN5 = REG_BIT_DEF(5, 5),
    EVEN6 = REG_BIT_DEF(6, 6),
    EVEN7 = REG_BIT_DEF(7, 7),
    EVEN8 = REG_BIT_DEF(8, 8),
    EVEN9 = REG_BIT_DEF(9, 9),
    EVEN10 = REG_BIT_DEF(10, 10),
    EVEN11 = REG_BIT_DEF(11, 11),
    EVEN12 = REG_BIT_DEF(12, 12),
    EVEN13 = REG_BIT_DEF(13, 13),
    EVEN14 = REG_BIT_DEF(14, 14),
    EVEN15 = REG_BIT_DEF(15, 15),
    EVEN16 = REG_BIT_DEF(16, 16),
    EVEN17 = REG_BIT_DEF(17, 17),
    EVEN18 = REG_BIT_DEF(18, 18),
    EVEN19 = REG_BIT_DEF(19, 19)
};

enum class RTEN_Bits : uint32_t {
    RTEN0 = REG_BIT_DEF(0, 0),
    RTEN1 = REG_BIT_DEF(1, 1),
    RTEN2 = REG_BIT_DEF(2, 2),
    RTEN3 = REG_BIT_DEF(3, 3),
    RTEN4 = REG_BIT_DEF(4, 4),
    RTEN5 = REG_BIT_DEF(5, 5),
    RTEN6 = REG_BIT_DEF(6, 6),
    RTEN7 = REG_BIT_DEF(7, 7),
    RTEN8 = REG_BIT_DEF(8, 8),
    RTEN9 = REG_BIT_DEF(9, 9),
    RTEN10 = REG_BIT_DEF(10, 10),
    RTEN11 = REG_BIT_DEF(11, 11),
    RTEN12 = REG_BIT_DEF(12, 12),
    RTEN13 = REG_BIT_DEF(13, 13),
    RTEN14 = REG_BIT_DEF(14, 14),
    RTEN15 = REG_BIT_DEF(15, 15),
    RTEN16 = REG_BIT_DEF(16, 16),
    RTEN17 = REG_BIT_DEF(17, 17),
    RTEN18 = REG_BIT_DEF(18, 18),
    RTEN19 = REG_BIT_DEF(19, 19)
};

enum class FTEN_Bits : uint32_t {
    FTEN0 = REG_BIT_DEF(0, 0),
    FTEN1 = REG_BIT_DEF(1, 1),
    FTEN2 = REG_BIT_DEF(2, 2),
    FTEN3 = REG_BIT_DEF(3, 3),
    FTEN4 = REG_BIT_DEF(4, 4),
    FTEN5 = REG_BIT_DEF(5, 5),
    FTEN6 = REG_BIT_DEF(6, 6),
    FTEN7 = REG_BIT_DEF(7, 7),
    FTEN8 = REG_BIT_DEF(8, 8),
    FTEN9 = REG_BIT_DEF(9, 9),
    FTEN10 = REG_BIT_DEF(10, 10),
    FTEN11 = REG_BIT_DEF(11, 11),
    FTEN12 = REG_BIT_DEF(12, 12),
    FTEN13 = REG_BIT_DEF(13, 13),
    FTEN14 = REG_BIT_DEF(14, 14),
    FTEN15 = REG_BIT_DEF(15, 15),
    FTEN16 = REG_BIT_DEF(16, 16),
    FTEN17 = REG_BIT_DEF(17, 17),
    FTEN18 = REG_BIT_DEF(18, 18),
    FTEN19 = REG_BIT_DEF(19, 19)
};

enum class SWIEV_Bits : uint32_t {
    SWIEV0 = REG_BIT_DEF(0, 0),
    SWIEV1 = REG_BIT_DEF(1, 1),
    SWIEV2 = REG_BIT_DEF(2, 2),
    SWIEV3 = REG_BIT_DEF(3, 3),
    SWIEV4 = REG_BIT_DEF(4, 4),
    SWIEV5 = REG_BIT_DEF(5, 5),
    SWIEV6 = REG_BIT_DEF(6, 6),
    SWIEV7 = REG_BIT_DEF(7, 7),
    SWIEV8 = REG_BIT_DEF(8, 8),
    SWIEV9 = REG_BIT_DEF(9, 9),
    SWIEV10 = REG_BIT_DEF(10, 10),
    SWIEV11 = REG_BIT_DEF(11, 11),
    SWIEV12 = REG_BIT_DEF(12, 12),
    SWIEV13 = REG_BIT_DEF(13, 13),
    SWIEV14 = REG_BIT_DEF(14, 14),
    SWIEV15 = REG_BIT_DEF(15, 15),
    SWIEV16 = REG_BIT_DEF(16, 16),
    SWIEV17 = REG_BIT_DEF(17, 17),
    SWIEV18 = REG_BIT_DEF(18, 18),
    SWIEV19 = REG_BIT_DEF(19, 19)
};

enum class PD_Bits : uint32_t {
    PD0 = REG_BIT_DEF(0, 0),
    PD1 = REG_BIT_DEF(1, 1),
    PD2 = REG_BIT_DEF(2, 2),
    PD3 = REG_BIT_DEF(3, 3),
    PD4 = REG_BIT_DEF(4, 4),
    PD5 = REG_BIT_DEF(5, 5),
    PD6 = REG_BIT_DEF(6, 6),
    PD7 = REG_BIT_DEF(7, 7),
    PD8 = REG_BIT_DEF(8, 8),
    PD9 = REG_BIT_DEF(9, 9),
    PD10 = REG_BIT_DEF(10, 10),
    PD11 = REG_BIT_DEF(11, 11),
    PD12 = REG_BIT_DEF(12, 12),
    PD13 = REG_BIT_DEF(13, 13),
    PD14 = REG_BIT_DEF(14, 14),
    PD15 = REG_BIT_DEF(15, 15),
    PD16 = REG_BIT_DEF(16, 16),
    PD17 = REG_BIT_DEF(17, 17),
    PD18 = REG_BIT_DEF(18, 18),
    PD19 = REG_BIT_DEF(19, 19)
};
*/


} // namespace exti
