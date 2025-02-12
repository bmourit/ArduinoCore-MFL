//
// MFL Template versions of read_register/write_register
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

#include "BitRW.hpp"

/**
 * @brief Reads the value of a register.
 *
 * @tparam T The data type of the register value.
 * @tparam RegType The type of the register identifier.
 * @tparam Instance The type of the instance containing the register.
 * @param instance The instance containing the register.
 * @param reg The register identifier.
 * @return T The value read from the register.
 */
template<typename T, typename RegType, typename Instance>
inline T read_register(const Instance& instance, RegType reg) {
    return *reinterpret_cast<volatile T*>(instance.reg_address(reg));
}

/**
 * @brief Writes a value to a register.
 *
 * @tparam T The data type of the value to write.
 * @tparam RegType The type of the register identifier.
 * @tparam Instance The type of the instance containing the register.
 * @param instance The instance containing the register.
 * @param reg The register identifier.
 * @param value The value to write to the register.
 */
template<typename T, typename RegType, typename Instance>
inline void write_register(const Instance& instance, RegType reg, T value) {
    *reinterpret_cast<volatile T*>(instance.reg_address(reg)) = value;
}
