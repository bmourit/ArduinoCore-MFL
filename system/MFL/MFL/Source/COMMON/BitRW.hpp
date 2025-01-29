//
// MFL Template versions of read_bit_range/write_bit_range(s)
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

#include <stdint.h>

/// @brief Constants for bit operations.
inline constexpr uint32_t Clear = 0;
inline constexpr uint32_t Set = 1;

/**
 * @brief Reads a single bit from a register.
 * 
 * @tparam RegType The register type.
 * @tparam Instance The instance type containing the register.
 * @param instance The instance containing the register.
 * @param reg The register identifier.
 * @param bitno The bit number to read.
 * @return bool The value of the bit at the given position.
 */
template <typename RegType, typename Instance>
inline bool read_bit(const Instance& instance, RegType reg, uint32_t bitno) {
    return (*instance.reg_address(reg) & (1U << bitno)) != 0;
}

/**
 * @brief Writes a single bit to a register.
 * 
 * @tparam RegType The register type.
 * @tparam Instance The instance type containing the register.
 * @param instance The instance containing the register.
 * @param reg The register identifier.
 * @param bitno The bit number to write to.
 * @param value The value to write to the bit at the given position.
 */
template <typename RegType, typename Instance>
inline void write_bit(const Instance& instance, RegType reg, uint32_t bitno, bool value) {
    volatile uint32_t* addr = instance.reg_address(reg);
    if (value) {
        *addr |= (1U << bitno);
    } else {
        *addr &= ~(1U << bitno);
    }
}

/**
 * @brief Writes a set of bits to a register.
 * 
 * @tparam RegType The register type.
 * @tparam Instance The instance type containing the register.
 * @param instance The instance containing the register.
 * @param reg The register identifier.
 * @param bits A mask of the bits to write to the register.
 * @param value The value to write to the register bits.
 */
template <typename RegType, typename Instance>
inline void write_bits(const Instance& instance, RegType reg, uint32_t bits, bool value) {
    volatile uint32_t* addr = instance.reg_address(reg);
    if (value) {
        *addr |= bits;
    } else {
        *addr &= ~bits;
    }
}

/**
 * @brief Writes a sequence of bits to a register in the order of the bit numbers provided.
 * 
 * @tparam RegType The register type.
 * @tparam Instance The instance type containing the register.
 * @param instance The instance containing the register.
 * @param reg The register identifier.
 * @param bitno The first bit number to write to.
 * @param value The value to write to the bit at the given position.
 * @param args The remaining bit numbers and values to write to the register.
 * 
 * Example: write_bits_sequence(instance, reg, 1, true, 3, false, 5, true);
 *          writes true to bit 1, false to bit 3, and true to bit 5 in the given register.
 */
template <typename RegType, typename Instance, typename... Args>
inline void write_bits_sequence(const Instance& instance, RegType reg, uint32_t bitno, bool value, Args... args) {
    write_bit(instance, reg, bitno, value);
    if constexpr (sizeof...(args) > 0U) {
        write_bits_sequence(instance, reg, args...);
    }
}

/**
 * @brief Atomically writes a single bit to a register.
 * 
 * @tparam RegType The register type.
 * @tparam Instance The instance type containing the register.
 * @param instance The instance containing the register.
 * @param reg The register identifier.
 * @param bitno The bit number to write to.
 * @param value The value to write to the bit at the given position.
 */
template <typename RegType, typename Instance>
inline void atomic_write_bit(const Instance& instance, RegType reg, uint32_t bitno, bool value) {
    *instance.reg_address(reg) = value ? (1U << bitno) : (0U << bitno);
}

/**
 * @brief Reads a range of bits from a register.
 * 
 * @tparam RegType The register type.
 * @tparam Instance The instance type containing the register.
 * @param instance The instance containing the register.
 * @param reg The register identifier.
 * @param bits The encoded bit definition.
 * @return size_t The value of the specified bit range.
 */
template <typename RegType, typename Instance>
inline uint32_t read_bit_range(const Instance& instance, RegType reg, uint32_t bits) {
    const uint32_t width = bits & 0xFFU;
    const uint32_t bitno = bits >> 16U;
    return (*instance.reg_address(reg) >> bitno) & ((1U << width) - 1);
}

/**
 * @brief Reads a 16-bit range from a register.
 * 
 * @tparam RegType The register type.
 * @tparam Instance The instance type containing the register.
 * @param instance The instance containing the register.
 * @param reg The register identifier.
 * @param bits The encoded bit definition.
 * @return uint16_t The 16-bit value of the specified bit range.
 */
template <typename RegType, typename Instance>
inline uint16_t read_bit16_range(const Instance& instance, RegType reg, uint32_t bits) {
    return static_cast<uint16_t>(read_bit_range<RegType, Instance>(instance, reg, bits) & 0x0000FFFFU);
}

/**
 * @brief Reads an 8-bit range from a register.
 * 
 * @tparam RegType The register type.
 * @tparam Instance The instance type containing the register.
 * @param instance The instance containing the register.
 * @param reg The register identifier.
 * @param bits The encoded bit definition.
 * @return uint8_t The 8-bit value of the specified bit range.
 */
template <typename RegType, typename Instance>
inline uint8_t read_bit8_range(const Instance& instance, RegType reg, uint32_t bits) {
    return static_cast<uint8_t>(read_bit_range<RegType, Instance>(instance, reg, bits) & 0x000000FFU);
}

/**
 * @brief Writes a range of bits to a register.
 * 
 * @tparam RegType The register type.
 * @tparam Instance The instance type containing the register.
 * @param instance The instance containing the register.
 * @param reg The register identifier.
 * @param bits The encoded bit definition.
 * @param value The value to write to the bit range.
 */
template <typename RegType, typename Instance>
inline void write_bit_range(const Instance& instance, RegType reg, uint32_t bits, uint32_t value) {
    const uint32_t width = bits & 0xFFU;
    const uint32_t bitno = bits >> 16U;
    const uint32_t mask = ((1U << width) - 1U) << bitno;
    volatile uint32_t* addr = instance.reg_address(reg);
    *addr = (*addr & ~mask) | ((value << bitno) & mask);
}

/**
 * @brief Atomically writes a range of bits to a register.
 * 
 * @tparam RegType The register type.
 * @tparam Instance The instance type containing the register.
 * @param instance The instance containing the register.
 * @param reg The register identifier.
 * @param bits The encoded bit definition.
 * @param value The value to write to the bit range.
 */
template <typename RegType, typename Instance>
inline void atomic_write_bit_range(const Instance& instance, RegType reg, uint32_t bits, uint32_t value) {
    const uint32_t bitno = bits >> 16U;
    *instance.reg_address(reg) = value << bitno;
}

/**
 * @brief Base case for writing multiple bit ranges: No-op when no arguments are provided.
 * 
 * @tparam RegType The register type.
 * @tparam Instance The instance type containing the register.
 * @param instance The instance containing the register.
 * @param reg The register identifier.
 */
template <typename RegType, typename Instance>
inline void write_bit_ranges(const Instance& instance, RegType reg) {
    // Base case: No-op if no fields are passed
}

/**
 * @brief Recursively writes multiple bit ranges to a register.
 * 
 * @tparam RegType The register type.
 * @tparam Instance The instance type containing the register.
 * @tparam Args Variadic template arguments representing bit-value pairs.
 * @param instance The instance containing the register.
 * @param reg The register identifier.
 * @param bits The encoded bit definition for the first range.
 * @param value The value to write to the first bit range.
 * @param args Additional bit-value pairs.
 */
template <typename RegType, typename Instance, typename... Args>
inline void write_bit_ranges(const Instance& instance, RegType reg, uint32_t bits, uint32_t value, Args... args) {
    write_bit_range(instance, reg, bits, value);
    if constexpr (sizeof...(args) > 0U) {
        write_bit_ranges(instance, reg, args...);
    }
}
