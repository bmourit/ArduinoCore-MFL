//
// MFL gd32f30x CRC functionality in C++
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

#include "CRC.hpp"
#include "RCU.hpp"

namespace crc {

CRC& CRC::get_instance() {
    static CRC instance;
    return instance;
}

CRC::CRC() : is_clock_enabled_(false) {
    if (!is_clock_enabled_) {
        RCU_I.set_pclk_enable(rcu::RCU_PCLK::PCLK_CRC, true);
        is_clock_enabled_ = true;
    }
}

/**
 * @brief Resets the CRC registers to their default values.
 *
 * This method performs the following actions:
 *  - Writes the default reset value to the DATA register.
 *  - Sets the free data register bits to their reset values.
 *  - Triggers a reset of the CRC calculation logic by setting the RST bit in the CTL register.
 */
void CRC::reset() {
    write_register(*this, CRC_Regs::DATA, DataResetValue);
    write_bit_range(*this, CRC_Regs::FDATA, static_cast<uint32_t>(FDATA_Bits::FDATA), FreeDataResetValue);
    write_bit(*this, CRC_Regs::CTL, static_cast<uint32_t>(CTL_Bits::RST), true);
}

/**
 * @brief Resets the CRC calculation logic by setting the RST bit in the CTL register.
 *
 * This method effectively resets the data register to its default value.
 */
void CRC::reset_data() {
    write_bit(*this, CRC_Regs::CTL, static_cast<uint32_t>(CTL_Bits::RST), true);
}

/**
 * @brief Retrieves the current CRC data register value.
 * 
 * @return The current value of the CRC data register.
 */
uint32_t CRC::get_data() {
    return read_register<uint32_t>(*this, CRC_Regs::DATA);
}

/**
 * @brief Retrieves the current value of the free data register bits.
 *
 * The free data register is a collection of bits that can be used to
 * store data that is not used by the CRC calculation logic. The
 * contents of the free data register are not affected by the CRC
 * calculation process.
 *
 * @return The current value of the free data register bits.
 */
uint8_t CRC::get_free_data(void) {
    return read_bit8_range(*this, CRC_Regs::FDATA, static_cast<uint32_t>(FDATA_Bits::FDATA));
}

/**
 * @brief Sets the free data register bits with the specified value.
 *
 * This method updates the free data register with the provided
 * 8-bit value. The free data register is a collection of bits
 * that can be used to store data not involved in the CRC calculation.
 *
 * @param data The 8-bit value to be set in the free data register.
 */
void CRC::set_free_data(uint8_t data) {
    write_bit_range(*this, CRC_Regs::FDATA, static_cast<uint32_t>(FDATA_Bits::FDATA), static_cast<uint32_t>(data));
}

/**
 * @brief Calculates the CRC of a given 32-bit data value and returns the result.
 *
 * This method takes a 32-bit data value as an argument and calculates the
 * associated CRC value. The CRC calculation is performed by writing the
 * specified data value to the DATA register and then reading the result
 * from the same register after the calculation is complete.
 *
 * @param data The 32-bit data value for which to calculate the CRC.
 *
 * @return The calculated CRC value associated with the provided data value.
 */
uint32_t CRC::calculate_data(uint32_t data) {
    write_register(*this, CRC_Regs::DATA, data);
    return read_register<uint32_t>(*this, CRC_Regs::DATA);
}

/**
 * @brief Calculates the CRC for an array of 32-bit data values and returns the result.
 *
 * This method iterates over an array of 32-bit data values, writing each
 * value to the DATA register to compute the cumulative CRC. After processing
 * all the data, the final CRC value is retrieved from the DATA register.
 *
 * @param data Pointer to an array of 32-bit data values for which to calculate the CRC.
 * @param size The number of elements in the data array.
 *
 * @return The calculated CRC value after processing all the data values.
 */
uint32_t CRC::calculate_mulitple_data(const uint32_t *data, uint32_t size) {
    for (auto i = 0U; i < size; ++i) {
        write_register(*this, CRC_Regs::DATA, data[i]);
    }
    return read_register<uint32_t>(*this, CRC_Regs::DATA);
}


} // namespace crc
