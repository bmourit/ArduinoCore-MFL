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

#include "BKP.hpp"
#include "RCU.hpp"

namespace bkp {

BKP& BKP::get_instance() {
    static BKP instance;
    return instance;
}

BKP::BKP() : is_clock_enabled_(false) {
    if (!is_clock_enabled_) {
        RCU_I.set_pclk_enable(rcu::RCU_PCLK::PCLK_PMU, true);
        RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_BKPIRST, true);
        RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_BKPIRST, false);
        is_clock_enabled_ = true;
    }
}

/**
 * @brief Resets the BKP peripheral by toggling its peripheral clock reset.
 *
 * This function enables the peripheral clock reset for the BKP peripheral by
 * setting the reset register, then disables the reset to complete the reset
 * operation. This effectively resets all registers of the BKP peripheral to
 * their default values.
 */
void BKP::reset() {
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_BKPIRST, true);
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_BKPIRST, false);
}

/**
 * @brief Sets the value of the specified backup data register.
 *
 * This function sets the current 16-bit value of a specified backup data
 * register within the BKP peripheral. The register is selected based on the
 * provided Backup_Data enumeration value. The data is only set if the
 * specified register index is within the valid range (DATA_0 to DATA_41).
 *
 * @param data_offset The backup data register index to set, specified as a
 *              Backup_Data enumeration value.
 * @param data The new 16-bit value to set the selected backup data register.
 */
void BKP::set_data(Backup_Data data_offset, uint16_t data) {
    uint32_t offset = static_cast<uint32_t>(data_offset);
    uint32_t reg_offset;

    // Use conditional operator to avoid branching
    reg_offset = (offset <= static_cast<uint32_t>(Backup_Data::DATA_9))
        ? (0x04U + (offset - 1U) * 0x04U)
        : (0x40U + (offset - 10U) * 0x04U);

    // Early bounds check using single comparison
    if (offset <= static_cast<uint32_t>(Backup_Data::DATA_41)) {
        *reinterpret_cast<volatile uint16_t*>(BKP_baseAddress + reg_offset) = data;
    }
}

/**
 * @brief Retrieves the value of the specified backup data register.
 *
 * This function returns the current 16-bit value of a specified backup data
 * register within the BKP peripheral. The register is selected based on the
 * provided Backup_Data enumeration value. The data is returned only if the
 * specified register index is within the valid range (DATA_0 to DATA_41).
 *
 * @param data_offset The backup data register index to get, specified as a
 *              Backup_Data enumeration value.
 * @return The current 16-bit value of the selected backup data register, or
 *         0 if the specified register index is not within the valid range.
 */
uint16_t BKP::get_data(Backup_Data data_offset) {
    uint32_t offset = static_cast<uint32_t>(data_offset);
    uint32_t reg_offset;
    
    // Use conditional operator for efficient register offset calculation
    reg_offset = (offset <= static_cast<uint32_t>(Backup_Data::DATA_9))
        ? (0x04U + (offset - 1U) * 0x04U)
        : (0x40U + (offset - 10U) * 0x04U);
        
    // Single bounds check for valid range
    if (offset <= static_cast<uint32_t>(Backup_Data::DATA_41)) {
        return *reinterpret_cast<volatile uint16_t*>(BKP_baseAddress + reg_offset);
    }
    
    return 0U;
}

/**
 * @brief Enables or disables the output calibration of the RTC.
 *
 * If the output calibration is enabled, the RTC output signal is
 * calibrated to ensure that the output signal is correct. The
 * calibration is done by adjusting the clock signal of the RTC
 * to match the crystal oscillator frequency. If the output
 * calibration is disabled, the RTC output signal is not
 * calibrated and may be incorrect.
 *
 * @param enable Set to true to enable the output calibration, or
 *               false to disable it.
 */
void BKP::set_rtc_output_calibration_enable(bool enable) {
    write_bit(*this, BKP_Regs::OCTL, static_cast<uint32_t>(OCTL_Bits::COEN), enable);
}

/**
 * @brief Enables or disables the RTC output signal.
 *
 * If the output signal is enabled, the RTC generates a signal on the
 * selected output pin. If the output signal is disabled, the RTC
 * does not generate a signal on the output pin.
 *
 * @param enable Set to true to enable the output signal, or false
 *               to disable it.
 */
void BKP::set_rtc_output_signal_enable(bool enable) {
    write_bit(*this, BKP_Regs::OCTL, static_cast<uint32_t>(OCTL_Bits::ASOEN), enable);
}

/**
 * @brief Selects the RTC output pulse type.
 *
 * This function selects either a first pulse or second pulse type for the
 * RTC output signal. The pulse type is selected by setting the ROSEL bit
 * in the OCTL register to either 0 or 1. If ROSEL is 0, the first pulse is
 * selected, and if ROSEL is 1, the second pulse is selected.
 *
 * @param pulse The output pulse type to select, either FIRST_PULSE or
 *              SECOND_PULSE.
 */
void BKP::set_rtc_output_pulse(Output_Pulse pulse) {
    write_bit(*this, BKP_Regs::OCTL, static_cast<uint32_t>(OCTL_Bits::ROSEL),
           (pulse == Output_Pulse::SECOND_PULSE) ? true : false);
}

/**
 * @brief Sets the clock divider for the RTC output signal.
 *
 * This function sets the clock divider for the RTC output signal to either
 * 1 or 2. The divider is set by writing the CCOSEL bit in the OCTL register
 * to either 0 or 1. If the CCOSEL bit is 0, the divider is set to 2; if the
 * CCOSEL bit is 1, the divider is set to 1.
 *
 * @param divider The clock divider to set, specified as a
 *                Clock_Divider enumeration value.
 */
void BKP::set_rtc_clock_divider(Clock_Divider divider) {
    write_bit(*this, BKP_Regs::OCTL, static_cast<uint32_t>(OCTL_Bits::CCOSEL),
           (divider == Clock_Divider::DIV_1) ? true : false);
}

/**
 * @brief Sets the type of the RTC clock calibration.
 *
 * This function sets the type of the RTC clock calibration to either
 * slow down or speed up the clock. The calibration type is set by
 * setting the CALDIR bit in the OCTL register.
 *
 * @param type The type of the RTC clock calibration, specified as a
 *             Calibration_Type enumeration value.
 */
void BKP::set_rtc_clock_calibration_type(Calibration_Type type) {
    write_bit(*this, BKP_Regs::OCTL, static_cast<uint32_t>(OCTL_Bits::CALDIR),
           (type == Calibration_Type::SPEED_UP) ? true : false);
}

/**
 * @brief Sets the RTC clock calibration value.
 *
 * This function sets the value of the RTC clock calibration register.
 * The calibration value is a 7-bit unsigned integer from 0 to 127.
 * The calibration value is used to adjust the RTC clock frequency to
 * compensate for frequency deviations due to process variations or
 * temperature changes.
 *
 * @param value The calibration value to set, specified as a 7-bit unsigned
 *              integer from 0 to 127.
 */
void BKP::set_rtc_calibration_value(uint8_t value) {
    write_bit_range(*this, BKP_Regs::OCTL, static_cast<uint32_t>(OCTL_Bits::RCCV), static_cast<uint32_t>(value));
}

/**
 * @brief Enables or disables the tamper detection feature of the BKP peripheral.
 *
 * This function enables or disables the tamper detection feature of the BKP
 * peripheral. When enabled, the BKP peripheral will detect changes in the
 * voltage level of the tamper detection pin and generate a tamper detection
 * event.
 *
 * @param enable Set to true to enable the tamper detection feature or false to
 *               disable it.
 */
void BKP::set_tamper_detection_enable(bool enable) {
    write_bit(*this, BKP_Regs::TPCTL, static_cast<uint32_t>(TPCTL_Bits::TPEN), enable);
}

/**
 * @brief Sets the level of the tamper detection pin.
 *
 * This function sets the level of the tamper detection pin to either active
 * high or active low. When set to active high, the BKP peripheral will detect
 * changes in the voltage level of the tamper detection pin and generate a
 * tamper detection event when the pin is high. When set to active low, the BKP
 * peripheral will detect changes in the voltage level of the tamper detection
 * pin and generate a tamper detection event when the pin is low.
 *
 * @param level The level of the tamper detection pin, specified as a
 *              Tamper_Level enumeration value.
 */
void BKP::set_tamper_level(Tamper_Level level) {
    write_bit(*this, BKP_Regs::TPCTL, static_cast<uint32_t>(TPCTL_Bits::TPAL),
           level == Tamper_Level::ACTIVE_LOW ? true : false);
}

/**
 * @brief Enables or disables the tamper interrupt.
 *
 * This function sets the tamper interrupt enable bit in the TPCS register.
 * When the tamper interrupt is enabled, the BKP peripheral will generate
 * an interrupt in response to a tamper detection event.
 *
 * @param enable Set to true to enable the tamper interrupt or false to
 *               disable it.
 */
void BKP::set_tamper_interrupt_enable(bool enable) {
    write_bit(*this, BKP_Regs::TPCS, static_cast<uint32_t>(TPCS_Bits::TPIE), enable);
}

/**
 * @brief Retrieves the status of the specified flag.
 *
 * This function reads the specified flag from the BKP status register (TPCS)
 * and returns its status. If the flag is not applicable to the BKP
 * peripheral, this function returns false.
 *
 * @param flag The status flag to retrieve. Must be a value from the
 *             Status_Flags enumeration.
 * @return true if the flag is set, false otherwise.
 */
bool BKP::get_flag(Status_Flags flag) {
    return (read_bit(*this, BKP_Regs::TPCS, static_cast<uint32_t>(flag)));
}

/**
 * @brief Clears a specified flag in the BKP peripheral.
 *
 * This function clears a given flag specified by the Clear_Flags enumeration.
 * The flag is cleared by writing a 1 to the corresponding bit in the TPCS register.
 *
 * @param flag The flag to clear, specified as a Clear_Flags enumeration value.
 */
void BKP::clear_flag(Clear_Flags flag) {
    write_bit(*this, BKP_Regs::TPCS, static_cast<uint32_t>(flag), true);
}

/**
 * @brief Retrieves the status of the specified interrupt flag.
 *
 * This function reads the specified interrupt flag from the BKP status register (TPCS)
 * and returns its status. If the flag is not applicable to the BKP
 * peripheral, this function returns false.
 *
 * @param flag The interrupt flag to retrieve. Must be a value from the
 *             Interrupt_Flags enumeration.
 * @return true if the flag is set, false otherwise.
 */
bool BKP::get_interrupt_flag(Interrupt_Flags flag) {
    return (read_bit(*this, BKP_Regs::TPCS, static_cast<uint32_t>(flag)));
}

/**
 * @brief Clears a specified interrupt flag in the BKP peripheral.
 *
 * This function clears a given interrupt flag specified by the Clear_Flags
 * enumeration. The flag is cleared by writing a 1 to the corresponding bit
 * in the TPCS register.
 *
 * @param flag The interrupt flag to clear, specified as a Clear_Flags
 *             enumeration value.
 */
void BKP::clear_interrupt_flag(Clear_Flags flag) {
    write_bit(*this, BKP_Regs::TPCS, static_cast<uint32_t>(flag), true);
}


} // namespace bkp
