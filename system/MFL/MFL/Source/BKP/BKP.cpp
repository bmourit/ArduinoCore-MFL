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
 * Resets the BKP peripheral.
 *
 * This function will perform a reset of the BKP peripheral by asserting and then
 * deasserting the PCLK_BKPIRST bit in the RCU APC register. This will reset the
 * entire BKP peripheral and all its registers.
 */
void BKP::reset() {
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_BKPIRST, true);
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_BKPIRST, false);
}

/**
 * Sets the backup data register with the specified data value.
 *
 * This function writes a 16-bit data value to a specified backup data register
 * within the BKP peripheral. The register is selected based on the provided
 * Backup_Data enumeration value. The data is written only if the specified 
 * register index is within the valid range (DATA_0 to DATA_41).
 *
 * @param datax The backup data register index to set, specified as a 
 *              Backup_Data enumeration value.
 * @param data  The 16-bit data value to write to the selected backup 
 *              data register.
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
 * Retrieves the 16-bit data from a specified backup data register.
 *
 * This function reads the value from a backup data register within the
 * BKP peripheral. The register is selected based on the provided
 * Backup_Data enumeration value. The function returns the data only if
 * the specified register index is within the valid range (DATA_0 to DATA_41).
 *
 * @param datax The backup data register index to read, specified as a 
 *              Backup_Data enumeration value.
 * @return The 16-bit data value read from the selected backup data register,
 *         or 0 if the register index is invalid.
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
 * Enables or disables the output calibration for the RTC.
 *
 * When the output calibration is enabled, the RTC clock output is
 * divided by 32, and the output is the divided clock. The divided clock
 * is used to perform the calibration of the RTC.
 *
 * @param enable Set to true to enable the output calibration, or false to
 *               disable it.
 */
void BKP::set_rtc_output_calibration_enable(bool enable) {
    write_bit(*this, BKP_Regs::OCTL, static_cast<uint32_t>(OCTL_Bits::COEN), enable);
}

/**
 * Enables or disables the output signal for the RTC.
 *
 * When the output signal is enabled, the RTC output signal is
 * available at the output pin. The output signal is the clock
 * signal divided by the value specified in the RCCV field of the
 * OCTL register.
 *
 * @param enable Set to true to enable the output signal, or false to
 *               disable it.
 */
void BKP::set_rtc_output_signal_enable(bool enable) {
    write_bit(*this, BKP_Regs::OCTL, static_cast<uint32_t>(OCTL_Bits::ASOEN), enable);
}

/**
 * Selects the pulse width of the RTC output signal.
 *
 * The pulse width of the RTC output signal is controlled by the ROSEL bit of the OCTL
 * register. The pulse width is either 1 second or 2 seconds. If the ROSEL bit is set to
 * 0, the pulse width is 1 second. If the ROSEL bit is set to 1, the pulse width is 2
 * seconds.
 *
 * @param pulse Set to SECOND_PULSE to select a pulse width of 1 second, or
 *              SECOND_HALF_PULSE to select a pulse width of 2 seconds.
 */
void BKP::set_rtc_output_pulse(Output_Pulse pulse) {
    write_bit(*this, BKP_Regs::OCTL, static_cast<uint32_t>(OCTL_Bits::ROSEL),
           (pulse == Output_Pulse::SECOND_PULSE) ? true : false);
}

/**
 * Selects the clock divider for the RTC.
 *
 * The RTC clock divider is used to divide the clock signal that is used to
 * drive the RTC. The clock divider is either 1 (no division) or 32 (divide
 * by 32). If the CCOSEL bit of the OCTL register is set to 0, the clock divider
 * is 1. If the CCOSEL bit is set to 1, the clock divider is 32.
 *
 * @param divider Set to DIV_1 to select a clock divider of 1, or DIV_32 to
 *                select a clock divider of 32.
 */
void BKP::set_rtc_clock_divider(Clock_Divider divider) {
    write_bit(*this, BKP_Regs::OCTL, static_cast<uint32_t>(OCTL_Bits::CCOSEL),
           (divider == Clock_Divider::DIV_1) ? true : false);
}

/**
 * Selects the type of calibration for the RTC clock.
 *
 * The RTC clock can be calibrated to either speed up or slow down. This
 * function sets the CALDIR bit of the OCTL register to select the
 * type of calibration. If the CALDIR bit is set to 0, the RTC clock
 * is calibrated to slow down. If the CALDIR bit is set to 1, the
 * RTC clock is calibrated to speed up.
 *
 * @param type The type of calibration to select, specified as a
 *             Calibration_Type enumeration value.
 */
void BKP::set_rtc_clock_calibration_type(Calibration_Type type) {
    write_bit(*this, BKP_Regs::OCTL, static_cast<uint32_t>(OCTL_Bits::CALDIR),
           (type == Calibration_Type::SPEED_UP) ? true : false);
}

/**
 * Sets the value of the RTC calibration.
 *
 * The value of the RTC calibration is used to adjust the frequency of the
 * RTC clock. The calibration value is a 5-bit value that is stored in the
 * RCCV field of the OCTL register. The calibration value is used to adjust
 * the frequency of the RTC clock to within +/- 1% of the nominal frequency.
 *
 * @param value The value of the RTC calibration, specified as a
 *              uint8_t value in the range 0 to 31.
 */
void BKP::set_rtc_calibration_value(uint8_t value) {
    write_bit_range(*this, BKP_Regs::OCTL, static_cast<uint32_t>(OCTL_Bits::RCCV), static_cast<uint32_t>(value));
}

/**
 * Enables or disables the tamper detection function.
 *
 * @param enable Set to true to enable the tamper detection function, or false
 *               to disable it.
 */
void BKP::set_tamper_detection_enable(bool enable) {
    write_bit(*this, BKP_Regs::TPCTL, static_cast<uint32_t>(TPCTL_Bits::TPEN), enable);
}

/**
 * Sets the tamper detection level for the BKP peripheral.
 *
 * This function configures the tamper detection level to either active
 * high or active low, based on the specified `level` parameter. This setting
 * determines the voltage level at which the tamper detection event is triggered.
 *
 * @param level The tamper detection level to set, specified as a
 *              Tamper_Level enumeration value. It can be either
 *              ACTIVE_HIGH or ACTIVE_LOW.
 */
void BKP::set_tamper_level(Tamper_Level level) {
    write_bit(*this, BKP_Regs::TPCTL, static_cast<uint32_t>(TPCTL_Bits::TPAL),
           level == Tamper_Level::ACTIVE_LOW ? true : false);
}

/**
 * Enables or disables the tamper interrupt for the BKP peripheral.
 *
 * @param enable Set to true to enable the tamper interrupt, or false to disable it.
 */
void BKP::set_tamper_interrupt_enable(bool enable) {
    write_bit(*this, BKP_Regs::TPCS, static_cast<uint32_t>(TPCS_Bits::TPIE), enable);
}

/**
 * Checks the status of a specified flag within the BKP peripheral.
 *
 * This function reads the status of a given flag specified by the 
 * Status_Flags enumeration. The result indicates whether the flag is set.
 *
 * @param flag The status flag to check, specified as a Status_Flags 
 *             enumeration value.
 * @return True if the specified flag is set, otherwise false.
 */
bool BKP::get_flag(Status_Flags flag) {
    return (read_bit(*this, BKP_Regs::TPCS, static_cast<uint32_t>(flag)));
}

/**
 * Clears a specified flag within the BKP peripheral.
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
 * Checks the status of a specified interrupt flag within the BKP peripheral.
 *
 * This function reads the status of a given interrupt flag specified by the 
 * Interrupt_Flags enumeration. The result indicates whether the flag is set.
 *
 * @param flag The interrupt flag to check, specified as an Interrupt_Flags 
 *             enumeration value.
 * @return True if the specified flag is set, otherwise false.
 */
bool BKP::get_interrupt_flag(Interrupt_Flags flag) {
    return (read_bit(*this, BKP_Regs::TPCS, static_cast<uint32_t>(flag)));
}

/**
 * Clears a specified interrupt flag within the BKP peripheral.
 *
 * This function clears a given interrupt flag specified by the Clear_Flags enumeration.
 * The flag is cleared by writing a 1 to the corresponding bit in the TPCS register.
 *
 * @param flag The interrupt flag to clear, specified as a Clear_Flags enumeration value.
 */
void BKP::clear_interrupt_flag(Clear_Flags flag) {
    write_bit(*this, BKP_Regs::TPCS, static_cast<uint32_t>(flag), true);
}


} // namespace bkp
