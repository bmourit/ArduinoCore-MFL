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

#include "CTC.hpp"
#include "RCU.hpp"

namespace ctc {

CTC& CTC::get_instance() {
    static CTC instance;
    return instance;
}

CTC::CTC() : is_clock_enabled_(false) {
    if (!is_clock_enabled_) {
        RCU_I.set_pclk_enable(rcu::RCU_PCLK::PCLK_CTC, true);
        RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_CTCRST, true);
        RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_CTCRST, false);
        is_clock_enabled_ = true;
    }
}

/**
 * Reset the CTC peripheral.
 *
 * This function will assert the reset signal to the CTC peripheral and then
 * deassert it. This will cause the CTC peripheral to reset all of its internal
 * registers to their default values.
 */
void CTC::reset() {
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_CTCRST, true);
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_CTCRST, false);
}

/**
 * Enable or disable the trim counter.
 *
 * If the trim counter is enabled, the CTC peripheral will increment the trim
 * counter register (CTC_CTL1[15:0]) each time the IRC48M is toggled. The trim
 * counter register will wrap around to zero when it reaches 0x0000FFFF.
 *
 * @param[in] enable true to enable the trim counter, false to disable it
 */
void CTC::set_trim_counter_enable(bool enable) {
    write_bit(*this, CTC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::CNTEN), enable);
}

/**
 * Set the IRC48M trim value.
 *
 * This function sets the trim value for the IRC48M internal clock.
 * The value is written to the TRIMVALUE field of the CTC_CTL0 register.
 *
 * @param[in] value The trim value to set (0 to 255).
 */
void CTC::set_irc48m_trim(uint8_t value) {
    write_bit_range(*this, CTC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::TRIMVALUE), static_cast<uint32_t>(value));
}

/**
 * Generate a reference source pulse to initiate a reference source calibration.
 *
 * When this function is called, the CTC peripheral will generate a reference
 * source pulse to initiate a reference source calibration. This pulse will be
 * output on the REFOUT pin and will be active high.
 */
void CTC::generate_reference_source_pulse() {
    write_bit(*this, CTC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SWREFPUL), true);
}

/**
 * Enable or disable the hardware auto-trim feature.
 *
 * If the hardware auto-trim feature is enabled, the CTC peripheral will
 * automatically adjust the IRC48M trim value based on the reference clock
 * frequency. This feature is typically used to trim the IRC48M clock to a
 * specific frequency.
 *
 * @param[in] enable true to enable the hardware auto-trim feature, false to disable it
 */
void CTC::set_hardware_auto_trim_enable(bool enable) {
    write_bit(*this, CTC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::AUTOTRIM), enable);
}

/**
 * Set the reference source polarity.
 *
 * This function sets the polarity of the reference clock signal
 * that is output on the REFOUT pin. The polarity can be set
 * to either rising edge (Reference_Polarity::RISING) or falling
 * edge (Reference_Polarity::FALLING). The polarity is set in the
 * REFPOL field of the CTL1 register.
 *
 * @param[in] polarity The reference source polarity to set (RISING or FALLING)
 */
void CTC::set_reference_source_polarity(Reference_Polarity polarity) {
    write_bit(*this, CTC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::REFPOL), polarity == Reference_Polarity::FALLING ? true : false);
}

/**
 * Set the reference source signal.
 *
 * This function sets the reference source signal for the CTC peripheral.
 * The reference source signal is the clock signal that is used to generate
 * the IRC48M clock. The reference source signal is selected in the
 * REFSEL field of the CTL1 register.
 *
 * @param[in] reference The reference source signal to set (HXTAL, HXTAL_DIV2, or IRC8M)
 */
void CTC::set_refenece_source_signal(Reference_Select reference) {
    write_bit_range(*this, CTC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::REFSEL), static_cast<uint32_t>(reference));
}

/**
 * Set the prescaler for the reference source signal.
 *
 * This function sets the prescaler for the reference source signal that is
 * input to the IRC48M clock generator. The prescaler is set in the REFPSC
 * field of the CTL1 register.
 *
 * @param[in] prescaler The prescaler to set (DIV1 to DIV16)
 */
void CTC::set_reference_source_prescaler(Reference_Prescaler prescaler) {
    write_bit_range(*this, CTC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::REFPSC), static_cast<uint32_t>(prescaler));
}

/**
 * Set the IRC48M clock limit for the auto trim function.
 *
 * This function sets the upper limit for the IRC48M clock frequency
 * that is used to trim the IRC48M clock. The clock limit is set in the
 * CKLIM field of the CTL1 register.
 *
 * @param[in] limit The upper limit to set (0-255 in 100 Hz steps)
 */
void CTC::set_clock_trim_limit(uint8_t limit) {
    write_bit_range(*this, CTC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::CKLIM), static_cast<uint32_t>(limit));
}

/**
 * Set the trim counter reload value.
 *
 * This function sets the reload value for the trim counter of the CTC
 * peripheral. The trim counter is used to generate a reference clock signal
 * for the IRC48M clock generator. The reload value is set in the RLVALUE
 * field of the CTL1 register.
 *
 * @param[in] reload The reload value to set (0-65535)
 */
void CTC::set_trim_counter_reload(uint16_t reload) {
    write_bit_range(*this, CTC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::RLVALUE), static_cast<uint32_t>(reload));
}

/**
 * Get the capture value of the trim counter.
 *
 * This function returns the capture value of the trim counter of the CTC
 * peripheral. The capture value is the value of the trim counter at the
 * time of the last IRC48M clock edge. The capture value is read from the
 * REFCAP field of the STAT register.
 *
 * @return The capture value of the trim counter (0-65535)
 */
uint16_t CTC::get_trim_counter_capture() {
    return static_cast<uint16_t>(read_bit_range(*this, CTC_Regs::STAT, static_cast<uint32_t>(STAT_Bits::REFCAP)));
}

/**
 * Get the direction of the trim counter.
 *
 * This function returns the direction of the trim counter of the CTC
 * peripheral. The direction is either incrementing (true) or decrementing
 * (false). The direction is read from the REFDIR bit of the STAT register.
 *
 * @return The direction of the trim counter (true for incrementing, false for decrementing)
 */
bool CTC::get_trim_counter_direction() {
    return read_bit(*this, CTC_Regs::STAT, static_cast<uint32_t>(STAT_Bits::REFDIR));
}

/**
 * Get the trim counter reload value.
 *
 * This function returns the trim counter reload value as set by the
 * set_trim_counter_reload function. The reload value is read from the
 * RLVALUE field of the CTL1 register.
 *
 * @return The trim counter reload value (0-65535)
 */
uint16_t CTC::get_trim_counter_reload() {
    return static_cast<uint16_t>(read_bit_range(*this, CTC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::RLVALUE)));
}

/**
 * Get the IRC48M trim value.
 *
 * This function returns the IRC48M trim value that has been set by the
 * set_irc48m_trim function. The trim value is read from the TRIMVALUE
 * field of the CTL0 register.
 *
 * @return The IRC48M trim value (0-255)
 */
uint8_t CTC::get_trim_irc48m() {
    return static_cast<uint8_t>(read_bit_range(*this, CTC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::TRIMVALUE)));
}

/**
 * Check the status of a specific flag in the CTC peripheral.
 *
 * This function reads the status of the specified flag from the STAT register
 * of the CTC peripheral. The flag indicates a particular status or event within
 * the CTC module.
 *
 * @param[in] flag The status flag to check.
 * @return True if the flag is set, false otherwise.
 */
bool CTC::get_flag(Status_Flags flag) {
    return read_bit(*this, CTC_Regs::STAT, static_cast<uint32_t>(flag));
}

/**
 * Clear a specified flag in the CTC peripheral.
 *
 * This function clears a given flag specified by the Clear_Flags enumeration.
 * The flag is cleared by writing a 1 to the corresponding bit in the INTC register.
 *
 * @param[in] flag The flag to clear, specified as a Clear_Flags enumeration value.
 */
void CTC::clear_flag(Clear_Flags flag) {
    write_register(*this, CTC_Regs::INTC, (1U << static_cast<uint32_t>(flag)));
}

/**
 * Check the status of a specific interrupt flag in the CTC peripheral.
 *
 * This function evaluates whether a specified interrupt flag is active by
 * checking both the status and control registers. It verifies if the
 * interrupt is enabled and the corresponding flag is set.
 *
 * @param flag The interrupt flag to check. Must be one of the Interrupt_Flags
 *             enumeration values.
 * @return True if the specified interrupt flag is set and enabled, false otherwise.
 */
bool CTC::get_interrupt_flag(Interrupt_Flags flag) {
    bool enabled = false;

    if (((1U << static_cast<uint32_t>(flag)) & INTR_ERRIC_FLAG_MASK)) {
        enabled = read_bit(*this, CTC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::ERRIE));
    } else {
        enabled = read_bit(*this, CTC_Regs::CTL0, static_cast<uint32_t>(flag));
    }
    bool status_flag = read_bit(*this, CTC_Regs::STAT, static_cast<uint32_t>(flag));

    return (enabled && status_flag);
}

/**
 * Clear a specified interrupt flag in the CTC peripheral.
 *
 * This function clears a given interrupt flag specified by the Clear_Flags enumeration.
 * The flag is cleared by writing a 1 to the corresponding bit in the INTC register.
 *
 * @param[in] flag The interrupt flag to clear, specified as a Clear_Flags enumeration value.
 */
void CTC::clear_interrupt_flag(Clear_Flags flag) {
    write_register(*this, CTC_Regs::INTC, (1U << static_cast<uint32_t>(flag)));
}

/**
 * Enable or disable a specific interrupt in the CTC peripheral.
 *
 * This function is used to enable or disable specific interrupts in the CTC
 * peripheral. The interrupt type is specified as one of the Interrupt_Type
 * enumeration values.
 *
 * @param type The interrupt type to enable or disable. Must be one of the
 *             Interrupt_Type enumeration values.
 * @param enable True to enable the interrupt, false to disable it.
 */
void CTC::set_interrupt_enable(Interrupt_Type type, bool enable) {
    write_bit(*this, CTC_Regs::CTL0, static_cast<uint32_t>(type), enable);
}


} // namespace ctc
