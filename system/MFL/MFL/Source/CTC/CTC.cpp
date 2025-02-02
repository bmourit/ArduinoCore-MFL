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
 * @brief Resets the CTC peripheral.
 *
 * This function resets the CTC peripheral by performing a peripheral
 * clock reset. All registers of the CTC peripheral are reset to their
 * default values.
 */
void CTC::reset() {
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_CTCRST, true);
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_CTCRST, false);
}

/**
 * @brief Enables or disables the trim counter.
 *
 * This function enables or disables the trim counter in the CTC peripheral
 * by setting or clearing the CNTEN bit in the CTL0 register.
 *
 * @param enable Set to true to enable the trim counter, or false to disable it.
 */
void CTC::set_trim_counter_enable(bool enable) {
    write_bit(*this, CTC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::CNTEN), enable);
}

/**
 * @brief Sets the IRC48M trim value.
 *
 * This function sets the IRC48M trim value by writing the provided value
 * to the TRIMVALUE bits in the CTL0 register. The IRC48M trim value is used
 * to adjust the frequency of the IRC48M clock source.
 *
 * @param value The new IRC48M trim value.
 */
void CTC::set_irc48m_trim(uint8_t value) {
    write_bit_range(*this, CTC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::TRIMVALUE), static_cast<uint32_t>(value));
}

/**
 * @brief Generates a software-controlled pulse on the reference source output.
 *
 * This function generates a software-controlled pulse on the reference source
 * output by setting the SWREFPUL bit in the CTL0 register. The pulse is
 * generated after a delay of 1 clock cycle. The pulse width is 1 clock cycle.
 */
void CTC::generate_reference_source_pulse() {
    write_bit(*this, CTC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SWREFPUL), true);
}

/**
 * @brief Enables or disables the hardware-controlled auto-trim feature.
 *
 * This function enables or disables the hardware-controlled auto-trim feature
 * by setting or clearing the AUTOTRIM bit in the CTL0 register. The
 * hardware-controlled auto-trim feature causes the CTC peripheral to
 * automatically adjust the IRC48M trim value based on the
 * reference source signal. The trim value is adjusted to keep the IRC48M
 * clock source in sync with the reference source signal.
 *
 * @param enable Set to true to enable the hardware-controlled auto-trim
 *               feature, or false to disable it.
 */
void CTC::set_hardware_auto_trim_enable(bool enable) {
    write_bit(*this, CTC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::AUTOTRIM), enable);
}

/**
 * @brief Sets the reference source signal polarity.
 *
 * This function sets the reference source signal polarity by setting or clearing
 * the REFOL bit in the CTL1 register. The reference source signal is used to
 * control the hardware-controlled auto-trim feature. The reference source signal
 * is either a rising or falling edge.
 *
 * @param polarity The desired reference source signal polarity. Use
 *                 Reference_Polarity::RISING to set the reference source signal
 *                 polarity to rising edge, or Reference_Polarity::FALLING to set
 *                 the reference source signal polarity to falling edge.
 */
void CTC::set_reference_source_polarity(Reference_Polarity polarity) {
    write_bit(*this, CTC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::REFPOL),
           (polarity == Reference_Polarity::FALLING) ? true : false);
}

/**
 * @brief Sets the reference source signal.
 *
 * This function configures the reference source signal by writing the provided
 * reference selection value to the REFSEL bits in the CTL1 register. The reference
 * source signal is used to control the hardware-controlled auto-trim feature.
 *
 * @param reference The reference source signal to be set, represented by the
 *                  Reference_Select enumeration.
 */
void CTC::set_refenece_source_signal(Reference_Select reference) {
    write_bit_range(*this, CTC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::REFSEL), static_cast<uint32_t>(reference));
}

/**
 * @brief Sets the prescaler value for the reference source signal.
 *
 * This function configures the prescaler value for the reference source signal
 * by writing the provided reference prescaler value to the REFPSC bits in the
 * CTL1 register. The reference source signal prescaler determines the division
 * factor applied to the clock signal provided to the reference source signal.
 *
 * @param prescaler The reference source signal prescaler to be set, represented
 *                  by the Reference_Prescaler enumeration.
 */
void CTC::set_reference_source_prescaler(Reference_Prescaler prescaler) {
    write_bit_range(*this, CTC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::REFPSC), static_cast<uint32_t>(prescaler));
}

/**
 * @brief Sets the clock trim limit for the CTC.
 *
 * This function sets the clock trim limit for the CTC by writing the provided
 * limit value to the CKLIM bits in the CTL1 register. The clock trim limit is
 * used to control the hardware-controlled auto-trim feature.
 *
 * @param limit The clock trim limit to be set. The limit value is a 5-bit
 *              unsigned integer.
 */
void CTC::set_clock_trim_limit(uint8_t limit) {
    write_bit_range(*this, CTC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::CKLIM), static_cast<uint32_t>(limit));
}

/**
 * @brief Sets the trim counter reload value.
 *
 * This function configures the trim counter reload value by writing the
 * specified reload value to the RLVALUE bits in the CTL1 register. The trim
 * counter reload value determines the number of clock cycles before the counter
 * is reloaded.
 *
 * @param reload The trim counter reload value to set.
 */
void CTC::set_trim_counter_reload(uint16_t reload) {
    write_bit_range(*this, CTC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::RLVALUE), static_cast<uint32_t>(reload));
}

/**
 * @brief Gets the current capture value of the trim counter.
 *
 * This function returns the current capture value of the trim counter by
 * reading from the REFCAp bits in the STAT register.
 *
 * @return The current capture value of the trim counter.
 */
uint16_t CTC::get_trim_counter_capture() {
    return static_cast<uint16_t>(read_bit_range(*this, CTC_Regs::STAT, static_cast<uint32_t>(STAT_Bits::REFCAP)));
}

/**
 * @brief Gets the current direction of the trim counter.
 *
 * This function returns the current direction of the trim counter by reading
 * the REFDIR bit in the STAT register. The direction indicates whether the
 * counter is counting up or down.
 *
 * @return true if the trim counter direction is up, false if it is down.
 */
bool CTC::get_trim_counter_direction() {
    return read_bit(*this, CTC_Regs::STAT, static_cast<uint32_t>(STAT_Bits::REFDIR));
}

/**
 * @brief Retrieves the current trim counter reload value.
 *
 * This function returns the current trim counter reload value by reading from
 * the RLVALUE bits in the CTL1 register.
 *
 * @return The current trim counter reload value.
 */
uint16_t CTC::get_trim_counter_reload() {
    return static_cast<uint16_t>(read_bit_range(*this, CTC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::RLVALUE)));
}

/**
 * @brief Retrieves the current trim value of the IRC48M oscillator.
 *
 * This function returns the current trim value of the IRC48M oscillator by
 * reading from the TRIMVALUE bits in the CTL0 register.
 *
 * @return The current trim value of the IRC48M oscillator.
 */
uint8_t CTC::get_trim_irc48m() {
    return static_cast<uint8_t>(read_bit_range(*this, CTC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::TRIMVALUE)));
}

/**
 * @brief Retrieves the status of the specified flag.
 *
 * This function reads the status register of the CTC peripheral and returns
 * the value of the specified flag, which can be any of the values in the
 * Status_Flags enumeration.
 *
 * @param flag The status flag to retrieve. Must be a value from the
 *             Status_Flags enumeration.
 * @return true if the flag is set, false otherwise.
 */
bool CTC::get_flag(Status_Flags flag) {
    return read_bit(*this, CTC_Regs::STAT, static_cast<uint32_t>(flag));
}

/**
 * @brief Clears a specified flag in the CTC peripheral.
 *
 * This function clears the specified flag in the CTC by writing a 1 to the
 * corresponding bit in the INTC register. The flag is cleared regardless of
 * whether or not the interrupt is enabled.
 *
 * @param flag The flag to clear, specified as a Clear_Flags enumeration value.
 */
void CTC::clear_flag(Clear_Flags flag) {
    write_register(*this, CTC_Regs::INTC, (1U << static_cast<uint32_t>(flag)));
}

/**
 * @brief Retrieves the status of the specified interrupt flag.
 *
 * This function checks both the status and control registers to determine if
 * a specified interrupt flag is set and enabled. If both the status and control
 * bits for the specified interrupt flag are set, the function returns true,
 * indicating that the interrupt condition is active and enabled.
 *
 * @param flag The interrupt flag to check, specified as an Interrupt_Flags enumeration value.
 * @return true if the specified interrupt flag is set and enabled, false otherwise.
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
 * @brief Clears a specified interrupt flag in the CTC peripheral.
 *
 * This function clears the specified interrupt flag in the CTC by writing a 1 to the
 * corresponding bit in the INTC register. The flag is cleared regardless of
 * whether or not the interrupt is enabled.
 *
 * @param flag The flag to clear, specified as a Clear_Flags enumeration value.
 */
void CTC::clear_interrupt_flag(Clear_Flags flag) {
    write_register(*this, CTC_Regs::INTC, (1U << static_cast<uint32_t>(flag)));
}

/**
 * @brief Enables or disables a specified interrupt type in the CTC peripheral.
 *
 * This function configures the interrupt settings for the CTC peripheral by
 * enabling or disabling the specified interrupt type. It modifies the interrupt
 * enable register to reflect the desired configuration.
 *
 * @param[in] type The interrupt type to enable or disable, specified as an
 *                 Interrupt_Type enumeration value.
 * @param[in] enable Set to true to enable the interrupt, false to disable it.
 */
void CTC::set_interrupt_enable(Interrupt_Type type, bool enable) {
    write_bit(*this, CTC_Regs::CTL0, static_cast<uint32_t>(type), enable);
}


} // namespace ctc
