//
// MFL gd32f30x RTC peripheral register access in C++
//
// Copyright (C) 2024 B. Mouritsen <bnmguy@gmail.com>. All rights reserved.
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

#include "RTC.hpp"
#include "RCU.hpp"
#include "PMU.hpp"
#include "BKP.hpp"

namespace rtc {

RTC& RTC::get_instance() {
    static RTC instance;
    return instance;
}

RTC::RTC() :
    bkp_(bkp::BKP::get_instance()),
    is_clock_enabled_(false)
{
    if (!is_clock_enabled_) {
        PMU_I.set_backup_write_enable(true);
        RCU_I.set_pclk_enable(rcu::RCU_PCLK::PCLK_RTC, true);
        is_clock_enabled_ = true;
    }
}

/**
 * @brief Enable RTC configuration mode
 *
 * This function will enable the RTC's configuration mode. The configuration mode
 * allows the RTC's registers to be modified. Without this mode enabled, the RTC's
 * registers are blocked from modification.
 *
 * @note This function will not wait until the configuration mode is actually
 * enabled. It is up to the caller to wait for the mode to be enabled.
 * @see wait_for_configuration_mode
 */
void RTC::start_configuration() {
    write_bit(*this, RTC_Regs::CTL, static_cast<uint32_t>(CTL_Bits::CMF), true);
}

/**
 * @brief Disable RTC configuration mode
 *
 * This function will disable the RTC's configuration mode. The configuration
 * mode allows the RTC's registers to be modified. Without this mode enabled, the
 * RTC's registers are blocked from modification.
 *
 * @note This function will not wait until the configuration mode is actually
 * disabled. It is up to the caller to wait for the mode to be disabled.
 * @see wait_for_configuration_mode
 */
void RTC::stop_configuration() {
    write_bit(*this, RTC_Regs::CTL, static_cast<uint32_t>(CTL_Bits::CMF), false);
}

/**
 * @brief Wait until the low power working mode is off
 *
 * This function will loop until the LWOFF flag is set. The LWOFF flag is
 * set when the low power working mode is off.
 */
void RTC::lwoff_wait() {
    // Loop until LWOFF flag gets set
    while (!read_bit(*this, RTC_Regs::CTL, static_cast<uint32_t>(CTL_Bits::LWOFF))) {
    }
}

/**
 * @brief Wait until the RTC's registers are synchronized
 *
 * This function will loop until the RSYNF flag is set. The RSYNF flag is
 * set when the RTC's registers are synchronized.
 */
void RTC::sync_register_wait() {
    // Clear RSYNF
    write_bit(*this, RTC_Regs::CTL, static_cast<uint32_t>(CTL_Bits::RSYNF), false);
    // Loop until RSYNF flag gets set
    while (!read_bit(*this, RTC_Regs::CTL, static_cast<uint32_t>(CTL_Bits::RSYNF))) {
    }
}

/**
 * @brief Get the current counter value of the RTC
 *
 * This function reads the high and low parts of the RTC counter
 * and returns the complete 32-bit counter value.
 *
 * @return The current counter value as a 32-bit unsigned integer.
 */
uint32_t RTC::get_counter() {
    return ((read_bit_range(*this, RTC_Regs::CNTH, static_cast<uint32_t>(CNTH_Bits::HIGH_CNT)) << 16) |
            read_bit_range(*this, RTC_Regs::CNTL, static_cast<uint32_t>(CNTL_Bits::LOW_CNT)));
}

/**
 * @brief Sets the current counter value of the RTC
 *
 * This function sets the current counter value of the RTC. The counter value
 * is a 32-bit unsigned integer. The value is split into two parts, a high part
 * and a low part. The high part is stored in the CNTH register and the low part
 * is stored in the CNTL register.
 *
 * @param counter The counter value as a 32-bit unsigned integer.
 */
void RTC::set_counter(uint32_t counter) {
    write_bit(*this, RTC_Regs::CTL, static_cast<uint32_t>(CTL_Bits::CMF), true);
    // Set CNTH
    write_bit_range(*this, RTC_Regs::CNTH, static_cast<uint32_t>(CNTH_Bits::HIGH_CNT), counter >> 16U);
    // Set CNTL
    write_bit_range(*this, RTC_Regs::CNTL, static_cast<uint32_t>(CNTL_Bits::LOW_CNT), counter & 0x0000FFFF);
    write_bit(*this, RTC_Regs::CTL, static_cast<uint32_t>(CTL_Bits::CMF), false);
}

/**
 * @brief Sets the prescaler value for the RTC.
 *
 * This function configures the prescaler value for the RTC by updating the
 * prescaler high (PSCH) and prescaler low (PSCL) register fields. The
 * prescaler value is a 32-bit unsigned integer, where the upper 4 bits are
 * stored in the PSCH register and the lower 16 bits are stored in the PSCL
 * register.
 *
 * @param prescaler The 32-bit prescaler value to set.
 */
void RTC::set_prescaler(uint32_t prescaler) {
    write_bit(*this, RTC_Regs::CTL, static_cast<uint32_t>(CTL_Bits::CMF), true);
    // Set PSCH
    write_bit_range(*this, RTC_Regs::PSCH, static_cast<uint32_t>(PSCH_Bits::HIGH_PSC), (prescaler & 0x000F0000U) >> 16U);
    // Set PSCL
    write_bit_range(*this, RTC_Regs::PSCL, static_cast<uint32_t>(PSCL_Bits::LOW_PSC), prescaler & 0x0000FFFFU);
    write_bit(*this, RTC_Regs::CTL, static_cast<uint32_t>(CTL_Bits::CMF), false);
}

/**
 * @brief Sets the alarm value for the RTC.
 *
 * This function sets the alarm value for the RTC. The alarm value is a 32-bit
 * unsigned integer, where the upper 16 bits are stored in the ALRMH register
 * and the lower 16 bits are stored in the ALRML register.
 *
 * @param alarm The 32-bit alarm value to set.
 */
void RTC::set_alarm(uint32_t alarm) {
    write_bit(*this, RTC_Regs::CTL, static_cast<uint32_t>(CTL_Bits::CMF), true);
    // Set ALRMH
    write_bit_range(*this, RTC_Regs::ALRMH, static_cast<uint32_t>(ALRMH_Bits::HIGH_ALRM), alarm >> 16U);
    // Set ALRML
    write_bit_range(*this, RTC_Regs::ALRML, static_cast<uint32_t>(ALRML_Bits::LOW_ALRM), alarm & 0x0000FFFFU);
    write_bit(*this, RTC_Regs::CTL, static_cast<uint32_t>(CTL_Bits::CMF), false);
}

/**
 * @brief Get the current divider value of the RTC
 *
 * This function reads the high and low parts of the divider value
 * and returns the complete 32-bit divider value.
 *
 * @return The current divider value as a 32-bit unsigned integer.
 */
uint32_t RTC::get_divider() {
    return ((read_bit_range(*this, RTC_Regs::DIVH, static_cast<uint32_t>(DIVH_Bits::HIGH_DIV)) << 16) |
            read_bit_range(*this, RTC_Regs::DIVL, static_cast<uint32_t>(DIVL_Bits::LOW_DIV)));
}

/**
 * @brief Returns the status of a specified flag in the RTC.
 *
 * This function reads the status of the specified flag from the CTL register
 * of the RTC peripheral. The flag indicates a particular status or event within
 * the RTC module.
 *
 * @param[in] flag The status flag to retrieve. Must be a value from the
 *                 Status_Flags enumeration.
 * @return true if the flag is set, false otherwise.
 */
bool RTC::get_flag(Status_Flags flag) {
    return read_bit(*this, RTC_Regs::CTL, static_cast<uint32_t>(flag));
}

/**
 * @brief Clears a specified status flag in the RTC.
 *
 * This function clears a specified status flag in the RTC by writing a
 * 0 to the corresponding bit in the CTL register.
 *
 * @param[in] flag The status flag to clear. Must be a value from the
 *                 Status_Flags enumeration.
 */
void RTC::clear_flag(Status_Flags flag) {
    write_bit(*this, RTC_Regs::CTL, static_cast<uint32_t>(flag), false);
}

/**
 * @brief Enables or disables a specific interrupt for the RTC.
 *
 * This function configures the interrupt settings for the RTC by enabling
 * or disabling the specified interrupt type. It modifies the interrupt
 * enable register to reflect the desired configuration.
 *
 * @param type The interrupt type to enable or disable. Must be a value from
 *             the Interrupt_Type enumeration.
 * @param enable Set to true to enable the interrupt, false to disable it.
 */
void RTC::set_interrupt_enable(Interrupt_Type type, bool enable) {
    write_bit(*this, RTC_Regs::INTEN, static_cast<uint32_t>(type), enable);
}


} // namespace rtc
