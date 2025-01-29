//
// MFL gd32f30x AFIO peripheral register access in C++
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

#include "AFIO.hpp"
#include "RCU.hpp"

namespace gpio {

AFIO& AFIO::get_instance() {
    static AFIO instance;
    return instance;
}

AFIO::AFIO() : 
    is_clock_enabled_(false)
{
    if (!is_clock_enabled_) {
        RCU_I.set_pclk_enable(rcu::RCU_PCLK::PCLK_AF, true);
        RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_AFRST, true);
        RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_AFRST, false);
        is_clock_enabled_ = true;
    }
}

/**
 * @brief Resets the AFIO peripheral by toggling the reset signal.
 *
 * This function enables and then disables the peripheral clock reset for AFIO,
 * effectively resetting the AFIO peripheral to its default state.
 */
void AFIO::reset() {
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_AFRST, true);
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_AFRST, false);
}

/**
 * @brief Configures the pin remapping for the AFIO peripheral.
 *
 * This function sets the pin remapping configuration based on the specified
 * remap selection. It writes the appropriate bitfields to the AFIO registers
 * to achieve the desired remap configuration.
 *
 * @param remap The pin remap selection to configure.
 */
void AFIO::set_remap(Pin_Remap_Select remap) {
    const auto& info = remap_index[static_cast<size_t>(remap)];
    write_bit_range(*this, info.register_offset, info.bit_info, static_cast<uint32_t>(info.type));
}

/**
 * @brief Sets the EXTI source for the specified pin.
 *
 * This function sets the EXTI source selection for the specified pin. The
 * source selection is used to determine which GPIO port is used to trigger
 * the EXTI interrupt. The source selection is specified by the `port`
 * parameter, which should be set to the desired GPIO port. The pin number
 * to be configured is specified by the `pin` parameter.
 *
 * @param port The GPIO port to use as the EXTI source.
 * @param pin The pin number to configure.
 */
void AFIO::set_exti_source(Source_Port port, Pin_Number pin) {
    if (pin == Pin_Number::INVALID || port == Source_Port::INVALID) {
        return;
    }
    const auto& source_info = source_index[static_cast<size_t>(pin)];
    write_bit_range(*this, source_info.reg, source_info.bit_info, static_cast<uint32_t>(port));
}

/**
 * @brief Configures the output event for the specified pin.
 *
 * This function sets the output event configuration for the specified pin.
 * The output event configuration is used to determine which GPIO port is used
 * to trigger the output event. The port selection is specified by the `port`
 * parameter, which should be set to the desired GPIO port. The pin number
 * to be configured is specified by the `pin` parameter.
 *
 * @param port The GPIO port to use as the output event source.
 * @param pin The pin number to configure.
 */
void AFIO::set_output_event(Event_Port port, Pin_Number pin) {
    if (pin == Pin_Number::INVALID || port == Event_Port::INVALID) {
        return;
    }
    write_bit_ranges(*this, AFIO_Regs::EC,
           static_cast<uint32_t>(EC_Bits::PORT), static_cast<uint32_t>(port),
           static_cast<uint32_t>(EC_Bits::PIN), static_cast<uint32_t>(pin));
}

/**
 * @brief Enables or disables the output event.
 *
 * This function sets the output event enable (EOE) bit in the AFIO_EC register.
 * If the `enable` parameter is `true`, the output event is enabled. If the
 * `enable` parameter is `false`, the output event is disabled.
 *
 * @param enable Set to true to enable the output event, false to disable it.
 */
void AFIO::set_output_event_enable(bool enable) {
    write_bit(*this, AFIO_Regs::EC, static_cast<uint32_t>(EC_Bits::EOE), enable);
}

/**
 * @brief Enables or disables the GPIO compensation.
 *
 * This function sets the GPIO compensation enable bit in the CPSCTL
 * register. If the `enable` parameter is `true`, the GPIO compensation is
 * enabled. If the `enable` parameter is `false`, the GPIO compensation is
 * disabled.
 *
 * @param enable Set to true to enable GPIO compensation, false to disable it.
 */
void AFIO::set_compensation(bool enable) {
    write_bit(*this, AFIO_Regs::CPSCTL, static_cast<uint32_t>(CPSCTL_Bits::CPS_EN), enable);
}

/**
 * @brief Checks if GPIO compensation is ready.
 *
 * This function reads the compensation ready bit from the CPSCTL register
 * and returns its status. It determines if the GPIO compensation circuitry
 * is ready to be used.
 *
 * @return true if GPIO compensation is ready, false otherwise.
 */
bool AFIO::get_compensation() {
    return read_bit(*this, AFIO_Regs::CPSCTL, static_cast<uint32_t>(CPSCTL_Bits::CPS_RDY));
}


} // namespace gpio

gpio::AFIO& AFIO_I = gpio::AFIO::get_instance();
