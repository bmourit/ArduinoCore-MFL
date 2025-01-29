//
// MFL gd32f30x GPIO peripheral register access in C++
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

#include "GPIO.hpp"
#include "RCU.hpp"
#include "AFIO.hpp"

namespace gpio {

template <GPIO_Base Base>
GPIO& get_instance_for_base() {
    static GPIO instance(Base);
    return instance;
}

Result<GPIO, GPIO_Error_Type> GPIO::get_instance(GPIO_Base Base) {
    switch (Base) {
    case GPIO_Base::GPIOA_BASE:
        return get_enum_instance<GPIO_Base, GPIO, GPIO_Error_Type>(
                   Base, GPIO_Base::GPIOA_BASE, get_instance_for_base<GPIO_Base::GPIOA_BASE>()
               );
    case GPIO_Base::GPIOB_BASE:
        return get_enum_instance<GPIO_Base, GPIO, GPIO_Error_Type>(
                   Base, GPIO_Base::GPIOB_BASE, get_instance_for_base<GPIO_Base::GPIOB_BASE>()
               );
    case GPIO_Base::GPIOC_BASE:
        return get_enum_instance<GPIO_Base, GPIO, GPIO_Error_Type>(
                   Base, GPIO_Base::GPIOC_BASE, get_instance_for_base<GPIO_Base::GPIOC_BASE>()
               );
    case GPIO_Base::GPIOD_BASE:
        return get_enum_instance<GPIO_Base, GPIO, GPIO_Error_Type>(
                   Base, GPIO_Base::GPIOD_BASE, get_instance_for_base<GPIO_Base::GPIOD_BASE>()
               );
    case GPIO_Base::INVALID:
    default:
        return RETURN_RESULT(GPIO, GPIO_Error_Type::INVALID_PORT);
    }
}

std::array<bool, static_cast<size_t>(GPIO_Base::INVALID)> GPIO::clock_enabled_ = {false};

GPIO::GPIO(GPIO_Base Base) :
    base_(Base),
    GPIO_pclk_info_(GPIO_pclk_index[static_cast<size_t>(Base)]),
    base_address_(GPIO_baseAddress[static_cast<uint8_t>(Base)])
{
    if (!clock_enabled_[static_cast<size_t>(Base)]) {
        RCU_I.set_pclk_enable(GPIO_pclk_info_.clock_reg, true);
        RCU_I.set_pclk_reset_enable(GPIO_pclk_info_.reset_reg, true);
        RCU_I.set_pclk_reset_enable(GPIO_pclk_info_.reset_reg, false);
        clock_enabled_[static_cast<size_t>(Base)] = true;
    }
}

/**
 * @brief Resets the GPIO peripheral by toggling the reset signal.
 *
 * This function enables and then disables the peripheral clock reset for the
 * GPIO, effectively resetting the GPIO peripheral to its default state.
 */
void GPIO::reset() {
    RCU_I.set_pclk_reset_enable(GPIO_pclk_info_.reset_reg, true);
    RCU_I.set_pclk_reset_enable(GPIO_pclk_info_.reset_reg, false);
}

/**
 * @brief Configures the mode, speed, and initialization of a specified GPIO pin.
 *
 * This function sets the mode and speed for the given GPIO pin.
 * It supports various pin modes including analog, input (floating, pull-up, pull-down),
 * output (push-pull, open-drain) and alternate function.
 *
 * @param pin The pin number to configure.
 * @param mode The desired pin mode (e.g., ANALOG, INPUT_FLOATING, OUTPUT_PUSHPULL).
 * @param speed The desired output speed (e.g., SPEED_50MHZ, SPEED_MAX).
 */
void GPIO::set_pin_mode(Pin_Number pin, Pin_Mode mode, Output_Speed speed) {
    if ((pin == Pin_Number::INVALID) || (mode == Pin_Mode::INVALID)) return;

    const uint32_t shift = (static_cast<uint32_t>(pin) & 0x7U) << 2U;
    const GPIO_Regs reg = (static_cast<uint32_t>(pin) >= 8U) ? GPIO_Regs::CTL1 : GPIO_Regs::CTL0;
    uint32_t ctl = read_register<uint32_t>(*this, reg) & ~(0xFU << shift);

    uint32_t cfg = 0U;
    if (mode <= Pin_Mode::INPUT_PULLDOWN) {
        // Handle input modes
        cfg = (mode == Pin_Mode::INPUT_FLOATING) ? 0x4U : 
              (mode == Pin_Mode::ANALOG) ? 0x0U : 0x8U;

        if (mode == Pin_Mode::INPUT_PULLUP)
            atomic_write_bit(*this, GPIO_Regs::BOP, static_cast<uint32_t>(pin), true);
        else if (mode == Pin_Mode::INPUT_PULLDOWN)
            atomic_write_bit(*this, GPIO_Regs::BC, static_cast<uint32_t>(pin), true);
    } else {
        // Handle output modes
        const bool is_open_drain = (mode == Pin_Mode::OUTPUT_OPENDRAIN || mode == Pin_Mode::ALT_OPENDRAIN);
        const bool is_alt = (mode == Pin_Mode::ALT_PUSHPULL || mode == Pin_Mode::ALT_OPENDRAIN);

        cfg = (is_open_drain ? 0x4U : 0x0U) | (is_alt ? 0x8U : 0x0U);

        if (speed == Output_Speed::SPEED_MAX) {
            cfg |= 0x3U;
            write_bit(*this, GPIO_Regs::SPD, static_cast<uint32_t>(pin), true);
            AFIO_I.set_compensation(true);
            while (!AFIO_I.get_compensation()) {
                // Wait for ready
            }
        } else {
            cfg |= (speed == Output_Speed::INVALID) ?
                   static_cast<uint32_t>(Output_Speed::SPEED_50MHZ) :
                   static_cast<uint32_t>(speed);
        }
    }

    write_register(*this, reg, ctl | (cfg << shift));
}

/**
 * @brief Get the current pin mode for the given pin.
 *
 * @param[in] pin The pin number to query.
 *
 * @return The current pin mode.
 */
Pin_Mode GPIO::get_pin_mode(Pin_Number pin) {
    if (pin == Pin_Number::INVALID) {
        return Pin_Mode::INVALID;
    }

    // Determine the correct register and bit position for the pin configuration
    const uint32_t shift = (static_cast<uint32_t>(pin) & 0x7U) << 2U;
    const GPIO_Regs reg = (static_cast<uint32_t>(pin) >= 8U) ? GPIO_Regs::CTL1 : GPIO_Regs::CTL0;

    // Read the current register value
    const uint32_t mode_bits = (read_register<uint32_t>(*this, reg) & (0xFU << shift)) >> shift;

    switch (mode_bits) {
    case 0x0U: return Pin_Mode::ANALOG;
    case 0x4U: return Pin_Mode::INPUT_FLOATING;
    case 0x8U: return get_pin_output_state(pin) ? Pin_Mode::INPUT_PULLUP : Pin_Mode::INPUT_PULLDOWN;
    case 0x1U: case 0x2U: case 0x3U: return Pin_Mode::OUTPUT_PUSHPULL;
    case 0x5U: case 0x6U: case 0x7U: return Pin_Mode::OUTPUT_OPENDRAIN;
    case 0x9U: case 0xAU: case 0xBU: return Pin_Mode::ALT_PUSHPULL;
    case 0xDU: case 0xEU: case 0xFU: return Pin_Mode::ALT_OPENDRAIN;
    default: return Pin_Mode::INVALID;
    }
}

/**
 * @brief Set the specified pin to a high state.
 *
 * @param[in] pin The pin number to set.
 */
void GPIO::set_pin_high(Pin_Number pin) {
    if (pin == Pin_Number::INVALID) {
        return;
    }
    atomic_write_bit(*this, GPIO_Regs::BOP, static_cast<uint32_t>(pin), true);
}

/**
 * @brief Set the specified pin to a low state.
 *
 * @param[in] pin The pin number to set.
 */
void GPIO::set_pin_low(Pin_Number pin) {
    if (pin == Pin_Number::INVALID) {
        return;
    }
    atomic_write_bit(*this, GPIO_Regs::BC, static_cast<uint32_t>(pin), true);
}

/**
 * @brief Sets the specified pin to a high or low state without changing the output mode.
 *
 * This function sets the specified pin to a high or low state without changing the output
 * mode. If the pin is currently in an output mode, then this function will set the pin to
 * the specified state. If the pin is currently in an input mode, then the pin will be set
 * to the specified pull state.
 *
 * @param[in] pin The pin number to set.
 * @param[in] high If true, the pin is set to a high state. If false, the pin is set to a low state.
 */
void GPIO::set_pin_level(Pin_Number pin, bool high) {
    if (pin == Pin_Number::INVALID) {
        return;
    }
    atomic_write_bit(*this, high ? GPIO_Regs::BOP : GPIO_Regs::BC, static_cast<uint32_t>(pin), true);
}

/**
 * @brief Writes the specified pin to a high or low state.
 *
 * This function writes the specified pin to a high or low state based on the value of the
 * `set` parameter. If `set` is true, the pin is set to a high state; if `set` is false, the
 * pin is set to a low state. The pin must be in an output mode for this function to have any
 * effect.
 *
 * @param[in] pin The pin number to write to.
 * @param[in] set If true, the pin is set to a high state. If false, the pin is set to a low state.
 */
void GPIO::write_pin(Pin_Number pin, bool set) {
    if (pin == Pin_Number::INVALID) {
        return;
    }
    atomic_write_bit(*this, set ? GPIO_Regs::BOP : GPIO_Regs::BC, static_cast<uint32_t>(pin), true);
}

/**
 * @brief Reads the current state of the specified pin.
 *
 * This function reads the current state of the specified pin. If the pin is currently in
 * an output mode, then this function will return the state of the pin. If the pin is
 * currently in an input mode, then this function will return the current state of the pin
 * as determined by the external hardware.
 *
 * @param[in] pin The pin number to read from.
 * @return The state of the specified pin. If the pin is in an output mode, the state
 *         returned is the last state written to the pin. If the pin is in an input mode,
 *         the state returned is the current state of the pin as determined by the
 *         external hardware.
 */
bool GPIO::read_pin(Pin_Number pin) {
    if (pin == Pin_Number::INVALID) {
        return false;
    }
    return read_bit(*this, GPIO_Regs::ISTAT, static_cast<uint32_t>(pin));
}

/**
 * @brief Toggles the specified GPIO pin.
 *
 * @param pin  GPIO pin to toggle.
 *
 * @note This function relies on the GPIO peripheral's semantics, where the BOP 
 *       (bit set) and BC (bit clear) registers are used for atomic pin operations:
 *         - Writing 1 << pin to BOP sets the pin high.
 *         - Writing 1 << pin to BC clears the pin low.
 *       The `atomic_write_bit` function always writes `true` in this context because
 *       the register selection determines whether the pin is set or cleared.
 *
 * @warning Since GPIO peripherals often include separate registers for set/clear,
 *          writing 0 (via `atomic_write_bit`) has no effect. Other peripherals 
 *          may behave differently when writing 0 to registers.
 */
void GPIO::toggle_pin(Pin_Number pin) {
    if (pin == Pin_Number::INVALID) {
        return;
    }
    bool is_set = read_bit(*this, GPIO_Regs::OCTL, static_cast<uint32_t>(pin));
    atomic_write_bit(*this, is_set ? GPIO_Regs::BC : GPIO_Regs::BOP, static_cast<uint32_t>(pin), true);
}

/**
 * @brief Writes the specified 16-bit data to the port.
 *
 * This function writes the specified 16-bit data to the port. The data is written to the
 * OCTL register, and the state of all pins on the port is modified accordingly.
 *
 * @param[in] data The 16-bit data to write to the port.
 */
void GPIO::set_port(uint16_t data) {
    write_register(*this, GPIO_Regs::OCTL, static_cast<uint32_t>(data));
}

/**
 * @brief Returns the current state of the specified pin as determined by the external hardware.
 *
 * This function returns the current state of the specified pin as determined by the external
 * hardware. If the pin is in an output mode, the state returned is the last state written to the
 * pin. If the pin is in an input mode, the state returned is the current state of the pin as
 * determined by the external hardware.
 *
 * @param[in] pin The pin number to read.
 * @return The current state of the pin. If the pin is in an output mode, the state returned is
 *         the last state written to the pin. If the pin is in an input mode, the state returned is
 *         the current state of the pin as determined by the external hardware.
 */
bool GPIO::get_pin_input_state(Pin_Number pin) {
    if (pin == Pin_Number::INVALID) {
        return false;
    }
    return read_bit(*this, GPIO_Regs::ISTAT, static_cast<uint32_t>(pin));
}

/**
 * @brief Gets the current output state of the specified pin.
 *
 * This function checks and returns the output state of the specified pin based
 * on the OCTL register. If the pin is in an output mode, the state returned is
 * the last state written to the pin. If the pin is invalid, it returns false.
 *
 * @param[in] pin The pin number to read from.
 * @return True if the pin is set to a high state, false otherwise.
 */
bool GPIO::get_pin_output_state(Pin_Number pin) {
    if (pin == Pin_Number::INVALID) {
        return false;
    }
    return read_bit(*this, GPIO_Regs::OCTL, static_cast<uint32_t>(pin));
}

/**
 * @brief Gets the current state of the entire port as determined by the external hardware.
 *
 * This function returns the current state of the entire port as determined by the external
 * hardware. If any pin on the port is in an output mode, the state returned will reflect the last
 * state written to that pin. If any pin on the port is in an input mode, the state returned will
 * reflect the current state of the pin as determined by the external hardware.
 *
 * @return The current state of the port.
 */
uint16_t GPIO::get_port_input_state() {
    return read_register<uint16_t>(*this, GPIO_Regs::ISTAT);
}

/**
 * @brief Gets the current output state of the entire port.
 *
 * This function returns the current state of the entire port as determined by
 * the OCTL register. If any pin on the port is in an output mode, the state
 * returned will reflect the last state written to that pin. If any pin on the
 * port is in an input mode, the state returned will reflect the current state of
 * the pin as determined by the external hardware.
 *
 * @return The current state of the port.
 */
uint16_t GPIO::get_port_output_state() {
    return read_register<uint16_t>(*this, GPIO_Regs::OCTL);
}

/**
 * @brief Locks the configuration of a specified GPIO pin.
 *
 * This function locks the configuration of the specified GPIO pin to prevent further
 * changes. The locking mechanism involves a specific sequence of write and read
 * operations on the LOCK register. Once a pin is locked, its configuration cannot be
 * modified until the microcontroller is reset.
 *
 * @param pin The pin number to lock.
 */
void GPIO::lock_pin(Pin_Number pin) {
    if (pin == Pin_Number::INVALID) { return; }
    uint32_t lock = LockValue;
    lock |= (1U << static_cast<size_t>(pin));

    // lock the pin by write/read in this order
    // write lock -> write pin -> write lock -> read 1x -> read 2x
    write_register(*this, GPIO_Regs::LOCK, lock);
    write_bit(*this, GPIO_Regs::LOCK, static_cast<uint32_t>(pin), true);
    write_register(*this, GPIO_Regs::LOCK, lock);
    lock = read_register<uint32_t>(*this, GPIO_Regs::LOCK);
    lock = read_register<uint32_t>(*this, GPIO_Regs::LOCK);
}


} // namespace gpio
