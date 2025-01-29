//
// MFL EXTI peripheral register access in C++
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


#include "EXTI.hpp"
#include "RCU.hpp"

namespace exti {

EXTI& EXTI::get_instance() {
    static EXTI instance;
    return instance;
}

EXTI::EXTI() {}

/**
 * @brief Initialize the EXTI line configuration.
 * 
 * This function initializes a specified EXTI line with the given mode and trigger settings.
 * It first resets the relevant registers for the line, disabling any previously set events 
 * or interrupts. Then, it configures the EXTI line to operate in interrupt or event mode 
 * based on the provided EXTI_Mode value. Additionally, it sets the trigger condition for 
 * the EXTI line according to the EXTI_Trigger value, allowing for rising, falling, both, 
 * or no edge detection.
 * 
 * @param line The EXTI line to configure. Must be a valid EXTI_Line value.
 * @param mode The operation mode for the EXTI line, either interrupt or event.
 * @param trigger The trigger condition for the EXTI line, specifying which edge(s) 
 *                should generate an interrupt or event.
 */
void EXTI::init(EXTI_Line line, EXTI_Mode mode, EXTI_Trigger trigger) {
    if (line == EXTI_Line::INVALID) {
        return;
    }

    // Reset
    write_bit(*this, EXTI_Regs::EVEN, static_cast<uint32_t>(line), false);
    write_bit(*this, EXTI_Regs::INTEN, static_cast<uint32_t>(line), false);
    write_bit(*this, EXTI_Regs::RTEN, static_cast<uint32_t>(line), false);
    write_bit(*this, EXTI_Regs::FTEN, static_cast<uint32_t>(line), false);

    // EXTI mode with interrupts or events
    const EXTI_Regs reg = (mode == EXTI_Mode::EXTI_INTERRUPT) ? EXTI_Regs::INTEN : EXTI_Regs::EVEN;
    write_bit(*this, reg, static_cast<uint32_t>(line), true);

    // EXTI trigger edge
    const bool rising_edge = (trigger == EXTI_Trigger::TRIG_FALLING) ? false : true;
    const bool falling_edge = (trigger == EXTI_Trigger::TRIG_RISING) ? false : true;
    write_bit(*this, EXTI_Regs::RTEN, static_cast<uint32_t>(line), rising_edge);
    write_bit(*this, EXTI_Regs::FTEN, static_cast<uint32_t>(line), falling_edge);
}

/**
 * @brief Resets the EXTI peripheral registers to their default values.
 *
 * This function resets all EXTI registers, effectively disabling all EXTI lines
 * and clearing all pending interrupts and events. The function writes the default
 * reset values to the EVEN, INTEN, SWIEV, RTEN, and FTEN registers, respectively.
 */
void EXTI::reset() {
    write_register(*this, EXTI_Regs::EVEN, Clear);
    write_register(*this, EXTI_Regs::INTEN, Clear);
    write_register(*this, EXTI_Regs::SWIEV, Clear);
    write_register(*this, EXTI_Regs::RTEN, Clear);
    write_register(*this, EXTI_Regs::FTEN, Clear);
}

/**
 * @brief Enables or disables the EXTI line event.
 *
 * This function sets or clears the bit in the EVEN register that corresponds
 * to the given EXTI line. If the `enable` parameter is true, the EXTI line
 * event is enabled, and if the `enable` parameter is false, the EXTI line
 * event is disabled.
 *
 * @param line The EXTI line to enable or disable the event for.
 * @param enable Set to true to enable the event, false to disable it.
 */
void EXTI::set_event_enable(EXTI_Line line, bool enable) {
    if (line == EXTI_Line::INVALID) {
        return;
    }
    write_bit(*this, EXTI_Regs::EVEN, static_cast<uint32_t>(line), enable);
}

/**
 * @brief Returns the status of the specified flag.
 *
 * This function retrieves the status of the specified flag from the EXTI
 * status register (PD). If the flag is not applicable to the EXTI peripheral,
 * this function returns false.
 *
 * @param flag The status flag to retrieve. Must be a value from the
 *             Status_Flags enumeration.
 * @return true if the flag is set, false otherwise.
 */
bool EXTI::get_flag(Status_Flags flag) {
    if (flag == Status_Flags::INVALID) {
        return false;
    }
    return read_bit(*this, EXTI_Regs::PD, static_cast<uint32_t>(flag));
}

/**
 * @brief Clears the specified flag.
 *
 * This function clears the specified flag by writing a 1 to the
 * corresponding bit in the PD register. If the flag is not applicable
 * to the EXTI peripheral, this function does nothing.
 *
 * @param flag The flag to clear. Must be a value from the Status_Flags
 *             enumeration.
 */
void EXTI::clear_flag(Status_Flags flag) {
    if (flag == Status_Flags::INVALID) {
        return;
    }
    write_bit(*this, EXTI_Regs::PD, static_cast<uint32_t>(flag), true);
}

/**
 * @brief Retrieves the status of the specified interrupt flag.
 *
 * This function retrieves the status of the specified interrupt flag from the
 * EXTI status register (PD). If the flag is not applicable to the EXTI
 * peripheral, this function returns false.
 *
 * @param flag The interrupt flag to retrieve. Must be a value from the
 *             Interrupt_Flags enumeration.
 * @return true if the flag is set, false otherwise.
 */
bool EXTI::get_interrupt_flag(Interrupt_Flags flag) {
    if (flag == Interrupt_Flags::INVALID) {
        return false;
    }
    return read_bit(*this, EXTI_Regs::PD, static_cast<uint32_t>(flag));
}

/**
 * @brief Clears the specified interrupt flag.
 *
 * This function clears the specified interrupt flag by writing to the
 * EXTI status register (PD). If the flag is not applicable to the EXTI
 * peripheral, this function does nothing.
 *
 * @param flag The interrupt flag to clear. Must be a value from the
 *             Interrupt_Flags enumeration.
 */
void EXTI::clear_interrupt_flag(Interrupt_Flags flag) {
    if (flag == Interrupt_Flags::INVALID) {
        return;
    }
    write_bit(*this, EXTI_Regs::PD, static_cast<uint32_t>(flag), true);
}

/**
 * @brief Enables or disables the EXTI line software interrupt.
 *
 * This function sets or clears the bit in the SWIEV register that corresponds
 * to the given EXTI line. If the `enable` parameter is true, the EXTI line
 * software interrupt is enabled, and if the `enable` parameter is false, the
 * EXTI line software interrupt is disabled.
 *
 * @param line The EXTI line to enable or disable the software interrupt for.
 * @param enable Set to true to enable the software interrupt, false to disable it.
 */
void EXTI::set_software_interrupt_enable(EXTI_Line line, bool enable) {
    if (line == EXTI_Line::INVALID) {
        return;
    }
    write_bit(*this, EXTI_Regs::SWIEV, static_cast<uint32_t>(line), enable);
}

/**
 * @brief Enables or disables the specified interrupt type.
 *
 * This function sets or clears the bit in the INTEN register that corresponds
 * to the given interrupt type. If the `enable` parameter is true, the interrupt
 * is enabled, and if the `enable` parameter is false, the interrupt is disabled.
 *
 * @param type The interrupt type to enable or disable. Must be a value from the
 *             Interrupt_Type enumeration.
 * @param enable Set to true to enable the interrupt, false to disable it.
 */
void EXTI::set_interrupt_enable(Interrupt_Type type, bool enable) {
    if (type == Interrupt_Type::INVALID) {
        return;
    }
    write_bit(*this, EXTI_Regs::INTEN, static_cast<uint32_t>(type), enable);
}


} // namespace exti
