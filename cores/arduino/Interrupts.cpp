/*
  Copyright (c) 2025 Arduino. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <stddef.h>
#include <utility>
#include <array>

#include "Arduino.h"
#include "ExtiManager.h"

/**
 * @brief Wraps a callback function with a parameter for ExtiManager.
 *
 * This function takes a void pointer as a parameter, which is expected to be
 * a pointer to a callback function with a void return type and one parameter.
 * The function will call the callback function with the parameter.
 *
 * This wrapper function is used to integrate the ExtiManager library with the
 * attachInterrupt function, which requires a callback function with a void
 * return type and one parameter.
 */
void callbackWrapper(void* param) {
    auto func = reinterpret_cast<voidFuncPtr>(param);
    if (func) {
        func();
    }
}

/**
 * @brief Attach an interrupt callback function to a specific pin.
 *
 * This function enables an external interrupt on a specific pin. It
 * configures the pin for input and sets the EXTI source in the AFIO.
 * The EXTI interrupt is enabled and the NVIC priority is set.
 *
 * @param pin The pin for which to enable the EXTI interrupt
 * @param callback The callback function to be called when the interrupt occurs
 * @param mode The type of EXTI interrupt to enable (RISING, FALLING, or CHANGE)
 * @param param The parameter to be passed to the callback function
 *
 * @note This function configures the pin to be an input pin and sets the EXTI
 * source in the AFIO. The EXTI interrupt is enabled and the NVIC priority
 * is set.
 */
void attachInterruptParam(pin_size_t pin, voidFuncPtrParam callback, PinStatus mode, void* param) {
    if (pin >= TOTAL_PIN_COUNT) return;
    const PortPinPair& pp = port_pin_map[pin];
    if (pp.pin == gpio::Pin_Number::INVALID || pp.port == gpio::GPIO_Base::INVALID) {
        return;
    }

    auto slotIndex = static_cast<uint8_t>(pp.pin);
    if (slotIndex >= maxExtiLines_) {
        return;
    }

    exti::EXTI_Trigger type = exti::EXTI_Trigger::TRIG_NONE;
    switch (mode) {
        case RISING: case HIGH: type = exti::EXTI_Trigger::TRIG_RISING; break;
        case FALLING: case LOW: type = exti::EXTI_Trigger::TRIG_FALLING; break;
        case CHANGE: type = exti::EXTI_Trigger::TRIG_BOTH; break;
        default: break;
    }

    freeDebugPins(pin);
    ExtiManager::get_instance().enablePinExtiInterrupt(pin, callback, param, type);
}

/**
 * @brief Attach an interrupt callback to the specified pin.
 *
 * Attaches an interrupt callback to the specified pin. The callback will be
 * called when the pin's value changes according to the specified mode.
 *
 * @param pin The pin to attach the interrupt to.
 * @param callback The callback function to call when the interrupt occurs.
 * @param mode The mode of the interrupt. Can be RISING, FALLING, CHANGE, LOW, or HIGH.
 */
void attachInterrupt(pin_size_t pin, voidFuncPtr callback, PinStatus mode) {
    attachInterruptParam(pin, callbackWrapper, mode, reinterpret_cast<void*>(callback));
}

/**
 * @brief Detaches an interrupt callback from the specified pin.
 *
 * Detaches an interrupt callback from the specified pin. The callback will no
 * longer be called when the pin's value changes.
 *
 * @param pin The pin to detach the interrupt from.
 */
void detachInterrupt(pin_size_t pin) {
    if (pin >= TOTAL_PIN_COUNT) return;
    const PortPinPair& pp = port_pin_map[pin];
    if (pp.pin == gpio::Pin_Number::INVALID || pp.port == gpio::GPIO_Base::INVALID) {
        return;
    }

    auto slotId = static_cast<uint8_t>(pp.pin);
    if (slotId >= maxExtiLines_) {
        return;
    }

    ExtiManager::get_instance().disablePinExtiInterrupt(pin);
}
