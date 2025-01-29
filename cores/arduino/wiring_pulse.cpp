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

#include "Arduino.h"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"

/**
 * @brief Measures the length (in microseconds) of a pulse on the pin; state is
 * HIGH or LOW, the type of pulse to measure.  Works on pulses from 2-3
 * microseconds to 3 minutes in length, but must be called at least a few
 * dozen microseconds before the start of the pulse.
 *
 * @param pin The pin number where the pulse is present
 * @param state The type of pulse to measure. State is HIGH or LOW
 * @param timeout The maximum time to wait for the pulse in microseconds
 *
 * @return The length of the pulse in microseconds or 0 if no complete pulse
 * could be read within the given time out.
 *
 * ATTENTION:
 * This function relies on micros() so cannot be used in noInterrupt() context
 */
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout) {
    // Cache the port and pinInPort of the pin in order to speed up the
    // pulse width measuring loop and achieve finer resolution.
    // Calling digitalRead() instead yields much coarser resolution.
    gpio::GPIO_Base port = getPortFromPin(static_cast<pin_size_t>(pin));
    gpio::Pin_Number pinInPort = getPinInPort(static_cast<pin_size_t>(pin));
    if (port == gpio::GPIO_Base::INVALID || pinInPort == gpio::Pin_Number::INVALID) {
        return 0;
    }

    bool level = (state == HIGH) ? true : false;
    uint32_t startMicros = micros();

    // Wait for any previous pulse to end
    while (gpio::fast_read_pin(port, pinInPort) == level) {
        if ((micros() - startMicros) > timeout) {
            return 0;
        }
    }

    // Wait for the pulse to start
    while (gpio::fast_read_pin(port, pinInPort) != level) {
        if ((micros() - startMicros) > timeout) {
            return 0;
        }
    }
    uint32_t start = micros();

    // Wait for the pulse to stop
    while (gpio::fast_read_pin(port, pinInPort) == level) {
        if ((micros() - startMicros) > timeout) {
            return 0;
        }
    }

    return (micros() - start);
}

/**
 * @brief Identical to pulseIn() but returns unsigned long instead of unsigned int.
 *
 * @param pin The pin number where the pulse is present
 * @param state The type of pulse to measure. State is HIGH or LOW
 * @param timeout The maximum time to wait for the pulse in microseconds
 *
 * @return The length of the pulse in microseconds or 0 if no complete pulse
 * could be read within the given time out.
 *
 * ATTENTION:
 * This function relies on micros() so cannot be used in noInterrupt() context
 */
unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout) {
    return pulseIn(pin, state, timeout);
}
