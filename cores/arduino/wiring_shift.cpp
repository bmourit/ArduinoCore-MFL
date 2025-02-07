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

/**
 * @brief Shifts in a byte of data from the given pin.
 *
 * This function shifts in a byte of data from the given pin. The data is
 * shifted in in the specified bit order. The clock pin is toggled
 * on each iteration of the loop.
 *
 * @param dataPin The pin to read data from
 * @param clockPin The pin to use as the clock
 * @param bitOrder The order of the bits in the data
 * @return The byte of data shifted in
 */
uint8_t shiftIn(pin_size_t dataPin, pin_size_t clockPin, BitOrder bitOrder) {
    uint8_t value = 0U;
    const bool isLSBFIRST = (bitOrder == LSBFIRST);
    uint8_t shiftIndex = isLSBFIRST ? 0U : 7U;
    const int8_t increment = isLSBFIRST ? 1 : -1;

    for (uint8_t i = 0U; i < 8U; i++) {
        digitalWrite(clockPin, HIGH);
        value |= digitalRead(dataPin) << shiftIndex;
        shiftIndex += increment;
        digitalWrite(clockPin, LOW);
    }

    return value;
}

/**
 * @brief Shifts out a byte of data to the given pin.
 *
 * This function shifts out a byte of data to the given pin. The data is
 * shifted out in the specified bit order. The clock pin is toggled
 * on each iteration of the loop.
 *
 * @param dataPin The pin to write data to
 * @param clockPin The pin to use as the clock
 * @param bitOrder The order of the bits in the data
 * @param val The byte of data to shift out
 */
void shiftOut(pin_size_t dataPin, pin_size_t clockPin, BitOrder bitOrder, uint8_t val) {
    const bool isLSBFIRST = (bitOrder == LSBFIRST);
    uint8_t shiftIndex = isLSBFIRST ? 0U : 7U;
    const int8_t increment = isLSBFIRST ? 1 : -1;

    for (uint8_t i = 0U; i < 8U; i++) {
        digitalWrite(dataPin, !!(val & (1U << shiftIndex)));
        digitalWrite(clockPin, HIGH);
        digitalWrite(clockPin, LOW);
        shiftIndex += increment;
    }
}
