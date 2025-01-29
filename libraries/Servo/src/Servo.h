/*
  Servo.h - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
  Copyright (c) 2009 Michael Margolis.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
 * This header has been adapted from the standard Arduino Servo library
 * at https://github.com/arduino-libraries/Servo for MFL C++ library
 * Copyright (c) 2025 B. Mouritsen. All rights reserved.
 */

#pragma once

#include <inttypes.h>

enum class Timers16Bit : uint8_t {
    timerIndex,
    timerCount,
};

struct ServoPin {
    uint8_t pin;
    uint8_t isActive;
};

struct ServoConfig {
    ServoPin pinNumber;
    volatile unsigned int ticks;
};

class Servo {
public:
    Servo();

    uint8_t attach(int pin);                    // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
    uint8_t attach(int pin, int min, int max);  // as above but also sets min and max values for writes.
    void detach();
    void write(int value);                      // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds
    void writeMicroseconds(int value);          // Write pulse width in microseconds
    int read();                                 // returns current pulse width as an angle between 0 and 180 degrees
    int readMicroseconds();                     // returns current pulse width in microseconds for this servo (was read_us() in first release)
    bool attached();                            // return true if this servo is attached, otherwise false

private:
    uint8_t servoIndex;                         // index into the channel data for this servo
    int8_t min;                                 // minimum is this value times 4 added to MIN_PULSE_WIDTH
    int8_t max;                                 // maximum is this value times 4 added to MAX_PULSE_WIDTH
};
