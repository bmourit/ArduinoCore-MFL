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

/*
 * This file has been adapted from the standard Arduino Servo library
 * at https://github.com/arduino-libraries/Servo for MFL C++ library
 * Copyright (c) 2025 B. Mouritsen. All rights reserved.
 */

#include <Arduino.h>
#include <GeneralTimer.h>
#include "Servo.h"

static ServoConfig servos[MAX_SERVOS];
static volatile int8_t timerChannel[static_cast<uint8_t>(Timers16Bit::timerCount)] = {-1};  // counter for the servo being pulsed for each timer (or -1 if refresh interval)

static auto& servoTimer = GeneralTimer::get_instance(static_cast<timer::TIMER_Base>(TIMER_SERVO));

uint8_t ServoCount = 0U;    // the total number of attached servos

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)   // minimum value in uS for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)   // maximum value in uS for this servo

#define SERVO_TIMER(timerId_)   (static_cast<Timers16Bit>(timerId_))

volatile uint32_t TotalCount = 0;

static void Servo_PeriodElapsedCallback() {
    constexpr Timers16Bit timerId = Timers16Bit::timer1;
    const uint8_t idx = static_cast<uint8_t>(timerId);
    int8_t channel = timerChannel[idx];

    if (channel < 0) {
        TotalCount = 0;
    } else if (channel < ServoCount && servos[channel].pinNumber.isActive == true) {
        digitalWrite(servos[channel].pinNumber.pin, LOW);   // Pulse LOW if active
    }

    // Increment to the next channel
    ++channel;
    timerChannel[idx] = channel;

    if (channel < ServoCount && channel < SERVOS_PER_TIMER) {
        const auto& servo = servos[channel];
        servoTimer.setRolloverValue(servo.ticks, TimerFormat::TICK);
        TotalCount = TotalCount + servo.ticks;

        if (servo.pinNumber.isActive) {
            digitalWrite(servo.pinNumber.pin, HIGH);    // Pulse HIGH if active
        }
    } else {
        // Finished all channels so wait for the refresh period to expire before starting over
        if (TotalCount + 4U < REFRESH_INTERVAL) {
            servoTimer.setRolloverValue(REFRESH_INTERVAL - TotalCount, TimerFormat::TICK);
        } else {
            servoTimer.refresh();   // Start over immediately
        }
        timerChannel[idx] = -1;     // Prepare for restart
    }
}

static void TimerServoInit() {
    servoTimer.setChannelMode(1, InputOutputMode::TIMING, INVALID_PIN);
    servoTimer.setPrescaler(static_cast<uint16_t>(servoTimer.getTimerClockFrequency() / 1'000'000U));
    servoTimer.setRolloverValue(REFRESH_INTERVAL, TimerFormat::TICK);
    servoTimer.attachInterrupt(Servo_PeriodElapsedCallback);
    servoTimer.setAutoReloadEnable(false);
    servoTimer.start();
}

// Check active status
static bool isTimerActive() {
    for (uint8_t channel = 0U; channel < SERVOS_PER_TIMER; channel++) {
        if (servos[channel].pinNumber.isActive) {
            return true;
        }
    }
    return false;
}

Servo::Servo() :
    servoIndex(0),
    min(0),
    max(0)
{
    if (ServoCount < MAX_SERVOS) {
        this->servoIndex = ServoCount++;                      // Assign a servo index to this instance
        servos[this->servoIndex].ticks = DEFAULT_PULSE_WIDTH; // Store default values
    } else {
        this->servoIndex = INVALID_SERVO;                     // Too many servos
    }
}

uint8_t Servo::attach(int pin) {
    return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t Servo::attach(int pin, int min, int max) {
    if (this->servoIndex < MAX_SERVOS) {
        pinMode(static_cast<pin_size_t>(pin), OUTPUT);
        servos[this->servoIndex].pinNumber.pin = pin;
        servos[this->servoIndex].ticks = DEFAULT_PULSE_WIDTH;

        // TODO: min/max check: abs(min - MIN_PULSE_WIDTH) / 4 < 128
        this->min = (MIN_PULSE_WIDTH - min) / 4;    // Resolution of min/max is 4 uS
        this->max = (MAX_PULSE_WIDTH - max) / 4;

        // Initialize the timer if it has not already been initialized
        if (isTimerActive() == false) {
            TimerServoInit();
        }
        servos[this->servoIndex].pinNumber.isActive = true; // Must be set after the check for isTimerActive
    }

    return this->servoIndex;
}

void Servo::detach() {
    servos[this->servoIndex].pinNumber.isActive = false;

    if (isTimerActive() == false) {
        servoTimer.stop();
    }
}

void Servo::write(int value) {
    // Treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
    if (value < MIN_PULSE_WIDTH) {
        if (value < 0) {
            value = 0;
        } else if (value > 180) {
            value = 180;
        }
        value = map(value, 0, 180, SERVO_MIN(), SERVO_MAX());
    }
    writeMicroseconds(value);
}

void Servo::writeMicroseconds(int value) {
    // Calculate and store the values for the given channel
    byte channel = this->servoIndex;
    if ((channel < MAX_SERVOS)) {   // Ensure channel is valid
        if (value < SERVO_MIN()) {  // Ensure pulse width is valid
            value = SERVO_MIN();
        } else if (value > SERVO_MAX()) {
            value = SERVO_MAX();
        }
        servos[channel].ticks = value;
    }
}

int Servo::read() {
    return map(readMicroseconds() + 1, SERVO_MIN(), SERVO_MAX(), 0, 180);
}

int Servo::readMicroseconds() {
    unsigned int pulsewidth;
    if (this->servoIndex != INVALID_SERVO) {
        pulsewidth = servos[this->servoIndex].ticks;
    } else {
        pulsewidth = 0;
    }

    return pulsewidth;
}

bool Servo::attached() {
    return servos[this->servoIndex].pinNumber.isActive;
}
