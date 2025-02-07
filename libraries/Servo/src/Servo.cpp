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
#include <Servo.h>
#include "GeneralTimer.h"

constexpr int MIN_PULSE_WIDTH = 544;
constexpr int MAX_PULSE_WIDTH = 2400;
constexpr int DEFAULT_PULSE_WIDTH = 1500;

constexpr uint8_t SERVO_VERSION = 2U;
constexpr uint8_t SERVOS_PER_TIMER = 12U;
constexpr uint16_t INVALID_SERVO = 255U;
constexpr uint32_t REFRESH_INTERVAL = 20000U;
constexpr uint16_t MAX_SERVOS = (static_cast<uint16_t>(Timers16Bit::timerCount) * SERVOS_PER_TIMER);

static ServoConfig servos[MAX_SERVOS];
static volatile int8_t timerChannel[static_cast<uint8_t>(Timers16Bit::timerCount)] = {-1};  // counter for the servo being pulsed for each timer (or -1 if refresh interval)

timer::TIMER_Base timer_base = static_cast<timer::TIMER_Base>(TIMER_SERVO);
auto& servoTimer = GeneralTimer::get_instance(timer_base);

uint8_t ServoCount = 0U;    // the total number of attached servos

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)   // minimum value in uS for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)   // maximum value in uS for this servo

#define SERVO_TIMER(timerId_)   (static_cast<Timers16Bit>(timerId_))

volatile uint32_t TotalCount = 0;

static void Servo_PeriodElapsedCallback() {
    uint8_t timerId = static_cast<uint8_t>(Timers16Bit::timerIndex);

    if (timerChannel[timerId] < 0) {
        TotalCount = 0U;
    } else {
        if (timerChannel[timerId] < ServoCount && servos[timerChannel[timerId]].pinNumber.isActive == true) {
            digitalWrite(servos[timerChannel[timerId]].pinNumber.pin, LOW);  // Pulse this channel low if activated
        }
    }

    // Increment to the next channel
    timerChannel[timerId] = timerChannel[timerId] + 1;
    if (timerChannel[timerId] < ServoCount && timerChannel[timerId] < SERVOS_PER_TIMER) {
        servoTimer.setRolloverValue(servos[timerChannel[timerId]].ticks, TimerFormat::TICK);
        TotalCount = TotalCount + servos[timerChannel[timerId]].ticks;
        if (servos[timerChannel[timerId]].pinNumber.isActive == true) {
            digitalWrite(servos[timerChannel[timerId]].pinNumber.pin, HIGH);   // It is an active channel so pulse it high
        }
    } else {
        // Finished all channels so wait for the refresh period to expire before starting over
        if (TotalCount + 4U < REFRESH_INTERVAL) {
            // Allow a few ticks to ensure the next OCR1A not missed
            servoTimer.setRolloverValue(REFRESH_INTERVAL - TotalCount, TimerFormat::TICK);
        } else {
            // Generate update to restart immediately from the beginning with the 1st servo
            servoTimer.refresh();
        }
        timerChannel[timerId] = -1; // This will get incremented at the end of the refresh period to start again at the first channel
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
        if (servos[channel].pinNumber.isActive == true) {
            return true;
        }
    }

    return false;
}

Servo::Servo() : servoIndex(0), min(0), max(0) {
    if (ServoCount < MAX_SERVOS) {
        this->servoIndex = ServoCount++;                      // assign a servo index to this instance
        servos[this->servoIndex].ticks = DEFAULT_PULSE_WIDTH; // store default values
    } else {
        this->servoIndex = INVALID_SERVO;                     // too many servos
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
