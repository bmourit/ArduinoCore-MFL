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
#include <utility>
#include "Servo.h"

static ServoConfig servos[MAX_SERVOS];
static volatile int8_t timerChannel[static_cast<uint8_t>(Timers16Bit::timerCount)] = {-1};  // counter for the servo being pulsed for each timer (or -1 if refresh interval)

static auto& servoTimer = GeneralTimer::get_instance(static_cast<timer::TIMER_Base>(TIMER_SERVO));

uint8_t ServoCount = 0U;    // the total number of attached servos

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)   // minimum value in uS for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)   // maximum value in uS for this servo

#define SERVO_TIMER(timerId_)   (static_cast<Timers16Bit>(timerId_))

volatile uint32_t TotalCount = 0;

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

/**
 * @brief Callback function triggered when the timer period elapses for servo control.
 *
 * This function manages the pulsing of servos by iterating over active servo channels.
 * It pulses the corresponding pin to LOW or HIGH depending on the channel's activity, 
 * updates the rollover value for the timer, and manages the refresh interval once all 
 * channels have been processed. If there are no active channels, it resets the TotalCount.
 */
static void Servo_PeriodElapsedCallback() {
    constexpr Timers16Bit timerId = Timers16Bit::timer1;
    const auto idx = static_cast<uint8_t>(timerId);
    int8_t channel = timerChannel[idx];

    if (channel < 0) {
        TotalCount = 0;
    } else if (std::cmp_less(channel , ServoCount) && servos[channel].pinNumber.isActive == true) {
        digitalWrite(servos[channel].pinNumber.pin, LOW);   // Pulse LOW if active
    }

    // Increment to the next channel
    ++channel;
    timerChannel[idx] = channel;

    if (std::cmp_less(channel , ServoCount) && std::cmp_less(channel , SERVOS_PER_TIMER)) {
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

/**
 * @brief Initializes the servo timer.
 *
 * This function sets the channel mode to timing mode, sets the prescaler to the
 * timer clock frequency divided by 1,000,000, sets the rollover value to the
 * refresh interval, attaches the Servo_PeriodElapsedCallback to the timer
 * interrupt, and starts the timer. The autoreload enable is disabled.
 */
static void TimerServoInit() {
    servoTimer.setChannelMode(1, InputOutputMode::TIMING, INVALID_PIN_NUMBER);
    servoTimer.setPrescaler(static_cast<uint16_t>(servoTimer.getTimerClockFrequency() / 1'000'000U));
    servoTimer.setRolloverValue(REFRESH_INTERVAL, TimerFormat::TICK);
    servoTimer.attachInterrupt(Servo_PeriodElapsedCallback);
    servoTimer.setAutoReloadEnable(false);
    servoTimer.start();
}

/**
 * @brief Returns true if any servos are currently active, false otherwise.
 *
 * This function checks the isActive member of each servo's pinNumber
 * and returns true if any are currently active, false otherwise.
 */
static auto isTimerActive() -> bool {
    for (auto & servo : servos) {
        if (servo.pinNumber.isActive) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Attaches the given pin to the next free channel, sets pinMode, and initializes the timer if it has not already been initialized.
 *
 * This function is equivalent to calling attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH).
 *
 * @param pin The pin to use for the servo
 * @return The servo index of the attached servo, or 255 if no free channels were available.
 */
auto Servo::attach(int pin) -> uint8_t {
    return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

/**
 * @brief Attaches the given pin to the next free channel, sets pinMode, and initializes the timer if it has not already been initialized.
 * @param pin The pin to use for the servo
 * @param min The minimum pulse width for the given pin
 * @param max The maximum pulse width for the given pin
 * @return The servo index of the attached servo, or 255 if no free channels were available.
 */
auto Servo::attach(int pin, int min, int max) -> uint8_t {
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

/**
 * @brief Detaches the servo from the pin it is currently attached to.
 *
 * This function sets the isActive flag for the servo to false and stops the timer
 * if no other servos are currently attached.
 */
void Servo::detach() {
    servos[this->servoIndex].pinNumber.isActive = false;

    if (isTimerActive() == false) {
        servoTimer.stop();
    }
}

/**
 * @brief Writes an angle or pulse width to the servo.
 *
 * This function interprets the given value as an angle in degrees if it is less
 * than 544; otherwise, it is treated as a pulse width in microseconds. For angles
 * less than 0 or greater than 180, the value is clamped to 0 and 180, respectively.
 * The angle is then mapped to a corresponding pulse width between SERVO_MIN() and
 * SERVO_MAX(). The resulting pulse width is written to the servo using the
 * writeMicroseconds function.
 *
 * @param value The angle in degrees (0-180) or pulse width in microseconds to write.
 */
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

/**
 * @brief Writes a pulse width in microseconds to the servo.
 *
 * This function sets the pulse width for the servo corresponding to the
 * current instance by updating the `ticks` member of the servo object.
 * The pulse width is clamped between `SERVO_MIN()` and `SERVO_MAX()`
 * to ensure it falls within the acceptable range for the servo.
 *
 * @param value The pulse width in microseconds to write to the servo.
 */
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

/**
 * @brief Returns the current angle of the servo.
 *
 * This function maps the current pulse width, obtained from readMicroseconds(),
 * to an angle between 0 and 180 degrees, inclusive. It adjusts the pulse width
 * by adding 1 to ensure proper mapping within the specified range.
 *
 * @return The current angle of the servo in degrees.
 */
auto Servo::read() -> int {
    return map(readMicroseconds() + 1, SERVO_MIN(), SERVO_MAX(), 0, 180);
}

/**
 * @brief Reads the current pulse width set on the servo.
 *
 * This function reads the last set pulse width value from the servo's ticks member
 * and returns it. If the servo is not attached, it returns 0.
 *
 * @return The current pulse width set on the servo in microseconds.
 */
auto Servo::readMicroseconds() -> int {
    unsigned int pulsewidth;
    if (this->servoIndex != INVALID_SERVO) {
        pulsewidth = servos[this->servoIndex].ticks;
    } else {
        pulsewidth = 0;
    }

    return pulsewidth;
}

/**
 * @brief Returns true if the servo is currently attached to a pin, false otherwise.
 *
 * This function checks the isActive member of the servo's pinNumber and returns true
 * if it is currently attached, false otherwise.
 */
auto Servo::attached() -> bool {
    return servos[this->servoIndex].pinNumber.isActive;
}
