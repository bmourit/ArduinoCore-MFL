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
#include "GeneralTimer.h"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"
#include "variant.h"

inline constexpr uint32_t Max_Frequency = 0x0000FFFFU;

struct TonePinInfo {
    pin_size_t pin;
    int32_t count;
    gpio::GPIO_Base port;
    gpio::Pin_Number pinNum;
};

static TonePinInfo pinInfo = { NO_PIN, 0, gpio::GPIO_Base::INVALID, gpio::Pin_Number::INVALID };
volatile bool destruct_ = false;

static void timerTonePinInit(pin_size_t pin, uint32_t frequency, uint32_t duration);
static void toneHandler();

static GeneralTimer& toneInstance = GeneralTimer::get_instance(static_cast<timer::TIMER_Base>(TIMER_TONE));

/**
 * @brief Initializes the timer for generating a tone on a specified pin.
 *
 * Configures the timer based on the given pin, frequency, and duration.
 * If the frequency is zero, the timer is stopped. If the frequency exceeds
 * the maximum allowed value, the pin operations are set up, and the timer
 * is started with the specified settings.
 *
 * @param pin        The pin number to output the tone. Use NO_PIN to disable.
 * @param frequency  The frequency of the tone in Hertz.
 * @param duration   The duration of the tone in milliseconds. A duration of zero
 *                   indicates an indefinite tone.
 */
static void timerTonePinInit(pin_size_t pin, uint32_t frequency, uint32_t duration) {
    if (pin == NO_PIN) return;

    uint32_t toneFreq = frequency * 2;

    if (frequency == 0) {
        toneInstance.stop();
    } else if (frequency <= Max_Frequency) {
        pinInfo.pin = pin;
        pinInfo.count = (duration > 0) ? ((toneFreq * duration) / 1000) : -1;
        pinInfo.port = getPortFromPin(pinInfo.pin);
        pinInfo.pinNum = getPinInPort(pinInfo.pin);
        if (pinInfo.port == gpio::GPIO_Base::INVALID || pinInfo.pinNum == gpio::Pin_Number::INVALID) {
            return;
        }

        setPinOp(pinInfo.pin, createPackedPinOps(
                     gpio::Pin_Mode::OUTPUT_PUSHPULL,
                     MAX_SPEED,
                     gpio::Pin_Remap_Select::NO_REMAP,
                     0U,
                     0U
                 ));
        toneInstance.setChannelMode(0U, InputOutputMode::TIMING, NO_PIN);
        toneInstance.setRolloverValue(toneFreq, TimerFormat::HERTZ);
        toneInstance.attachInterrupt(toneHandler);
        toneInstance.start();
    }
}

/**
 * @brief Stops the Tone timer and resets the associated pin configuration.
 *
 * This function deinitializes the Tone timer instance and sets the pin to input floating mode
 * if a valid pin is configured.
 */
static void timerTonePinDeinit() {
    toneInstance.stop();

    if (pinInfo.pin != NO_PIN && pinInfo.pin <= MAX_PIN_NUM) {
        setPinOp(pinInfo.pin, createPackedPinOps(
                     gpio::Pin_Mode::INPUT_FLOATING,
                     MAX_SPEED,
                     gpio::Pin_Remap_Select::NO_REMAP,
                     0,
                     0
                 ));
        pinInfo.pin = NO_PIN;
    }
}

/**
 * @brief Generates a tone on the specified pin with the given frequency and duration.
 *
 * @param pin The pin number where the tone will be generated.
 * @param frequency The frequency of the tone in Hertz.
 * @param duration The duration of the tone in milliseconds.
 */
void tone(uint8_t pin, unsigned int frequency, unsigned long duration) {
    if ((pin != NO_PIN) && ((pinInfo.pin == NO_PIN) || (pinInfo.pin == pin))) {
        timerTonePinInit(static_cast<pin_size_t>(pin), static_cast<uint32_t>(frequency), static_cast<uint32_t>(duration));
    }
}

/**
 * @brief Stops tone generation on the specified pin.
 *
 * If global variable `destruct_` is true, deinitializes the timer resources;
 * otherwise, stops the ongoing tone.
 *
 * @param pin The pin on which to stop the tone.
 */
void noTone(uint8_t pin) {
    if (pin == NO_PIN || pinInfo.pin != pin) return;

    if (destruct_) {
        timerTonePinDeinit();
    } else {
        toneInstance.stop();
    }
}

/**
 * @brief Handles the tone period elapsed event by toggling the GPIO pin.
 *
 * Decrements the pin's count and turns off the pin when the count reaches zero.
 */
static void toneHandler() {
    if (pinInfo.port == gpio::GPIO_Base::INVALID || pinInfo.pinNum == gpio::Pin_Number::INVALID) {
        return;
    }

    auto result = gpio::GPIO::get_instance(pinInfo.port);
    if (result.error() != gpio::GPIO_Error_Type::OK) {
        return;
    }
    auto& instance = result.value();

    if (pinInfo.count != 0) {
        if (pinInfo.count > 0) {
            pinInfo.count--;
        }
        instance.toggle_pin(pinInfo.pinNum);
    } else {
        instance.write_pin(pinInfo.pinNum, false);
    }
}
