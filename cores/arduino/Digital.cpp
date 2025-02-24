#include "Arduino.h"
#include "Analog.h"
#include "PinConfigManager.hpp"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"

/**
 * @brief Configures the specified pin as an input or output.
 *
 * This function takes a pin number and a pin mode as arguments and configures
 * the specified pin accordingly. The pin mode can be one of the following:
 * INPUT, INPUT_PULLUP, INPUT_PULLDOWN, OUTPUT, OUTPUT_OPENDRAIN, or
 * INPUT_ANALOG. If the pin is not configured, the function does not take any
 * action.
 *
 * If the pin is currently configured as a PWM pin, the function stops the PWM
 * output on that pin before configuring it as an input or output.
 *
 * @param pin The pin number to configure.
 * @param mode The pin mode to set. Valid values are PinMode::INPUT,
 *             PinMode::INPUT_PULLUP, PinMode::INPUT_PULLDOWN,
 *             PinMode::OUTPUT, PinMode::OUTPUT_OPENDRAIN, or
 *             PinMode::INPUT_ANALOG.
 */
void pinMode(pin_size_t pin, PinMode mode) {
    if (pin == NO_PIN) {
        core_debug("Selected pin is not a valid pin number");
        return;
    }

    if (pinConfigManager.isPinConfigured(pin)) {
        if (isPinInPinOps(TIMER_PinOps, pin)) {
            pwmStop(pin);
        }
        pinConfigManager.resetPinConfigured(pin);
    }

    switch (mode) {
        case INPUT_PULLUP:
            setPinOp(pin, createPackedPinOps(gpio::Pin_Mode::INPUT_PULLUP, gpio::Output_Speed::SPEED_MAX, gpio::Pin_Remap_Select::NO_REMAP, 0, 0));
            break;
        case INPUT_PULLDOWN:
            setPinOp(pin, createPackedPinOps(gpio::Pin_Mode::INPUT_PULLDOWN, gpio::Output_Speed::SPEED_MAX, gpio::Pin_Remap_Select::NO_REMAP, 0, 0));
            break;
        case OUTPUT:
            setPinOp(pin, createPackedPinOps(gpio::Pin_Mode::OUTPUT_PUSHPULL, gpio::Output_Speed::SPEED_MAX, gpio::Pin_Remap_Select::NO_REMAP, 0, 0));
            break;
        case OUTPUT_OPENDRAIN:
            setPinOp(pin, createPackedPinOps(gpio::Pin_Mode::OUTPUT_OPENDRAIN, gpio::Output_Speed::SPEED_MAX, gpio::Pin_Remap_Select::NO_REMAP, 0, 0));
            break;
        // NOTE: INPUT_ANALOG is deprecated.
        // Besides not being part of the Arduino API,
        // it is already handled directly in Analog.cpp
#pragma GCC diagnostic ignored "-Wswitch"
        case INPUT_ANALOG:
            if ((pin != ADC_TEMP) && (pin != ADC_VREF)) {
                pinOpsPinout(ADC_PinOps, pin);
            }
            break;
        case INPUT:
        default:    // Default to INPUT
            setPinOp(pin, createPackedPinOps(gpio::Pin_Mode::INPUT_FLOATING, gpio::Output_Speed::SPEED_MAX, gpio::Pin_Remap_Select::NO_REMAP, 0, 0));
            break;
    }
}

/**
 * @brief Writes a digital value to a pin.
 *
 * This function takes a pin number and a PinStatus as arguments and writes the
 * specified digital value to the pin. The PinStatus can be either LOW (0) or
 * HIGH (1). If the pin is not configured as an output, the function does not
 * take any action.
 *
 * @param pin The pin number to write to.
 * @param status The PinStatus to write to the pin. Valid values are LOW (0) or HIGH (1).
 */
void digitalWrite(pin_size_t pin, PinStatus status) {
    gpio::GPIO_Base port = getPortFromPin(pin);
    if (port == gpio::GPIO_Base::INVALID) {
        return;
    }
    auto result = gpio::GPIO::get_instance(port);
    if (result.error() != gpio::GPIO_Error_Type::OK) {
        return;
    }
    auto& instance = result.value();
    gpio::Pin_Number pinNum = getPinInPort(pin);
    if (pinNum == gpio::Pin_Number::INVALID) {
        return;
    }
    instance.write_pin(pinNum, (status != LOW));
}

/**
 * @brief Reads the current state of a digital GPIO pin.
 *
 * This function takes a pin number and returns the current state of the pin
 * as a PinStatus. The PinStatus can be either LOW (0) or HIGH (1). If the pin
 * is not configured as a digital input, the function returns LOW.
 *
 * @param pin The pin number to read from.
 * @return The current state of the pin as a PinStatus.
 */
PinStatus digitalRead(pin_size_t pin) {
    auto result = gpio::GPIO::get_instance(getPortFromPin(pin));
    auto& instance = result.value();
    return instance.read_pin(getPinInPort(pin)) ? HIGH : LOW;
}

/**
 * @brief Toggles the specified digital GPIO pin.
 *
 * This function takes a pin number as an argument and toggles the specified
 * digital GPIO pin. If the pin is not configured as a digital output, the
 * function does not take any action.
 *
 * @param pin The pin number to toggle.
 */
void digitalToggle(pin_size_t pin) {
    gpio::GPIO_Base port = getPortFromPin(pin);
    if (port == gpio::GPIO_Base::INVALID) {
        return;
    }
    auto result = gpio::GPIO::get_instance(port);
    if (result.error() != gpio::GPIO_Error_Type::OK) {
        return;
    }
    gpio::Pin_Number pinNum = getPinInPort(pin);
    if (pinNum == gpio::Pin_Number::INVALID) {
        return;
    }

    auto& instance = result.value();
    instance.toggle_pin(pinNum);
}
