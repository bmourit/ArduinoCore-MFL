#include "Arduino.h"
#include "Analog.h"
#include "PinConfigManager.hpp"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"
#include "variant.h"

/**
 * @brief Configures the specified pin as an input or output.
 *
 * This function takes a pin number and a pin mode as arguments and configures
 * the specified pin accordingly. The pin mode can be one of the following:
 * INPUT, INPUT_PULLUP, INPUT_PULLDOWN, OUTPUT, OUTPUT_OPENDRAIN, or
 * INPUT_ANALOG. If the pin is not configured, the function does not take any
 * action.
 *
 * If the pin is currently configured as a PWM or DAC pin, the function stops the
 * output on that pin before configuring it as an input or output.
 *
 * @param pin The pin number to configure.
 * @param mode The pin mode to set. Valid values are INPUT,
 *             INPUT_PULLUP, INPUT_PULLDOWN,
 *             OUTPUT, or OUTPUT_OPENDRAIN.
 *             INPUT_ANALOG is deprecated and will be removed in a future release.
 */
void pinMode(pin_size_t pin, PinMode mode) {
    // Check for invalid pin
    if (pin == NO_PIN) {
        core_debug("Selected pin is not a valid pin number");
        return;
    }

    // Reset pin configuration if already configured
    if (pinConfigManager.isPinConfigured(pin)) {
        // Stop PWM or DAC if active on this pin
        if (isPinInPinOps(TIMER_PinOps, pin)) {
            pwmStop(pin);
        } else if (isPinInPinOps(DAC_PinOps, pin)) {
            dacStop(pin);
        }
        pinConfigManager.resetPinConfigured(pin);
    }

    // Configure pin based on mode
    switch (mode) {
        case INPUT:
            setPinOp(pin, createPackedPinOps(gpio::Pin_Mode::INPUT_FLOATING, MAX_SPEED, gpio::Pin_Remap_Select::NO_REMAP, 0, 0));
            break;
        case INPUT_PULLUP:
            setPinOp(pin, createPackedPinOps(gpio::Pin_Mode::INPUT_PULLUP, MAX_SPEED, gpio::Pin_Remap_Select::NO_REMAP, 0, 0));
            break;
        case INPUT_PULLDOWN:
            setPinOp(pin, createPackedPinOps(gpio::Pin_Mode::INPUT_PULLDOWN, MAX_SPEED, gpio::Pin_Remap_Select::NO_REMAP, 0, 0));
            break;
        case OUTPUT:
            setPinOp(pin, createPackedPinOps(gpio::Pin_Mode::OUTPUT_PUSHPULL, MAX_SPEED, gpio::Pin_Remap_Select::NO_REMAP, 0, 0));
            break;
        case OUTPUT_OPENDRAIN:
            setPinOp(pin, createPackedPinOps(gpio::Pin_Mode::OUTPUT_OPENDRAIN, MAX_SPEED, gpio::Pin_Remap_Select::NO_REMAP, 0, 0));
            break;
        // NOTE: INPUT_ANALOG is deprecated.
        // It is not part of the Arduino API and
        // is already being handled directly in Analog.cpp
        #pragma GCC diagnostic ignored "-Wswitch"
        case INPUT_ANALOG:
            if ((pin != ADC_TEMP) && (pin != ADC_VREF)) {
                pinOpsPinout(ADC_PinOps, pin);
            }
            break;
        default:    // Do nothing
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
    const PortPinPair& pp = port_pin_map[pin];
    if (pp.port == gpio::GPIO_Base::INVALID || pp.pin == gpio::Pin_Number::INVALID) {
        return;
    }

    auto result = gpio::GPIO::get_instance(pp.port);
    if (result.error() == gpio::GPIO_Error_Type::OK) {
        result.value().write_pin(pp.pin, (status != LOW));
    }
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
    const PortPinPair& pp = port_pin_map[pin];
    if (pp.port == gpio::GPIO_Base::INVALID || pp.pin == gpio::Pin_Number::INVALID) {
        return LOW; // Return a default value for invalid port or pin
    }

    auto result = gpio::GPIO::get_instance(pp.port);
    if (result.error() != gpio::GPIO_Error_Type::OK) {
        return LOW; // Return a default value if instance retrieval fails
    }

    return result.value().read_pin(pp.pin) ? HIGH : LOW;
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
    const PortPinPair& pp = port_pin_map[pin];
    if (pp.port == gpio::GPIO_Base::INVALID || pp.pin == gpio::Pin_Number::INVALID) {
        return;
    }

    auto result = gpio::GPIO::get_instance(pp.port);
    if (result.error() == gpio::GPIO_Error_Type::OK) {
        result.value().toggle_pin(pp.pin);
    }
}
