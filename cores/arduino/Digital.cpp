#include "Arduino.h"
#include "Analog.h"
#include "PinConfigManager.hpp"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"

/**
 * Configures the specified pin's mode of operation.
 * 
 * @param pin The pin number to configure
 * @param mode The mode to set (INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN, 
 *             OUTPUT_OPENDRAIN, or INPUT_ANALOG)
 * 
 * Handles PWM cleanup if pin was previously configured for timer output.
 * For analog inputs, uses ADC pin configuration if not temp/vref pins.
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
    case INPUT:
        setPinOp(pin, createPackedPinOps(gpio::Pin_Mode::INPUT_FLOATING, gpio::Output_Speed::SPEED_MAX, gpio::Pin_Remap_Select::NO_REMAP, 0, 0));
        break;
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
    #pragma GCC diagnostic ignored "-Wswitch"
    case INPUT_ANALOG:
    default:    // Default to INPUT_ANALOG
        if ((pin != ADC_TEMP) && (pin != ADC_VREF)) {
            pinOpsPinout(ADC_PinOps, pin);
        }
        break;
    }
}

/**
 * Sets the digital output value of a specified pin to HIGH or LOW.
 * 
 * @param pin The pin number to write to
 * @param status The output value (HIGH/LOW) to set
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
 * Reads the digital state of a specified pin.
 * 
 * @param pin The pin number to read from
 * @return PinStatus HIGH if pin is high, LOW if pin is low
 */
PinStatus digitalRead(pin_size_t pin) {
    auto result = gpio::GPIO::get_instance(getPortFromPin(pin));
    auto& instance = result.value();
    return instance.read_pin(getPinInPort(pin)) ? HIGH : LOW;
}

/**
 * Toggles the state of a digital GPIO pin.
 * 
 * @param pin The pin number to toggle
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
