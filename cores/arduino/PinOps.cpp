
#include "Arduino.h"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"

/**
 * @brief Configures a pin based on a packed 32-bit pin configuration value
 *
 * This function takes a pin number and a packed 32-bit pin configuration value
 * and configures the pin accordingly. The packed pin configuration value
 * consists of the pin mode, output speed, and pin remap select.
 *
 * @param pin The pin number to configure
 * @param packedPinOps The packed 32-bit pin configuration value
 */
void setPinOp(pin_size_t pin, uint32_t packedPinOps) {
    gpio::Pin_Remap_Select remap = getPackedPinRemap(packedPinOps);
    if (remap != gpio::Pin_Remap_Select::NO_REMAP) {
        gpio::AFIO::get_instance().set_remap(remap);
    }

    gpio::GPIO_Base port = getPortFromPin(pin);
    if (port == gpio::GPIO_Base::INVALID) {
        core_debug("Invalid port");
        return;
    }

    gpio::Pin_Number pinInPort = getPinInPort(pin);
    if (pinInPort == gpio::Pin_Number::INVALID) {
        core_debug("Invalid pin number");
        return;
    }

    gpio::Pin_Mode pinMode = getPackedPinMode(packedPinOps);
    if (pinMode == gpio::Pin_Mode::INVALID) {
        core_debug("Invalid pin mode");
        return;
    }

    gpio::Output_Speed pinSpeed = getPackedPinSpeed(packedPinOps);
    if (pinSpeed == gpio::Output_Speed::INVALID) {
        core_debug("Invalid output speed");
        return;
    }

    auto result = gpio::GPIO::get_instance(port);
    if (result.error() == gpio::GPIO_Error_Type::OK) {
        result.value().set_pin_mode(pinInPort, pinMode, pinSpeed);
    }
}

/**
 * @brief Configures a pin on the specified port with the given packed pin configuration
 *
 * This function takes a packed 32-bit pin configuration value and configures the pin
 * on the specified port accordingly. The packed pin configuration value consists of
 * the pin mode, output speed, and pin remap select.
 *
 * @param port The GPIO port to configure the pin on
 * @param pin The pin number to configure
 * @param packedPinOps The packed 32-bit pin configuration value
 */
void setPinOp(gpio::GPIO_Base port, gpio::Pin_Number pin, uint32_t packedPinOps) {
    gpio::Pin_Remap_Select remap = getPackedPinRemap(packedPinOps);
    if (remap != gpio::Pin_Remap_Select::NO_REMAP) {
        gpio::AFIO::get_instance().set_remap(remap);
    }

    gpio::Pin_Mode mode = getPackedPinMode(packedPinOps);
    if (mode == gpio::Pin_Mode::INVALID) {
        core_debug("Invalid pin mode");
        return;
    }

    gpio::Output_Speed speed = getPackedPinSpeed(packedPinOps);
    if (speed == gpio::Output_Speed::INVALID) {
        core_debug("Invalid output speed");
        return;
    }

    auto result = gpio::GPIO::get_instance(port);
    if (result.error() == gpio::GPIO_Error_Type::OK) {
        result.value().set_pin_mode(pin, mode, speed);
    }
}

bool isPinNumberValid(pin_size_t pin) {
    return (pin >= 0U && pin <= DIGITAL_PIN_COUNT);
}

uint32_t portOutputRegister(GPIO_Port_t port) {
    return getPortOutputRegister(static_cast<gpio::GPIO_Base>(port));
}

uint32_t portInputRegister(GPIO_Port_t port) {
    return getPortInputRegister(static_cast<gpio::GPIO_Base>(port));
}

uint32_t portSetRegister(GPIO_Port_t port) {
    return getPortSetRegister(static_cast<gpio::GPIO_Base>(port));
}

uint32_t portClearRegister(GPIO_Port_t port) {
    return getPortClearRegister(static_cast<gpio::GPIO_Base>(port));
}

GPIO_Port_t digitalPinToPort(pin_size_t pin) {
    return static_cast<GPIO_Port_t>(getPortFromPin(pin));
}
