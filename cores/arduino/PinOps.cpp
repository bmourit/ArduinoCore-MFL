
#include "Arduino.h"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"
#include "variant.h"

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
    auto remap = getPackedPinRemap(packedPinOps);
    if (remap != gpio::Pin_Remap_Select::NO_REMAP) {
        gpio::AFIO::get_instance().set_remap(remap);
    }

    const PortPinPair& pp = port_pin_map[pin];

    if (pp.port == gpio::GPIO_Base::INVALID || pp.pin == gpio::Pin_Number::INVALID) {
        core_debug("Invalid port/pin");
        return;
    }

    auto pinMode = getPackedPinMode(packedPinOps);
    if (pinMode == gpio::Pin_Mode::INVALID) {
        core_debug("Invalid pin mode");
        return;
    }

    auto pinSpeed = getPackedPinSpeed(packedPinOps);
    if (pinSpeed == gpio::Output_Speed::INVALID) {
        core_debug("Invalid output speed");
        return;
    }

    auto result = gpio::GPIO::get_instance(pp.port);
    if (result.error() == gpio::GPIO_Error_Type::OK) {
        result.value().set_pin_mode(pp.pin, pinMode, pinSpeed);
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

    // Always set the remap, even if it is NO_REMAP
    // in case remap was previously set and needs to be cleared
    gpio::AFIO::get_instance().set_remap(remap);

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

/**
 * @brief Checks if the given pin number is valid
 *
 * This function takes a pin number as input and checks if it is within the valid range
 * of digital pins. The function returns true if the pin number is valid, and false
 * otherwise.
 *
 * @param pin The pin number to check
 * @return true if the pin number is valid, and false otherwise
 */
bool isPinNumberValid(pin_size_t pin) {
    return (pin >= 0U && pin <= DIGITAL_PIN_COUNT);
}

/**
 * @brief Returns the output register for the given port
 *
 * This function takes a GPIO port as input and returns the output register
 * associated with that port. The output register is the register that controls
 * the output value of the port.
 *
 * @param port The GPIO port to retrieve the output register for
 * @return The output register of the given port
 */
uint32_t portOutputRegister(GPIO_Port_t port) {
    return getPortOutputRegister(static_cast<gpio::GPIO_Base>(port));
}

/**
 * @brief Returns the input register for the given port
 *
 * This function takes a GPIO port as input and returns the input register
 * associated with that port. The input register is the register that reflects
 * the current input state of the port.
 *
 * @param port The GPIO port to retrieve the input register for
 * @return The input register of the given port
 */
uint32_t portInputRegister(GPIO_Port_t port) {
    return getPortInputRegister(static_cast<gpio::GPIO_Base>(port));
}

/**
 * @brief Returns the set register for the given port
 *
 * This function takes a GPIO port as input and returns the set register
 * associated with that port. The set register is the register that sets
 * the output value of the port.
 *
 * @param port The GPIO port to retrieve the set register for
 * @return The set register of the given port
 */
uint32_t portSetRegister(GPIO_Port_t port) {
    return getPortSetRegister(static_cast<gpio::GPIO_Base>(port));
}

/**
 * @brief Returns the clear register for the given port
 *
 * This function takes a GPIO port as input and returns the clear register
 * associated with that port. The clear register is the register that clears
 * the output value of the port.
 *
 * @param port The GPIO port to retrieve the clear register for
 * @return The clear register of the given port
 */
uint32_t portClearRegister(GPIO_Port_t port) {
    return getPortClearRegister(static_cast<gpio::GPIO_Base>(port));
}

/**
 * @brief Returns the GPIO port associated with the given digital pin number
 *
 * This function takes a digital pin number as input and returns the GPIO port
 * associated with that pin number. The GPIO port is an opaque type that can be
 * used as an argument to the portOutputRegister, portInputRegister, portSetRegister,
 * and portClearRegister functions.
 *
 * @param pin The digital pin number to retrieve the GPIO port for
 * @return The GPIO port associated with the given digital pin number
 */
GPIO_Port_t digitalPinToPort(pin_size_t pin) {
    const PortPinPair& pp = port_pin_map[pin];
    return static_cast<GPIO_Port_t>(pp.port);
}
