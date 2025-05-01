
#include "PinConfigManager.hpp"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"

inline constexpr uint8_t Max_Pins_Per_Port = 16U;

/**
 * @brief Check if the specified pin is configured.
 *
 * This function checks if the specified pin has been configured using one of
 * the pinMode(), analogRead(), or analogWrite() functions. It returns true if
 * the pin has been configured or false if it has not been configured.
 *
 * @param pin The pin number to check.
 *
 * @return true if the pin has been configured, false if it has not been
 *         configured.
 */
auto PinConfigManager::isPinConfigured(pin_size_t pin) -> bool {
    const PortPinPair& pp = port_pin_map[pin];
    return (pinIsConfig[static_cast<size_t>(pp.port)] & (1U << static_cast<size_t>(pp.pin))) != 0U;
}

/**
 * @brief Set the specified pin as configured.
 *
 * This function sets the specified pin as being configured. This is used by
 * the pinMode(), analogRead(), and analogWrite() functions to keep track of
 * which pins have been configured.
 *
 * @param pin The pin number to set as configured.
 */
void PinConfigManager::setPinConfigured(pin_size_t pin) {
    const PortPinPair& pp = port_pin_map[pin];
    pinIsConfig[static_cast<size_t>(pp.port)] |= (1U << static_cast<size_t>(pp.pin));
}

/**
 * @brief Reset the specified pin to unconfigured.
 *
 * This function resets the specified pin as not being configured. This is used
 * by the pinMode(), analogRead(), and analogWrite() functions to keep track of
 * which pins have been configured.
 *
 * @param pin The pin number to reset as unconfigured.
 */
void PinConfigManager::resetPinConfigured(pin_size_t pin) {
    const PortPinPair& pp = port_pin_map[pin];
    pinIsConfig[static_cast<size_t>(pp.port)] &= (~(1U << static_cast<size_t>(pp.pin)));
}

PinConfigManager pinConfigManager;
