#pragma once

#include "Arduino.h"

inline constexpr uint8_t Max_Ports = 4U;

class PinConfigManager {
public:
    PinConfigManager() {}

    // Check if the pin is already configured
    bool isPinConfigured(pin_size_t pin);
    // Mark the pin as configured
    void setPinConfigured(pin_size_t pin);
    // Reset to unconfigured
    void resetPinConfigured(pin_size_t pin);

    uint16_t pinIsConfig[Max_Ports] = {0U};
};

extern PinConfigManager pinConfigManager;
