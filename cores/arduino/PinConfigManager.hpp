#pragma once

#include "Arduino.h"

inline constexpr uint8_t Max_Ports = 4U;

class PinConfigManager {
public:
    PinConfigManager() = default;

    auto isPinConfigured(pin_size_t pin) -> bool;
    void setPinConfigured(pin_size_t pin);
    void resetPinConfigured(pin_size_t pin);

    uint16_t pinIsConfig[Max_Ports] = {0U};
};

extern PinConfigManager pinConfigManager;
