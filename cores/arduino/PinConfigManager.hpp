#pragma once

#include "Arduino.h"
#include <array>

inline constexpr uint8_t Max_Ports = 4U;

class PinConfigManager {
public:
    PinConfigManager() = default;

    auto isPinConfigured(pin_size_t pin) -> bool;
    void setPinConfigured(pin_size_t pin);
    void resetPinConfigured(pin_size_t pin);

    std::array<uint16_t, Max_Ports> pinIsConfig = {0U};
};

extern PinConfigManager pinConfigManager;
