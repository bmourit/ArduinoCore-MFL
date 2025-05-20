#pragma once

#include "Arduino.h"
#include <array>

inline constexpr uint8_t Max_Ports = 4U;

class PinConfigManager {
public:
    static auto get_instance() -> PinConfigManager&;

    auto isPinConfigured(pin_size_t pin) -> bool;
    void setPinConfigured(pin_size_t pin);
    void resetPinConfigured(pin_size_t pin);

private:
    PinConfigManager();
    std::array<uint16_t, Max_Ports> pinIsConfig = {0U};

    // Prevent copying or assigning
    PinConfigManager(const PinConfigManager&) = delete;
    auto operator=(const PinConfigManager&) -> PinConfigManager& = delete;
};

extern PinConfigManager& pinConfigManager;
