#pragma once

#include <Arduino.h>

#ifdef EEPROM_SIZE
    inline static constexpr uint32_t ReservedSize = EEPROM_SIZE;
#else
    inline static constexpr uint32_t ReservedSize = 2048U;  // 2KB default
#endif

inline static constexpr uint32_t EepromEnd = ReservedSize - 1U;

class EepromFlash {
public:
    static auto get_instance() -> EepromFlash&;

    // Read/write
    uint8_t readByteEeprom(const uint32_t address);
    void writeByteEeprom(uint32_t address, uint8_t data);
    // Read/write buffered
    uint8_t readByteBuffered(const uint32_t address);
    void writeByteBuffered(uint32_t address, uint8_t data);
    // Buffer operations
    void fillBuffer();
    void flushBuffer();

private:
    EepromFlash();

    fmc::FMC& fmc_;
};
