
#include <Arduino.h>
#include "EepromFlash.h"

constexpr uint32_t FlashEndAddress = fmc::Bank0_End_Address;
constexpr uint32_t FlashStartAddress = (FlashEndAddress + 1U) - ReservedSize;

static uint8_t eeprom_buffer[EepromEnd + 1] __attribute__((aligned(4))) = {0};

auto EepromFlash::get_instance() -> EepromFlash& {
    static EepromFlash instance;
    return instance;
}

EepromFlash::EepromFlash() :
    fmc_(fmc::FMC::get_instance())
= default;

uint8_t EepromFlash::readByteEeprom(const uint32_t address) {
    fillBuffer();
    return readByteBuffered(address);
}

void EepromFlash::writeByteEeprom(uint32_t address, uint8_t data) {
    writeByteBuffered(address, data);
    flushBuffer();
}

uint8_t EepromFlash::readByteBuffered(const uint32_t address) {
    return eeprom_buffer[address];
}

void EepromFlash::writeByteBuffered(uint32_t address, uint8_t data) {
    eeprom_buffer[address] = data;
}

void EepromFlash::fillBuffer() {
    memcpy(eeprom_buffer, reinterpret_cast<const uint8_t*>(FlashStartAddress), EepromEnd + 1U);
}

void EepromFlash::flushBuffer() {
    uint32_t offset = 0U;
    uint32_t address = FlashStartAddress;
    const uint32_t address_end = FlashStartAddress + EepromEnd;
    uint64_t data = 0U;

    fmc_.unlock();
    fmc_.clear_flag(fmc::Status_Flags::FLAG_BANK0_END);
    fmc_.clear_flag(fmc::Status_Flags::FLAG_BANK0_WPERR);
    fmc_.clear_flag(fmc::Status_Flags::FLAG_BANK0_PGERR);

    if (fmc_.erase_page(address) != fmc::FMC_Error_Type::READY) {
        return;
    }

    while (address <= address_end) {
        data = *reinterpret_cast<uint64_t*>(eeprom_buffer + offset);
        if (fmc_.program_word(address, data) == fmc::FMC_Error_Type::READY) {
            address += 4;
            offset += 4;
        } else {
            address = address_end + 1;
        }
    }
    fmc_.lock();
}
