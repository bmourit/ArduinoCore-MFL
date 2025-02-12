/*
 * SPI Master library for MFL Arduino Core
 * Copyright (c) 2025 Arduino LLC
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <Arduino.h>
#include <assert.h>

#include "SPI.h"

namespace arduino {

SPIClassMFL& SPIClassMFL::get_instance(spi::SPI_Base Base, pin_size_t mosiPin,
                                       pin_size_t misoPin, pin_size_t sclkPin, pin_size_t sselPin)
{
    switch (Base) {
        case spi::SPI_Base::SPI0_BASE: {
            static SPIClassMFL MYSPI0(spi::SPI_Base::SPI0_BASE, mosiPin, misoPin, sclkPin, sselPin);
            return MYSPI0;
        }
        case spi::SPI_Base::SPI1_BASE: {
            static SPIClassMFL MYSPI1(spi::SPI_Base::SPI1_BASE, mosiPin, misoPin, sclkPin, sselPin);
            return MYSPI1;
        }
        case spi::SPI_Base::SPI2_BASE: {
            static SPIClassMFL MYSPI2(spi::SPI_Base::SPI2_BASE, mosiPin, misoPin, sclkPin, sselPin);
            return MYSPI2;
        }
        case spi::SPI_Base::INVALID:
        default:
#ifdef CORE_DEBUG
            core_debug("Invalid SPI instance!");
            __builtin_trap();
#endif
            static SPIClassMFL dummy(spi::SPI_Base::INVALID, mosiPin, misoPin, sclkPin, sselPin);
            return dummy;

    }
}

SPIClassMFL::SPIClassMFL(spi::SPI_Base Base, pin_size_t mosiPin,
                         pin_size_t misoPin, pin_size_t sclkPin, pin_size_t sselPin) :
    base_(Base),
    spi_(spi::SPI::get_instance(Base).value()),
    customMosiPin_(mosiPin),
    customMisoPin_(misoPin),
    customSclkPin_(sclkPin),
    customSselPin_(sselPin),
    config_(spi::default_config),
    settings_(DEFAULT_SPI_SETTINGS),
    interruptMask_(0),
    initialized_(false)
{
}

/**
 * @brief Initializes the SPI peripheral.
 *
 * This function configures the SPI peripheral's operational mode, frame format, polarity,
 * clock phase, endianness, and NSS type, as well as the divider for the peripheral clock,
 * and enables the peripheral.
 *
 * @note This function does not configure the pins. If the default pins are not used,
 *       the user must configure the pins using the configurePins() method before
 *       calling begin().
 *
 * @note The default settings are:
 *         - Operational mode: Master Full-Duplex
 *         - Frame format: 8-bit
 *         - NSS type: Software NSS
 *         - PCLK divider: PCLK_2
 *         - MSBF: Most Significant Bit First
 *         - Polarity: Pull Low
 *         - Clock phase: Phase First Edge
 */
void SPIClassMFL::begin() {
    configurePins();
    spi_.init({
        spi::Operational_Mode::MFD_MODE,
        spi::Frame_Format::FF_8BIT,
        spi::NSS_Type::SOFTWARE_NSS,
        spi::PCLK_Divider::PCLK_2,
        spi::Endian_Type::MSBF,
        spi::Clock_Polarity::PULL_LOW,
        spi::Clock_Phase::PHASE_FIRST_EDGE
    });
    updateSettings(DEFAULT_SPI_SETTINGS);
    spi_.set_enable(true);
    initialized_ = true;
}

/**
 * @brief Resets the SPI peripheral.
 *
 * This function resets the SPI peripheral, disables it, and marks it as uninitialized.
 *
 * @note This function does not reset the pins to their default state.
 */
void SPIClassMFL::end() {
    spi_.reset();
    spi_.set_enable(false);
    // TODO:
    //  Reset the pins to default input analog
    initialized_ = false;
}

/**
 * @brief Initializes the SPI peripheral and changes the current settings.
 *
 * This function does the following:
 * - If the SPI peripheral is not initialized, it initializes it.
 * - If the given settings are different from the current settings, it updates the settings.
 * - It reinitializes the SPI peripheral with the given settings.
 *
 * @param settings The settings for the SPI peripheral
 */
void SPIClassMFL::beginTransaction(SPISettings settings) {
    if (!initialized_) {
        begin();
    }
    // Only if settings have changed
    if (settings_ != settings) { updateSettings(settings); }
    settings_ = settings;

    spi_.init(config_);
}

/**
 * @brief Releases the SPI bus
 *
 * This function releases the SPI bus, re-enabling other libraries to use the SPI bus.
 *
 * @note This function does not reset the pins to their default state.
 */
void SPIClassMFL::endTransaction() {
}

/**
 * @brief Transfer one byte of data over the SPI bus.
 *
 * This function transfers one byte of data over the SPI bus and returns the received byte.
 *
 * @param data The byte of data to send
 *
 * @return The byte of data received while sending
 */
uint8_t SPIClassMFL::transfer(uint8_t data) {
    spi_.data_transmit(data);
    while (!spi_.get_flag(spi::Status_Flags::FLAG_RBNE)) {}
    return static_cast<uint8_t>(spi_.data_receive() & 0xFF);
}

/**
 * @brief Transfer one 16-bit word of data over the SPI bus.
 *
 * This function transfers one 16-bit word of data over the SPI bus and returns the received word.
 *
 * @param data The 16-bit word of data to send
 *
 * @return The 16-bit word of data received while sending
 */
uint16_t SPIClassMFL::transfer16(uint16_t data) {
    spi_.data_transmit(data);
    while (!spi_.get_flag(spi::Status_Flags::FLAG_RBNE)) {}
    return spi_.data_receive();
}

/**
 * @brief Transfer a block of data over the SPI bus.
 *
 * This function transfers the given block of data over the SPI bus.
 * The data is sent and received in the same buffer.
 *
 * @param buf The buffer containing the data to be sent
 * @param count The number of bytes in the buffer
 */
void SPIClassMFL::transfer(void* buf, size_t count) {
    uint8_t* buffer = static_cast<uint8_t*>(buf);
    for (size_t i = 0; i < count; i++) {
        buffer[i] = transfer(buffer[i]);
    }
}

/**
 * @brief Enable interrupts for the given interrupt number.
 *
 * This function enables the given interrupt number and sets the corresponding bit in the interrupt mask.
 * If the given interrupt number is invalid, the function does nothing.
 *
 * @param interruptNumber The interrupt number to enable
 */
void SPIClassMFL::usingInterrupt(int interruptNumber) {
    if (interruptNumber < 0 || static_cast<uint32_t>(interruptNumber) >= sizeof(interruptMask_) * 8) {
        // Invalid interrupt number, TODO: handle error if necessary
        return;
    }

    // Set the bit for the given interrupt number in the mask
    interruptMask_ |= (1 << interruptNumber);

    // Enable the interrupt in the NVIC
    NVIC_EnableIRQ(static_cast<IRQn_Type>(interruptNumber));
}

/**
 * @brief Disable interrupts for the given interrupt number.
 *
 * This function disables the given interrupt number and clears the corresponding bit in the interrupt mask.
 * If the given interrupt number is invalid, the function does nothing.
 *
 * @param interruptNumber The interrupt number to disable
 */
void SPIClassMFL::notUsingInterrupt(int interruptNumber) {
    if (interruptNumber < 0 || static_cast<uint32_t>(interruptNumber) >= sizeof(interruptMask_) * 8) {
        // Invalid interrupt number, TODO: handle error if necessary
        return;
    }

    // Clear the bit for the given interrupt number in the mask
    interruptMask_ &= ~(1 << interruptNumber);

    // Disable the interrupt in the NVIC
    NVIC_DisableIRQ(static_cast<IRQn_Type>(interruptNumber));
}

/**
 * @brief Set the SPI data mode.
 *
 * This function sets the SPI data mode to one of the four available modes.
 * The data mode determines the clock polarity and clock phase of the SPI bus.
 * The available modes are:
 *
 * - SPI_MODE0: Clock polarity is low, clock phase is first edge.
 * - SPI_MODE1: Clock polarity is low, clock phase is second edge.
 * - SPI_MODE2: Clock polarity is high, clock phase is first edge.
 * - SPI_MODE3: Clock polarity is high, clock phase is second edge.
 *
 * @param dataMode The SPI data mode to set
 */
void SPIClassMFL::setDataMode(SPIMode dataMode) {
    switch (dataMode) {
        case SPI_MODE0:
            config_.polarity_pull = spi::Clock_Polarity::PULL_LOW;
            config_.clock_phase = spi::Clock_Phase::PHASE_FIRST_EDGE;
            break;
        case SPI_MODE1:
            config_.polarity_pull = spi::Clock_Polarity::PULL_LOW;
            config_.clock_phase = spi::Clock_Phase::PHASE_SECOND_EDGE;
            break;
        case SPI_MODE2:
            config_.polarity_pull = spi::Clock_Polarity::PULL_HIGH;
            config_.clock_phase = spi::Clock_Phase::PHASE_FIRST_EDGE;
            break;
        case SPI_MODE3:
            config_.polarity_pull = spi::Clock_Polarity::PULL_HIGH;
            config_.clock_phase = spi::Clock_Phase::PHASE_SECOND_EDGE;
            break;
    }
    spi_.init(config_);
}

/**
 * @brief Set the SPI bit order.
 *
 * This function sets the SPI bit order to one of two available orders.
 * The bit order determines the order of the bits in the SPI data frame.
 * The available orders are:
 *
 * - MSBFIRST: Most significant bit first
 * - LSBFIRST: Least significant bit first
 *
 * @param bitOrder The SPI bit order to set
 */
void SPIClassMFL::setBitOrder(BitOrder bitOrder) {
    config_.msbf = (bitOrder == MSBFIRST) ? spi::Endian_Type::MSBF : spi::Endian_Type::LSBF;
    spi_.init(config_);
}

/**
 * @brief Set the SPI clock divider.
 *
 * This function sets the SPI clock divider from the MCU clock frequency.
 * The SPI clock divider determines the SPI clock frequency.
 * The available divider values are 2, 4, 8, 16, 32, 64, 128, and 256.
 *
 * @param clock The desired SPI clock frequency in Hz
 */
void SPIClassMFL::setClockDivider(uint32_t clock) {
    uint32_t pclk = RCU_I.get_system_clock();
    uint32_t div = pclk / clock;

    config_.pclk_divider = (div <= 2)   ? spi::PCLK_Divider::PCLK_2 :
                           (div <= 4)   ? spi::PCLK_Divider::PCLK_4 :
                           (div <= 8)   ? spi::PCLK_Divider::PCLK_8 :
                           (div <= 16)  ? spi::PCLK_Divider::PCLK_16 :
                           (div <= 32)  ? spi::PCLK_Divider::PCLK_32 :
                           (div <= 64)  ? spi::PCLK_Divider::PCLK_64 :
                           (div <= 128) ? spi::PCLK_Divider::PCLK_128 :
                           spi::PCLK_Divider::PCLK_256;

    /*if (div <= 2) config_.pclk_divider = spi::PCLK_Divider::PCLK_2;
    else if (div <= 4) config_.pclk_divider = spi::PCLK_Divider::PCLK_4;
    else if (div <= 8) config_.pclk_divider = spi::PCLK_Divider::PCLK_8;
    else if (div <= 16) config_.pclk_divider = spi::PCLK_Divider::PCLK_16;
    else if (div <= 32) config_.pclk_divider = spi::PCLK_Divider::PCLK_32;
    else if (div <= 64) config_.pclk_divider = spi::PCLK_Divider::PCLK_64;
    else if (div <= 128) config_.pclk_divider = spi::PCLK_Divider::PCLK_128;
    else config_.pclk_divider = spi::PCLK_Divider::PCLK_256;*/

    spi_.init(config_);
}

/**
 * @brief Update the SPI settings
 *
 * This function updates the SPI settings with the settings from the given
 * SPISettings object.
 *
 * @param settings The SPISettings object with the new settings
 */
void SPIClassMFL::updateSettings(SPISettings settings) {
    setDataMode(settings.getDataMode());
    setBitOrder(settings.getBitOrder());
    setClockDivider(settings.getClockFreq());
}

/**
 * @brief Configures the SPI pins.
 *
 * This function configures the SPI pins (MOSI, MISO, SCLK, and SSEL) for use with the
 * SPI peripheral. If a custom pin is specified for any of the pins, the function will
 * configure that pin. Otherwise, it will use the pin mapping defined in the Variant.h file.
 *
 * The function will set the pin mode and speed according to the pin operations defined for
 * the SPI peripheral and the pin.
 *
 * The function will also check if a remap is required for the pin and apply it if necessary.
 */
void SPIClassMFL::configurePins() {
    // MOSI pin
    if (customMosiPin_ != NO_PIN) {
        pinOpsPinout(SPI_MOSI_PinOps, customMosiPin_);
    } else {
        auto mosiPinOps = getPinOpsByPeripheral(SPI_MOSI_PinOps, base_);
        if (mosiPinOps == invalidPinOps) {
            return;
        }
        auto mosiMode = getPackedPinMode(mosiPinOps.packedPinOps);
        auto mosiSpeed = getPackedPinSpeed(mosiPinOps.packedPinOps);
        // Initialize pin
        auto& mosiPort = gpio::GPIO::get_instance(mosiPinOps.port).value();
        mosiPort.set_pin_mode(mosiPinOps.pin, mosiMode, mosiSpeed);
        // Check remap
        auto mosiRemap = getPackedPinRemap(mosiPinOps.packedPinOps);
        if (mosiRemap != gpio::Pin_Remap_Select::NO_REMAP) {
            AFIO_I.set_remap(mosiRemap);
        }
    }

    // MISO pin
    if (customMisoPin_ != NO_PIN) {
        pinOpsPinout(SPI_MISO_PinOps, customMisoPin_);
    } else {
        auto misoPinOps = getPinOpsByPeripheral(SPI_MISO_PinOps, base_);
        if (misoPinOps == invalidPinOps) {
            return;
        }
        auto misoMode = getPackedPinMode(misoPinOps.packedPinOps);
        auto misoSpeed = getPackedPinSpeed(misoPinOps.packedPinOps);
        // Initialize pin
        auto& misoPort = gpio::GPIO::get_instance(misoPinOps.port).value();
        misoPort.set_pin_mode(misoPinOps.pin, misoMode, misoSpeed);
        // Check remap
        auto misoRemap = getPackedPinRemap(misoPinOps.packedPinOps);
        if (misoRemap != gpio::Pin_Remap_Select::NO_REMAP) {
            AFIO_I.set_remap(misoRemap);
        }
    }

    // SCLK pin
    if (customSclkPin_ != NO_PIN) {
        pinOpsPinout(SPI_SCLK_PinOps, customSclkPin_);
    } else {
        auto sclkPinOps = getPinOpsByPeripheral(SPI_SCLK_PinOps, base_);
        if (sclkPinOps == invalidPinOps) {
            return;
        }
        auto sclkMode = getPackedPinMode(sclkPinOps.packedPinOps);
        auto sclkSpeed = getPackedPinSpeed(sclkPinOps.packedPinOps);
        // Initialize pin
        auto& sclkPort = gpio::GPIO::get_instance(sclkPinOps.port).value();
        sclkPort.set_pin_mode(sclkPinOps.pin, sclkMode, sclkSpeed);
        // Check remap
        auto sclkRemap = getPackedPinRemap(sclkPinOps.packedPinOps);
        if (sclkRemap != gpio::Pin_Remap_Select::NO_REMAP) {
            AFIO_I.set_remap(sclkRemap);
        }
    }

    // SSEL pin (optional)
    if (customSselPin_ != NO_PIN) {
        pinOpsPinout(SPI_SSEL_PinOps, customSselPin_);
    }
}

} // namespace arduino

#ifdef SUPPORT_SPI0
    arduino::SPIClassMFL& SPI = arduino::SPIClassMFL::get_instance(spi::SPI_Base::SPI0_BASE);
#endif
#ifdef SUPPORT_SPI1
    arduino::SPIClassMFL& SPI1 = arduino::SPIClassMFL::get_instance(spi::SPI_Base::SPI1_BASE);
#endif
#ifdef SUPPORT_SPI2
    arduino::SPIClassMFL& SPI2 = arduino::SPIClassMFL::get_instance(spi::SPI_Base::SPI2_BASE);
#endif
