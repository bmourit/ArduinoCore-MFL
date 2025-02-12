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

#pragma once

#include "Arduino.h"
#include "api/HardwareSPI.h"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"

namespace arduino {

// SPI_HAS_TRANSACTION means SPI has
//  - beginTransaction()
//  - endTransaction()
//  - usingInterrupt()
//  - SPISetting(clock, bitorder, datamode)
//#define SPI_HAS_TRANSACTION 1

// SPI_HAS_NOTUSINGINTERRUPT means that SPI has notUsingInterrupt() method
#define SPI_HAS_NOTUSINGINTERRUPT 1

class SPIClassMFL : public HardwareSPI {
public:
    static SPIClassMFL& get_instance(spi::SPI_Base Base,
                                     pin_size_t mosiPin = NO_PIN, pin_size_t misoPin = NO_PIN,
                                     pin_size_t sclkPin = NO_PIN, pin_size_t sselPin = NO_PIN);

    void begin() override;
    void end() override;
    void beginTransaction(SPISettings settings) override;
    void endTransaction() override;
    uint8_t transfer(uint8_t data) override;
    uint16_t transfer16(uint16_t data) override;
    void transfer(void *buf, size_t count) override;
    void usingInterrupt(int interruptNumber) override;
    void notUsingInterrupt(int interruptNumber) override;
    void attachInterrupt() {}
    void detachInterrupt() {}
    void setDataMode(SPIMode dataMode);
    void setBitOrder(BitOrder bitOrder);
    void setClockDivider(uint32_t clock);
    void configurePins();

private:
    SPIClassMFL(spi::SPI_Base Base, pin_size_t mosiPin, pin_size_t misoPin,
                pin_size_t sclkPin, pin_size_t sselPin);

    spi::SPI_Base base_;
    spi::SPI& spi_;
    pin_size_t customMosiPin_;
    pin_size_t customMisoPin_;
    pin_size_t customSclkPin_;
    pin_size_t customSselPin_;
    spi::SPI_Config config_;
    SPISettings settings_;
    int interruptMask_;
    bool initialized_;

    SPISettings const DEFAULT_SPI_SETTINGS = SPISettings(1000000, MSBFIRST, SPI_MODE0);

    void updateSettings(SPISettings settings);
};

} // namespace arduino

#ifdef SUPPORT_SPI0
    extern arduino::SPIClassMFL& SPI;
#endif
#ifdef SUPPORT_SPI1
    extern arduino::SPIClassMFL& SPI1;
#endif
#ifdef SUPPORT_SPI2
    extern arduino::SPIClassMFL& SPI2;
#endif
