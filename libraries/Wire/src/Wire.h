/*
 * TWI/I2C library for the MFL Ardunio Core
 * Copyright (c) 2025 Arduino LLC. All rights reserved.
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
#include "api/HardwareI2C.h"
#include "api/RingBuffer.h"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"

namespace arduino {

#ifndef WIRE_BUFFER_LENGTH
    #define WIRE_BUFFER_LENGTH 32
#endif

class TwoWire : public HardwareI2C {
public:
    static TwoWire& get_instance(i2c::I2C_Base Base, pin_size_t sdaPin = NO_PIN, pin_size_t sclPin = NO_PIN);

    void begin() override;
    void begin(uint8_t address) override;
    void end() override;
    void setClock(uint32_t speed) override;
    void beginTransmission(uint8_t address) override;
    uint8_t endTransmission(bool stopBit) override;
    uint8_t endTransmission() override;
    size_t requestFrom(uint8_t address, size_t len, bool stopBit) override;
    size_t requestFrom(uint8_t address, size_t len) override;
    void onReceive(void(*)(int)) override;
    void onRequest(void(*)(void)) override;

    virtual size_t write(uint8_t data);
    virtual size_t write(const uint8_t* buffer, size_t len);

    virtual int available();
    virtual int read();
    virtual int peek();
    virtual void flush();

    inline size_t write(unsigned long n) { return write(static_cast<uint8_t>(n)); }
    inline size_t write(long n) { return write(static_cast<uint8_t>(n)); }
    inline size_t write(unsigned int n) { return write(static_cast<uint8_t>(n)); }
    inline size_t write(int n) { return write(static_cast<uint8_t>(n)); }
    using Print::write;

    // Interrupt handlers
    void errorHandler();
    void interruptHandler();
    void configurePins();

private:
    TwoWire(i2c::I2C_Base Base, pin_size_t sdaPin, pin_size_t sclPin);

    i2c::I2C_Base base_;
    i2c::I2C& i2c_;
    pin_size_t customSdaPin_;
    pin_size_t customSclPin_;
    RingBufferN<WIRE_BUFFER_LENGTH> rxBuffer_;
    RingBufferN<WIRE_BUFFER_LENGTH> txBuffer_;
    uint8_t ownerAddress_;
    uint32_t txAddress_;
    bool transmitting_;

    // Master helpers
    i2c::I2C_Error_Type masterTransmit(uint8_t address, uint8_t* buffer, uint8_t len, bool stopBit);
    i2c::I2C_Error_Type masterReceive(uint8_t address, uint8_t* buffer, uint8_t len, bool stopBit);

    // Slave helper
    i2c::I2C_Error_Type writeSlaveBuffer(const uint8_t* buffer, uint8_t len);

    // Read/write
    uint8_t readByte(uint32_t last);
    i2c::I2C_Error_Type writeByte(uint8_t data);

    // State check helpers
    i2c::I2C_Error_Type waitForReadyState(uint8_t address);
    i2c::I2C_Error_Type checkBusyState();
    i2c::I2C_Error_Type stop();

    // Interrupt helpers
    void setSlaveInterruptEnable();
    void (*onReceiveCallback)(int);
    void (*onRequestCallback)(void);
};

} // namespace arduino


#ifdef SUPPORT_I2C0
    extern arduino::TwoWire& Wire;
#endif

#ifdef SUPPORT_I2C1
    extern arduino::TwoWire& Wire1;
#endif
