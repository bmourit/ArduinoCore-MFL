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

#include <Arduino.h>

#include "Wire.h"

namespace arduino {

inline constexpr uint8_t BUFFER_LENGTH = WIRE_BUFFER_LENGTH;
inline constexpr uint8_t OWNER_ADDRESS = 0x33U;
inline constexpr uint32_t DEFAULT_SPEED = 100'000U;
inline constexpr uint32_t I2C_TIMEOUT_DEFAULT = 0xF0000U;

TwoWire& TwoWire::get_instance(i2c::I2C_Base Base, pin_size_t sdaPin, pin_size_t sclPin) {
    switch (Base) {
        case i2c::I2C_Base::I2C0_BASE: {
            static TwoWire WI2C0(i2c::I2C_Base::I2C0_BASE, sdaPin, sclPin);
            return WI2C0;
        }
        case i2c::I2C_Base::I2C1_BASE: {
            static TwoWire WI2C1(i2c::I2C_Base::I2C1_BASE, sdaPin, sclPin);
            return WI2C1;
        }
        case i2c::I2C_Base::INVALID:
        default:
#ifdef CORE_DEBUG
            core_debug("Invalid I2C instance!");
            __builtin_trap();
#endif
            static TwoWire dummy(i2c::I2C_Base::INVALID, sdaPin, sclPin);
            return dummy;
    }
}

TwoWire::TwoWire(i2c::I2C_Base Base, pin_size_t sdaPin, pin_size_t sclPin) :
    base_(Base),
    i2c_(i2c::I2C::get_instance(Base).value()),
    customSdaPin_(sdaPin),
    customSclPin_(sclPin),
    rxBuffer_(),
    txBuffer_(),
    ownerAddress_(0),
    txAddress_(0),
    transmitting_(false)
{
}

/**
 * @brief Enables the I2C interface with the default address.
 *
 * This function enables the I2C interface with the default address
 * (0x33) and configures the pins as open-drain outputs.
 *
 * @note The I2C interface is disabled when this function is called.
 * @note The I2C interface is enabled when the function completes.
 *
 * @see begin(uint8_t)
 * @see end()
 */
void TwoWire::begin() {
    ownerAddress_ = OWNER_ADDRESS << 1U;
    i2c_.set_address_format(ownerAddress_, i2c::Address_Format::FORMAT_7BITS, i2c::Bus_Mode::I2C);
    i2c_.set_enable(true);
    i2c_.set_ack_enable(true);
    configurePins();
}

/**
 * @brief Enables the I2C interface with the specified address.
 *
 * This function enables the I2C interface with the specified address and
 * configures the pins as open-drain outputs.
 *
 * @param address The address to use for the I2C interface.
 *        The address must be a 7-bit address and will be shifted left one bit
 *        to allow for the read bit to be added.
 *
 * @note The I2C interface is disabled when this function is called.
 * @note The I2C interface is enabled when the function completes.
 *
 * @see begin()
 * @see end()
 */
void TwoWire::begin(uint8_t address) {
    ownerAddress_ = address << 1U;
    i2c_.set_address_format(ownerAddress_, i2c::Address_Format::FORMAT_7BITS, i2c::Bus_Mode::I2C);
    i2c_.set_enable(true);
    i2c_.set_ack_enable(true);

    configurePins();
    setSlaveInterruptEnable();
}

void TwoWire::end() {
    flush();
    i2c_.set_enable(false);
}

/**
 * @brief Sets the clock speed of the I2C interface.
 *
 * This function sets the clock speed of the I2C interface to the given speed.
 * The clock speed can only be changed while the I2C interface is disabled.
 *
 * @param speed The desired clock speed of the I2C interface in Hz.
 *
 * @note This function is not thread-safe and should only be called from a single
 * thread.
 */
void TwoWire::setClock(uint32_t speed) {
    // Clock can only be changed while the I2C is disabled
    i2c_.set_enable(false);
    i2c_.set_clock_speed_duty(speed, i2c::Duty_Cycle::DTCY_2);
    i2c_.set_enable(true);
}

/**
 * @brief Begin a transmission to a slave device.
 *
 * This function begins a transmission to the slave device with the given
 * address. The address must be a 7-bit address and will be shifted left one
 * bit to allow for the read bit to be added.
 *
 * @param address The address of the slave device to transmit to.
 *
 * @note This function is not thread-safe and should only be called from a single
 * thread.
 *
 * @see endTransmission()
 */
void TwoWire::beginTransmission(uint8_t address) {
    // Save address of target and clear buffer
    transmitting_ = true;
    txAddress_ = address << 1;
    txBuffer_.clear();
}

/**
 * @brief Completes a transmission to a slave device.
 *
 * This function completes a transmission to a slave device that was started
 * with beginTransmission(). The function transmits the data in the transmit
 * buffer in a single transmission. If the stopBit argument is false, the
 * function does not generate a stop condition after transmission and is
 * suitable for sending a stream of data to a slave device.
 *
 * @param stopBit If true, generate a stop condition after transmission.
 *
 * @return 0 on success, 1 on data too long, 2 on slave device NACKs address,
 * 3 on slave device NACKs data, 4 on other error.
 *
 * @note This function is not thread-safe and should only be called from a single
 * thread.
 *
 * @see beginTransmission()
 */
uint8_t TwoWire::endTransmission(bool stopBit) {
    i2c::I2C_Error_Type result = i2c::I2C_Error_Type::OK;

    while (txBuffer_.available()) {
        uint8_t data = txBuffer_.read_char();
        result = masterTransmit(txAddress_, &data, 1U, stopBit);
        if (result != i2c::I2C_Error_Type::OK) {
            if (result == i2c::I2C_Error_Type::NACK_ADDRESS) {
                return 2U;
            } else if (result == i2c::I2C_Error_Type::NACK_DATA) {
                return 3U;
            } else if (result == i2c::I2C_Error_Type::DATA_SIZE_ERROR) {
                return 1U;
            } else {
                return 4U;
            }
        }
    }
    txBuffer_.clear();
    transmitting_ = false;

    return 0U;
}

/**
 * @brief Completes a transmission to a slave device with a stop condition.
 *
 * This function is a convenience function for endTransmission(true).
 *
 * @return 0 on success, 1 on data too long, 2 on slave device NACKs address,
 * 3 on slave device NACKs data, 4 on other error.
 *
 * @see endTransmission(bool)
 */
uint8_t TwoWire::endTransmission() {
    return endTransmission(true);
}

/**
 * @brief Request data from a slave device.
 *
 * This function sends a request to the slave device for the specified number of
 * bytes. If the slave device responds with more bytes than the requested amount,
 * the extra bytes are discarded and not stored in the receive buffer. If the
 * slave device responds with less bytes than the requested amount, the receive
 * buffer will contain the received bytes and the remaining bytes will be
 * filled with zeros. The function returns the actual number of bytes received.
 *
 * If a start condition is sent, the function first sends the slave address with
 * the read bit set. The slave address is the 7-bit address of the slave device
 * plus the read bit set to 1 (0x01). The address is sent in the format:
 * [MSB] [R/W bit] [LSB].
 *
 * The function then waits for the slave device to send the requested number of
 * bytes. If the slave device does not respond, the function will timeout and
 * return 0.
 *
 * If a stop condition is sent, the function will send a stop condition after the
 * last byte is received.
 *
 * @param address The address of the slave device to request data from
 * @param len The number of bytes to request from the slave device
 * @param stopBit If true, a stop condition is sent after the last byte is received
 * @return The actual number of bytes received from the slave device
 */
size_t TwoWire::requestFrom(uint8_t address, size_t len, bool stopBit) {
    if (len > 0U) {
        // Send internal address; this mode allows sending a repeated start to access
        // some devices' internal registers. This function is executed by the hardware
        // TWI module on other processors (for example Due's TWI_IADR and TWI_MMR registers)
        beginTransmission(address);

        // Maximum size is 3 bytes
        if (len > 3U) {
            len = 3U;
        }

        // Write internal register address (MSB first)
        while (len-- > 0U) {
            write((address >> (len * 8U)));
        }
        endTransmission(false);
    }

    if (len > BUFFER_LENGTH) {
        len = BUFFER_LENGTH;    // Clamp to buffer length
    }

    size_t receivedLen = 0U;
    if (masterReceive(address << 1U, rxBuffer_._aucBuffer, len, stopBit) == i2c::I2C_Error_Type::OK) {
        for (size_t i = 0U; i < len; ++i) {
            uint8_t receivedByte = rxBuffer_.read_char();
            rxBuffer_.store_char(receivedByte);
        }
        receivedLen = len;
    }

    return receivedLen;
}

/**
 * @brief Request data bytes from a slave device.
 *
 * @param[in] address 7-bit address of the slave device.
 * @param[in] len Number of bytes to receive.
 * @return Number of bytes received.
 *
 * This function sends a request to a slave device and then receives
 * data bytes from the slave device. The received data bytes are stored
 * in the internal receive buffer. The function returns the number of
 * bytes received.
 */
size_t TwoWire::requestFrom(uint8_t address, size_t len) {
    return requestFrom(address, len, true);
}

/**
 * @brief Send a single byte of data to a slave device.
 *
 * @param[in] data The byte of data to send.
 * @return Number of bytes sent.
 *
 * This function sends a single byte of data to a slave device. If the
 * internal transmit buffer is full, the function returns 0. If the slave
 * device is in receive mode, the function returns 0. If the data byte is
 * sent successfully, the function returns 1.
 */
size_t TwoWire::write(uint8_t data) {
    if (transmitting_) {
        if (txBuffer_.availableForStore()) {
            txBuffer_.store_char(data);
            return 1;
        }
        return 0;   // Buffer is full
    } else {
        // Slave mode
        return (writeSlaveBuffer(&data, 1U) == i2c::I2C_Error_Type::OK) ? 1U : 0U;
    }
}

/**
 * @brief Send a sequence of bytes to a slave device.
 *
 * @param[in] buffer The sequence of bytes to send.
 * @param[in] len The number of bytes to send.
 * @return Number of bytes sent.
 *
 * This function sends a sequence of bytes to a slave device. If the
 * internal transmit buffer is full, the function returns the number of
 * bytes sent so far. If the slave device is in receive mode, the
 * function returns 0. If the data bytes are sent successfully, the
 * function returns the number of bytes sent.
 */
size_t TwoWire::write(const uint8_t* buffer, size_t len) {
    size_t result = len;

    if (transmitting_) {
        for (size_t i = 0U; i < len; ++i) {
            if (txBuffer_.availableForStore() > 0) {
                txBuffer_.store_char(buffer[i]);
            } else {
                result = i;
                break;
            }
        }
    } else {
        // Slave mode
        if (writeSlaveBuffer(buffer, len) != i2c::I2C_Error_Type::OK) {
            result = 0U;
        }
    }

    return result;
}

/**
 * @brief Get the number of bytes available to read from the receive buffer.
 *
 * @return Number of bytes available to read.
 *
 * This function returns the number of bytes available to read from the
 * receive buffer. If the receive buffer is empty, the function returns 0.
 */
int TwoWire::available(void) {
    return static_cast<int>(rxBuffer_.available());
}

/**
 * @brief Read a byte from the receive buffer.
 *
 * @return The byte read from the receive buffer, or -1 if the buffer is empty.
 *
 * This function reads a byte from the receive buffer. If the receive buffer
 * is empty, the function returns -1. If data is available, the function returns
 * the byte read from the receive buffer.
 */
int TwoWire::read() {
    return (rxBuffer_.available() > 0) ? rxBuffer_.read_char() : -1;
}

/**
 * @brief Peek at the next byte in the receive buffer without removing it.
 *
 * @return The next byte in the buffer, or -1 if the buffer is empty.
 *
 * This function returns the next byte in the receive buffer without removing
 * it. If the receive buffer is empty, the function returns -1. If data is
 * available, the function returns the byte peeked from the receive buffer.
 */
int TwoWire::peek(void) {
    return (rxBuffer_.available() > 0) ? rxBuffer_.peek() : -1;
}

/**
 * @brief Wait for all data to be sent.
 *
 * Waits for all data in the transmit buffer to be sent.
 */
void TwoWire::flush() {
    while (txBuffer_.available() > 0) {
        // wait for transmit data to be sent
    }
}

/**
 * @brief Registers a callback function for receiving data from the I2C bus.
 *
 * This function sets the callback function that will be called whenever
 * data is received from the I2C bus. The callback function must be
 * declared as void function(int) and must take a single int argument
 * indicating the number of bytes available in the receive buffer.
 *
 * @param function Pointer to the callback function to be registered.
 */
void TwoWire::onReceive(void(*function)(int)) {
    onReceiveCallback = function;
}

/**
 * @brief Registers a callback function for sending data to the I2C bus.
 *
 * This function sets the callback function that will be called whenever
 * the I2C master requests data from the slave device. The callback function
 * must be declared as void function(void) with no arguments.
 *
 * @param function Pointer to the callback function to be registered.
 */
void TwoWire::onRequest(void(*function)(void)) {
    onRequestCallback = function;
}

/**
 * @brief Transmit data to a slave device as master.
 *
 * This function transmits a sequence of bytes to a slave device as master.
 * If the internal transmit buffer is full, the function returns the number of
 * bytes sent so far. If the slave device NACKs the address or data, the
 * function returns the corresponding error value. If the data bytes are sent
 * successfully, the function returns i2c::I2C_Error_Type::OK.
 *
 * @param address The address of the slave device to transmit to.
 * @param buffer The sequence of bytes to transmit.
 * @param len The number of bytes to transmit.
 * @param stopBit If true, generate a stop condition after transmitting the data.
 * @return i2c::I2C_Error_Type::OK if transmission was successful, otherwise an error value.
 */
i2c::I2C_Error_Type TwoWire::masterTransmit(uint8_t address, uint8_t* buffer, uint8_t len, bool stopBit) {
    i2c::I2C_Error_Type result = i2c::I2C_Error_Type::OK;

    if (len == 0U) {
        return waitForReadyState(address);
    }

    if (checkBusyState() == i2c::I2C_Error_Type::BUSY) {
        return i2c::I2C_Error_Type::BUSY;
    }

    i2c_.generate_start_condition();
    uint32_t timeout = I2C_TIMEOUT_DEFAULT;
    while ((!i2c_.get_flag(i2c::Status_Flags::FLAG_SBSEND)) && (--timeout != 0U));
    if (timeout == 0U) {
        return i2c::I2C_Error_Type::TIMEOUT;
    }

    i2c_.set_direction_address(i2c::Transfer_Direction::TRANSMIT, address);
    timeout = I2C_TIMEOUT_DEFAULT;
    while ((!i2c_.get_flag(i2c::Status_Flags::FLAG_ADDSEND)) && (--timeout != 0U));
    if (timeout == 0U) {
        result = i2c::I2C_Error_Type::NACK_ADDRESS;
    }
    i2c_.clear_flag(i2c::Clear_Flags::FLAG_ADDSEND);

    for (uint32_t i = 0U; i < len; i++) {
        if (result != i2c::I2C_Error_Type::OK) {
            break;
        }
        if (writeByte(buffer[i]) != i2c::I2C_Error_Type::OK) {
            result = i2c::I2C_Error_Type::NACK_DATA;
        }
    }

    if (stopBit) {
        stop();
    }

    return result;
}

/**
 * @brief Master receives data from slave device.
 *
 * This function sends a start condition, transmits the slave address with the
 * read bit set, and then receives the specified amount of data from the slave
 * device. If the stopBit parameter is set to true, a stop condition is sent at
 * the end of the transmission.
 *
 * @param address The address of the slave device.
 * @param buffer The buffer to store the received data in.
 * @param len The number of bytes to receive.
 * @param stopBit Whether a stop condition should be sent at the end of the
 * transmission.
 *
 * @return i2c::I2C_Error_Type::OK on success, an error code otherwise.
 */
i2c::I2C_Error_Type TwoWire::masterReceive(uint8_t address, uint8_t* buffer, uint8_t len, bool stopBit) {
    i2c::I2C_Error_Type result = i2c::I2C_Error_Type::OK;

    if (checkBusyState() == i2c::I2C_Error_Type::BUSY) {
        return i2c::I2C_Error_Type::BUSY;
    }

    if (len == 1U) {
        i2c_.set_ack_enable(false);
    } else if (len == 2U) {
        i2c_.set_ack_position(i2c::ACK_Select::NEXT);
        i2c_.set_ack_enable(false);
    } else {
        i2c_.set_ack_enable(true);
    }

    uint32_t timeout = I2C_TIMEOUT_DEFAULT;

    i2c_.generate_start_condition();
    while ((!i2c_.get_flag(i2c::Status_Flags::FLAG_SBSEND)) && (--timeout != 0U));
    if (timeout == 0U) {
        return i2c::I2C_Error_Type::TIMEOUT;
    }

    i2c_.set_direction_address(i2c::Transfer_Direction::RECEIVE, address);
    timeout = I2C_TIMEOUT_DEFAULT;
    while ((!i2c_.get_flag(i2c::Status_Flags::FLAG_ADDSEND)) && (--timeout != 0U));
    if (timeout == 0U) {
        result = i2c::I2C_Error_Type::NACK_ADDRESS;
    }

    i2c_.clear_flag(i2c::Clear_Flags::FLAG_ADDSEND);

    for (uint32_t i = 0U; i < len; i++) {
        if (result != i2c::I2C_Error_Type::OK) {
            break;
        }
        if (len > 2U && i == static_cast<uint32_t>(len) - 3U) {
            timeout = I2C_TIMEOUT_DEFAULT;
            while ((!i2c_.get_flag(i2c::Status_Flags::FLAG_BTC)) && (--timeout != 0U));
            if (timeout == 0U) {
                result = i2c::I2C_Error_Type::NACK_DATA;
            }
            i2c_.set_ack_enable(false);
        } else if (len == 2U && i == 0U) {
            timeout = I2C_TIMEOUT_DEFAULT;
            while ((!i2c_.get_flag(i2c::Status_Flags::FLAG_BTC)) && (--timeout != 0U));
            if (timeout == 0U) {
                result = i2c::I2C_Error_Type::NACK_DATA;
            }
        }
        timeout = I2C_TIMEOUT_DEFAULT;
        while ((!i2c_.get_flag(i2c::Status_Flags::FLAG_RBNE)) && (--timeout != 0U));
        if (timeout == 0U) {
            result = i2c::I2C_Error_Type::NACK_DATA;
        } else {
            buffer[i] = i2c_.receive_data();
        }
    }

    if (stopBit) {
        stop();
    }

    return result;
}

/**
 * @brief Receive a single byte from the I2C bus as slave.
 *
 * This function will wait until a byte is received from the I2C bus.
 * If the last parameter is true, the ACK will be disabled after receiving the byte.
 * If the last parameter is false, the ACK will be enabled after receiving the byte.
 * The function will block until a byte is received or a timeout occurs.
 * If a timeout occurs, the function will return i2c::I2C_Error_Type::TIMEOUT.
 * Otherwise, the function will return the received byte.
 *
 * @param last If true, the ACK will be disabled after receiving the byte. If false, the ACK will be enabled.
 * @return Received byte if successful, otherwise i2c::I2C_Error_Type::TIMEOUT.
 */
uint8_t TwoWire::readByte(uint32_t last) {
    if (last) {
        i2c_.set_ack_enable(false);
    } else {
        i2c_.set_ack_enable(true);
    }

    // Wait until byte is received
    uint32_t timeout = I2C_TIMEOUT_DEFAULT;
    while ((i2c_.get_flag(i2c::Status_Flags::FLAG_RBNE)) == Clear) {
        if ((timeout--) == 0U) {
            return static_cast<uint8_t>(i2c::I2C_Error_Type::TIMEOUT);
        }
    }

    return i2c_.receive_data();
}

/**
 * @brief Transmit a single byte to the I2C bus as master.
 *
 * This function will transmit a single byte to the I2C bus as master.
 * If the transmission is successful, the function will return i2c::I2C_Error_Type::OK.
 * If the transmission times out, the function will return i2c::I2C_Error_Type::TIMEOUT.
 *
 * @param data The byte of data to be transmitted.
 * @return i2c::I2C_Error_Type::OK if transmission was successful, otherwise i2c::I2C_Error_Type::TIMEOUT.
 */
i2c::I2C_Error_Type TwoWire::writeByte(uint8_t data) {
    i2c_.transmit_data(data);
    // Wait for transmission
    uint32_t timeout = I2C_TIMEOUT_DEFAULT;
    while (((i2c_.get_flag(i2c::Status_Flags::FLAG_TBE)) == Clear) &&
            ((i2c_.get_flag(i2c::Status_Flags::FLAG_BTC)) == Clear)) {
        if ((timeout--) == 0U) {
            return i2c::I2C_Error_Type::TIMEOUT;
        }
    }

    return i2c::I2C_Error_Type::OK;
}

/**
 * @brief Store data in the internal transmit buffer for transmission to a slave device.
 *
 * This function will store the specified amount of data from the given buffer
 * in the internal transmit buffer. If the buffer is full, the function will
 * return i2c::I2C_Error_Type::DATA_SIZE_ERROR. If the data is successfully stored,
 * the function will return i2c::I2C_Error_Type::OK.
 *
 * @param buffer The buffer containing the data to be transmitted.
 * @param len The number of bytes to transmit.
 * @return i2c::I2C_Error_Type::OK if data was stored successfully, otherwise i2c::I2C_Error_Type::DATA_SIZE_ERROR.
 */
i2c::I2C_Error_Type TwoWire::writeSlaveBuffer(const uint8_t* buffer, uint8_t len) {
    if (txBuffer_.availableForStore() > 0) {
        for (uint8_t i = 0U; i < len; i++) {
            txBuffer_.store_char(buffer[i]);
        }
    } else  {
        return i2c::I2C_Error_Type::DATA_SIZE_ERROR;
    }

    return i2c::I2C_Error_Type::OK;
}

/**
 * @brief Generate a stop condition on the I2C bus.
 *
 * This function generates a stop condition on the I2C bus by setting the
 * STOP bit in the control register (CTL0). The function will block until the
 * stop bit is reset by hardware or a timeout occurs.
 *
 * @return i2c::I2C_Error_Type::OK if the stop condition was generated successfully, otherwise i2c::I2C_Error_Type::TIMEOUT.
 */
i2c::I2C_Error_Type TwoWire::stop() {
    i2c_.generate_stop_condition();

    // wait for stop bit reset with timeout
    uint32_t timeout = I2C_TIMEOUT_DEFAULT;
    while (i2c_.get_stop_condition()) {
        if ((timeout--) == 0U) {
            return i2c::I2C_Error_Type::TIMEOUT;
        }
    }

    return i2c::I2C_Error_Type::OK;
}

/**
 * @brief Waits for the bus to be ready to transmit data to a slave device.
 *
 * This function waits for the bus to be ready to transmit data to a slave
 * device. If the bus is busy, the function will return i2c::I2C_Error_Type::BUSY.
 * If the bus is ready, the function will return i2c::I2C_Error_Type::OK.
 *
 * @param address The address of the slave device to wait for.
 * @return i2c::I2C_Error_Type::OK if the bus is ready, otherwise i2c::I2C_Error_Type::BUSY or i2c::I2C_Error_Type::TIMEOUT.
 */
i2c::I2C_Error_Type TwoWire::waitForReadyState(uint8_t address) {
    i2c::I2C_Error_Type result = i2c::I2C_Error_Type::OK;
    bool addsend = false;
    bool aerr = false;

    if (checkBusyState() == i2c::I2C_Error_Type::BUSY) {
        return i2c::I2C_Error_Type::BUSY;
    }

    uint32_t timeout = I2C_TIMEOUT_DEFAULT;
    i2c_.generate_start_condition();
    while ((!i2c_.get_flag(i2c::Status_Flags::FLAG_SBSEND)) && (--timeout != 0U));
    if (timeout == 0U) {
        result = i2c::I2C_Error_Type::TIMEOUT;
    }

    i2c_.set_direction_address(i2c::Transfer_Direction::TRANSMIT, address);
    timeout = I2C_TIMEOUT_DEFAULT;

    do {
        addsend = i2c_.get_flag(i2c::Status_Flags::FLAG_ADDSEND);
        aerr = i2c_.get_flag(i2c::Status_Flags::FLAG_AERR);
    } while ((addsend | aerr) && (--timeout != 0U));

    if (timeout == 0U) {
        result = i2c::I2C_Error_Type::TIMEOUT;
    } else if (addsend) {
        i2c_.clear_flag(i2c::Clear_Flags::FLAG_ADDSEND);
        i2c_.generate_stop_condition();
        result = i2c::I2C_Error_Type::OK;
    } else {
        i2c_.clear_flag(i2c::Clear_Flags::FLAG_AERR);
        result = i2c::I2C_Error_Type::NACK_ADDRESS;
    }

    i2c_.generate_stop_condition();

    timeout = I2C_TIMEOUT_DEFAULT;

    while (i2c_.get_stop_condition() != 0U) {
        if ((timeout--) == 0U) {
            return i2c::I2C_Error_Type::TIMEOUT;
        }
    }

    return result;
}

/**
 * @brief Check if the I2C bus is busy.
 *
 * This function checks the I2C bus busy flag and returns i2c::I2C_Error_Type::BUSY if the bus is busy.
 * If the bus is not busy, the function returns i2c::I2C_Error_Type::OK. If the timeout elapses before the bus
 * is not busy, the function returns i2c::I2C_Error_Type::TIMEOUT.
 *
 * @return i2c::I2C_Error_Type::OK if the bus is not busy, otherwise i2c::I2C_Error_Type::BUSY or i2c::I2C_Error_Type::TIMEOUT.
 */
i2c::I2C_Error_Type TwoWire::checkBusyState() {
    uint32_t timeout = I2C_TIMEOUT_DEFAULT;
    while ((i2c_.get_flag(i2c::Status_Flags::FLAG_I2CBSY)) && (--timeout != 0U));
    if (timeout == 0U) {
        return i2c::I2C_Error_Type::BUSY;
    }

    return i2c::I2C_Error_Type::OK;
}

/**
 * @brief Enables interrupts for the slave device.
 *
 * This function enables interrupts for the slave device in the NVIC and
 * in the I2C peripheral. The interrupts enabled are the event interrupts,
 * error interrupts, and buffer interrupts.
 *
 * @note This function should be called before calling the begin() function.
 *
 * @see begin()
 */
void TwoWire::setSlaveInterruptEnable() {
    switch (base_) {
        case i2c::I2C_Base::I2C0_BASE:
            CORTEX_I.set_nvic_priority(I2C0_EV_IRQn, 2U, 3U);
            NVIC_EnableIRQ(I2C0_EV_IRQn);
            CORTEX_I.set_nvic_priority(I2C0_ER_IRQn, 2U, 2U);
            NVIC_EnableIRQ(I2C0_ER_IRQn);
            break;
        case i2c::I2C_Base::I2C1_BASE:
            CORTEX_I.set_nvic_priority(I2C1_EV_IRQn, 2U, 3U);
            NVIC_EnableIRQ(I2C1_EV_IRQn);
            CORTEX_I.set_nvic_priority(I2C1_ER_IRQn, 2U, 2U);
            NVIC_EnableIRQ(I2C1_ER_IRQn);
            break;
        default:
            break;
    }

    i2c_.set_interrupt_enable(i2c::Interrupt_Type::INTR_ERR, true);
    i2c_.set_interrupt_enable(i2c::Interrupt_Type::INTR_EV, true);
    i2c_.set_interrupt_enable(i2c::Interrupt_Type::INTR_BUF, true);
}

/**
 * @brief Handles errors in the I2C peripheral.
 *
 * This function checks the I2C status flags for error conditions and
 * clears the corresponding flags if an error is detected.
 *
 * The errors handled by this function are:
 * - Arbitration lost (INTR_FLAG_LOSTARB)
 * - Bus error (INTR_FLAG_BERR)
 * - CRC mismatch (INTR_FLAG_PECERR)
 * - Overrun or underrun when SCL stretch is disabled (INTR_FLAG_OUERR)
 * - SMBus alert (INTR_FLAG_SMBALT)
 * - SMBus mode bus timeout (INTR_FLAG_SMBTO)
 */
void TwoWire::errorHandler() {
    if (i2c_.get_interrupt_flag(i2c::Interrupt_Flags::INTR_FLAG_AERR)) {
        i2c_.clear_interrupt_flag(i2c::Clear_Flags::FLAG_AERR);
    }

    // SMBus alert
    if (i2c_.get_interrupt_flag(i2c::Interrupt_Flags::INTR_FLAG_SMBALT)) {
        i2c_.clear_interrupt_flag(i2c::Clear_Flags::FLAG_SMBALT);
    }

    // SMBus mode bus timeout
    if (i2c_.get_interrupt_flag(i2c::Interrupt_Flags::INTR_FLAG_SMBTO)) {
        i2c_.clear_interrupt_flag(i2c::Clear_Flags::FLAG_SMBTO);
    }

    // Overrun or underrun when SCL stretch is disabled
    if (i2c_.get_interrupt_flag(i2c::Interrupt_Flags::INTR_FLAG_OUERR)) {
        i2c_.clear_interrupt_flag(i2c::Clear_Flags::FLAG_OUERR);
    }

    // Arbitration lost
    if (i2c_.get_interrupt_flag(i2c::Interrupt_Flags::INTR_FLAG_LOSTARB)) {
        i2c_.clear_interrupt_flag(i2c::Clear_Flags::FLAG_LOSTARB);
    }

    // Bus error
    if (i2c_.get_interrupt_flag(i2c::Interrupt_Flags::INTR_FLAG_BERR)) {
        i2c_.clear_interrupt_flag(i2c::Clear_Flags::FLAG_BERR);
    }

    // CRC mismatch
    if (i2c_.get_interrupt_flag(i2c::Interrupt_Flags::INTR_FLAG_PECERR)) {
        i2c_.clear_interrupt_flag(i2c::Clear_Flags::FLAG_PECERR);
    }
}

/**
 * Interrupt handler for the I2C bus.
 *
 * This function is called when an interrupt occurs for the I2C bus. The
 * interrupt can be triggered by the following events:
 *
 * - A slave address has been sent and matched (ADDSEND)
 * - The transmit buffer is empty (TBE)
 * - The receive buffer is not empty (RBNE)
 * - A stop condition has been detected (STPDET)
 *
 * The function will clear the interrupt flag and perform the necessary actions
 * to handle the interrupt. If the interrupt is caused by a slave address match,
 * the function will reset the RX count and check if the bus is in transmit mode.
 * If it is, it will clear the TX buffer and call the registered transmit callback.
 *
 * If the interrupt is caused by the transmit buffer being empty, the function
 * will check if there is data available in the TX buffer and send it if so.
 *
 * If the interrupt is caused by the receive buffer not being empty, the function
 * will read the data from the bus and store it in the RX buffer.
 *
 * If the interrupt is caused by a stop condition being detected, the function
 * will call the registered slave receive callback and clear the RX buffer.
 */
void TwoWire::interruptHandler() {
    if (i2c_.get_interrupt_flag(i2c::Interrupt_Flags::INTR_FLAG_ADDSEND)) {
        i2c_.clear_interrupt_flag(i2c::Clear_Flags::FLAG_ADDSEND);

        // Reset RX count and check transmit mode
        rxBuffer_.clear();
        if (i2c_.get_flag(i2c::Status_Flags::FLAG_TRS)) {
            txBuffer_.clear();
            if (onRequestCallback) {
                onRequestCallback();   // Call the registered transmit callback
            }
        }
    } else if (i2c_.get_interrupt_flag(i2c::Interrupt_Flags::INTR_FLAG_TBE) &&
               !i2c_.get_interrupt_flag(i2c::Interrupt_Flags::INTR_FLAG_AERR)) {
        // Transmit Buffer Empty: Send next byte if available
        if (txBuffer_.available() > 0) {
            i2c_.transmit_data(txBuffer_.read_char());
        }
    } else if (i2c_.get_interrupt_flag(i2c::Interrupt_Flags::INTR_FLAG_RBNE)) {
        // Receive Buffer Not Empty: Read data and store in RX buffer
        if (rxBuffer_.availableForStore() > 0) {
            rxBuffer_.store_char(i2c_.receive_data());
        } else {
            // RX buffer too small
        }
    } else if (i2c_.get_interrupt_flag(i2c::Interrupt_Flags::INTR_FLAG_STPDET)) {
        i2c_.set_enable(true);
        // On stop detection, call the slave receive callback
        if (!i2c_.get_flag(i2c::Status_Flags::FLAG_TRS)) {
            if (onReceiveCallback) {
                onReceiveCallback(rxBuffer_.available());
            }
        }
        rxBuffer_.clear();
    }
}

/**
 * @brief Configures the SDA and SCL pins for I2C communication.
 *
 * If a custom pin is specified for SDA or SCL, the function will configure that
 * pin. Otherwise, it will use the pin mapping defined in the Variant.h file.
 *
 * The function will set the pin mode and speed according to the pin operations
 * defined for the I2C peripheral and the pin.
 *
 * The function will also check if a remap is required for the pin and apply it
 * if necessary.
 */
void TwoWire::configurePins() {
    // SDA pin
    if (customSdaPin_ != NO_PIN) {
        pinOpsPinout(I2C_SDA_PinOps, customSdaPin_);
    } else {
        auto sdaPinOps = getPinOpsByPeripheral(I2C_SDA_PinOps, base_);
        if (sdaPinOps == invalidPinOps) {
            return;
        }
        auto sdaMode = getPackedPinMode(sdaPinOps.packedPinOps);
        auto sdaSpeed = getPackedPinSpeed(sdaPinOps.packedPinOps);
        // Initialize pin
        auto& sdaPort = gpio::GPIO::get_instance(sdaPinOps.port).value();
        sdaPort.set_pin_mode(sdaPinOps.pin, sdaMode, sdaSpeed);
        // Check remap
        auto sdaRemap = getPackedPinRemap(sdaPinOps.packedPinOps);
        if (sdaRemap != gpio::Pin_Remap_Select::NO_REMAP) {
            AFIO_I.set_remap(sdaRemap);
        }
    }

    // SCL pin
    if (customSclPin_ != NO_PIN) {
        pinOpsPinout(I2C_SCL_PinOps, customSclPin_);
    } else {
        auto sclPinOps = getPinOpsByPeripheral(I2C_SCL_PinOps, base_);
        if (sclPinOps == invalidPinOps) {
            return;
        }
        auto sclMode = getPackedPinMode(sclPinOps.packedPinOps);
        auto sclSpeed = getPackedPinSpeed(sclPinOps.packedPinOps);
        // Initialize pin
        auto& sclPort = gpio::GPIO::get_instance(sclPinOps.port).value();
        sclPort.set_pin_mode(sclPinOps.pin, sclMode, sclSpeed);
        // Check remap
        auto sclRemap = getPackedPinRemap(sclPinOps.packedPinOps);
        if (sclRemap != gpio::Pin_Remap_Select::NO_REMAP) {
            AFIO_I.set_remap(sclRemap);
        }
    }
}


} // namespace arduino

#ifdef SUPPORT_I2C0
    arduino::TwoWire& Wire = arduino::TwoWire::get_instance(i2c::I2C_Base::I2C0_BASE);
#endif

#ifdef SUPPORT_I2C1
    arduino::TwoWire& Wire1 = arduino::TwoWire::get_instance(i2c::I2C_Base::I2C1_BASE);
#endif


extern "C" {

#ifdef SUPPORT_I2C0
    void I2C0_EV_IRQHandler(void) {
        Wire.interruptHandler();
    }
    void I2C0_ER_IRQHandler(void) {
        Wire.errorHandler();
    }
#endif

#ifdef SUPPORT_I2C1
    void I2C1_EV_IRQHandler(void) {
        Wire1.interruptHandler();
    }
    void I2C1_ER_IRQHandler(void) {
        Wire1.errorHandler();
    }
#endif

} // extern "C"
