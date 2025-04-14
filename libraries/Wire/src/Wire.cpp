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
#include <PinOpsMap.hpp>
#include <PinOps.hpp>
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
    uint8_t txLength = txBuffer_.available();

    if (txLength == 0) {
        // Nothing to transmit
        transmitting_ = false;
        return 0U;
    }

    // Transmit all bytes in the buffer
    i2c::I2C_Error_Type result = masterTransmit(txAddress_, txLength, stopBit);

    // Clear the buffer and reset transmitting flag
    txBuffer_.clear();
    transmitting_ = false;

    // Map error codes to Arduino Wire error codes
    switch (result) {
        case i2c::I2C_Error_Type::OK:
            return 0U;
        case i2c::I2C_Error_Type::DATA_SIZE_ERROR:
            return 1U;
        case i2c::I2C_Error_Type::NACK_ADDRESS:
            return 2U;
        case i2c::I2C_Error_Type::NACK_DATA:
            return 3U;
        default:
            return 4U;
    }
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
 * This function initiates a data request from a specified slave device on the
 * I2C bus. It sends a request to the device with the given address and attempts
 * to receive the specified number of bytes. The received data is stored in the
 * internal receive buffer, and the function returns the number of bytes successfully
 * received. If the requested length exceeds the buffer size, it will be clamped to
 * the buffer size.
 *
 * @param address The 7-bit address of the slave device.
 * @param len The number of bytes to request.
 * @param stopBit If true, a stop condition will be sent after the data is received.
 * 
 * @return The number of bytes successfully received.
 */
size_t TwoWire::requestFrom(uint8_t address, size_t len, bool stopBit) {
    // Check for valid length
    if (len == 0U) {
        return 0;
    }

    // Clamp length to buffer size
    if (len > BUFFER_LENGTH) {
        len = BUFFER_LENGTH;
    }

    // Receive data from the specified address
    if (masterReceive(address << 1U, len, stopBit) == i2c::I2C_Error_Type::OK) {
        return len;
    }

    return 0;
}

/**
 * @brief Request data from a slave device.
 *
 * This function is a wrapper around requestFrom(address, len, true) and is
 * provided for convenience. It will send a stop condition after the data is
 * received.
 *
 * @param address The 7-bit address of the slave device.
 * @param len The number of bytes to request.
 * 
 * @return The number of bytes successfully received.
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
        // Master mode - store data in transmit buffer
        if (txBuffer_.availableForStore()) {
            txBuffer_.store_char(data);
            return 1;
        }
        // Buffer is full
        return 0;
    } else {
        // Slave mode - directly write to slave buffer
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
    // Check for null buffer or zero length
    if (buffer == nullptr || len == 0) {
        return 0;
    }

    if (transmitting_) {
        // Master mode - store data in transmit buffer
        size_t i;
        for (i = 0U; i < len; ++i) {
            if (!txBuffer_.availableForStore()) {
                // Buffer is full, return number of bytes stored so far
                return i;
            }
            txBuffer_.store_char(buffer[i]);
        }
        return i; // Return number of bytes stored
    } else {
        // Slave mode - directly write to slave buffer
        return (writeSlaveBuffer(buffer, len) == i2c::I2C_Error_Type::OK) ? len : 0;
    }
}

/**
 * @brief Get the number of bytes available to read from the receive buffer.
 *
 * @return Number of bytes available to read.
 *
 * This function returns the number of bytes available to read from the
 * receive buffer. If the receive buffer is empty, the function returns 0.
 */
int TwoWire::available() {
    return rxBuffer_.available();
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
    return rxBuffer_.read_char();
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
int TwoWire::peek() {
    return rxBuffer_.peek();
}

/**
 * @brief Wait for all data to be sent.
 *
 * Waits for all data in the transmit buffer to be sent.
 */
void TwoWire::flush() {
    while (txBuffer_.available() > 0) {
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
i2c::I2C_Error_Type TwoWire::masterTransmit(uint8_t address, uint8_t len, bool stopBit) {
    // Special case for zero-length transmissions
    if (len == 0U) {
        return waitForReadyState(address);
    }

    // Check if bus is busy before starting transmission
    if (checkBusyState() == i2c::I2C_Error_Type::BUSY) {
        return i2c::I2C_Error_Type::BUSY;
    }

    // Generate start condition and wait for it to be sent
    i2c_.generate_start_condition();
    uint32_t timeout = I2C_TIMEOUT_DEFAULT;
    while (!i2c_.get_flag(i2c::Status_Flags::FLAG_SBSEND)) {
        if (--timeout == 0U) {
            return i2c::I2C_Error_Type::TIMEOUT;
        }
    }

    // Set direction and address, then wait for address to be sent
    i2c_.set_direction_address(i2c::Transfer_Direction::TRANSMIT, address);
    timeout = I2C_TIMEOUT_DEFAULT;
    while (!i2c_.get_flag(i2c::Status_Flags::FLAG_ADDSEND)) {
        if (--timeout == 0U) {
            return i2c::I2C_Error_Type::NACK_ADDRESS;
        }
    }

    // Clear address sent flag
    i2c_.clear_flag(i2c::Clear_Flags::FLAG_ADDSEND);

    // Transmit specified number of bytes from the buffer
    for (uint8_t i = 0U; i < len; i++) {
        if (txBuffer_.available() == 0) {
            // No more data in buffer
            break;
        }

        uint8_t data = txBuffer_.read_char();
        if (writeByte(data) != i2c::I2C_Error_Type::OK) {
            // Generate stop condition if requested
            if (stopBit) {
                stop();
            }
            return i2c::I2C_Error_Type::NACK_DATA;
        }
    }

    // Generate stop condition if requested
    if (stopBit) {
        i2c::I2C_Error_Type stopResult = stop();
        if (stopResult != i2c::I2C_Error_Type::OK) {
            return stopResult;
        }
    }

    return i2c::I2C_Error_Type::OK;
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
 * @param len The number of bytes to receive.
 * @param stopBit Whether a stop condition should be sent at the end of the
 * transmission.
 *
 * @return i2c::I2C_Error_Type::OK on success, an error code otherwise.
 */
i2c::I2C_Error_Type TwoWire::masterReceive(uint8_t address, uint8_t len, bool stopBit) {
    // Check if bus is busy before starting reception
    if (checkBusyState() == i2c::I2C_Error_Type::BUSY) {
        return i2c::I2C_Error_Type::BUSY;
    }

    // Clear the buffer before receiving new data
    rxBuffer_.clear();

    // Check if there's enough space in the buffer
    if (rxBuffer_.availableForStore() < len) {
        return i2c::I2C_Error_Type::DATA_SIZE_ERROR;
    }

    // Configure ACK behavior based on number of bytes to receive
    if (len == 1U) {
        // For single byte, disable ACK before reception
        i2c_.set_ack_enable(false);
    } else if (len == 2U) {
        // For two bytes, set ACK position to next and disable ACK
        i2c_.set_ack_position(i2c::ACK_Select::NEXT);
        i2c_.set_ack_enable(false);
    } else {
        // For more than two bytes, enable ACK
        i2c_.set_ack_enable(true);
    }

    // Generate start condition and wait for it to be sent
    i2c_.generate_start_condition();
    uint32_t timeout = I2C_TIMEOUT_DEFAULT;
    while (!i2c_.get_flag(i2c::Status_Flags::FLAG_SBSEND)) {
        if (--timeout == 0U) {
            return i2c::I2C_Error_Type::TIMEOUT;
        }
    }

    // Set direction to receive and send address
    i2c_.set_direction_address(i2c::Transfer_Direction::RECEIVE, address);

    // Wait for address to be sent
    timeout = I2C_TIMEOUT_DEFAULT;
    while (!i2c_.get_flag(i2c::Status_Flags::FLAG_ADDSEND)) {
        if (--timeout == 0U) {
            return i2c::I2C_Error_Type::NACK_ADDRESS;
        }
    }

    // Clear address sent flag
    i2c_.clear_flag(i2c::Clear_Flags::FLAG_ADDSEND);

    // Receive all bytes
    for (uint32_t i = 0U; i < len; i++) {
        // Special handling for multi-byte transfers
        if (len > 2U && i == static_cast<uint32_t>(len) - 3U) {
            // For transfers > 2 bytes, wait for byte transfer complete before
            // disabling ACK on the third-to-last byte
            timeout = I2C_TIMEOUT_DEFAULT;
            while (!i2c_.get_flag(i2c::Status_Flags::FLAG_BTC)) {
                if (--timeout == 0U) {
                    if (stopBit) {
                        stop();
                    }
                    return i2c::I2C_Error_Type::NACK_DATA;
                }
            }
            i2c_.set_ack_enable(false);
        } else if (len == 2U && i == 0U) {
            // For 2-byte transfers, wait for byte transfer complete after first byte
            timeout = I2C_TIMEOUT_DEFAULT;
            while (!i2c_.get_flag(i2c::Status_Flags::FLAG_BTC)) {
                if (--timeout == 0U) {
                    if (stopBit) {
                        stop();
                    }
                    return i2c::I2C_Error_Type::NACK_DATA;
                }
            }
        }

        // Wait for receive buffer not empty
        timeout = I2C_TIMEOUT_DEFAULT;
        while (!i2c_.get_flag(i2c::Status_Flags::FLAG_RBNE)) {
            if (--timeout == 0U) {
                if (stopBit) {
                    stop();
                }
                return i2c::I2C_Error_Type::NACK_DATA;
            }
        }

        // Read data from receive buffer and store in ring buffer
        uint8_t receivedByte = i2c_.receive_data();
        rxBuffer_.store_char(receivedByte);
    }

    // Generate stop condition if requested
    if (stopBit) {
        i2c::I2C_Error_Type stopResult = stop();
        if (stopResult != i2c::I2C_Error_Type::OK) {
            return stopResult;
        }
    }

    return i2c::I2C_Error_Type::OK;
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
    // Transmit the data byte
    i2c_.transmit_data(data);

    // Wait for either transmit buffer empty or byte transfer complete
    uint32_t timeout = I2C_TIMEOUT_DEFAULT;
    while (true) {
        // Check if either flag is set
        if (i2c_.get_flag(i2c::Status_Flags::FLAG_TBE) || 
            i2c_.get_flag(i2c::Status_Flags::FLAG_BTC)) {
            break;
        }

        // Check for timeout
        if (--timeout == 0U) {
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
    // Check if there's enough space in the ring buffer
    if (txBuffer_.availableForStore() >= len) {
        for (uint8_t i = 0U; i < len; i++) {
            txBuffer_.store_char(buffer[i]);
        }
        return i2c::I2C_Error_Type::OK;
    }

    return i2c::I2C_Error_Type::DATA_SIZE_ERROR;
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
        if (--timeout == 0U) {
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
    // Check if bus is busy
    if (checkBusyState() == i2c::I2C_Error_Type::BUSY) {
        return i2c::I2C_Error_Type::BUSY;
    }

    // Generate start condition
    i2c_.generate_start_condition();

    // Wait for start bit to be sent
    uint32_t timeout = I2C_TIMEOUT_DEFAULT;
    while (!i2c_.get_flag(i2c::Status_Flags::FLAG_SBSEND)) {
        if (--timeout == 0U) {
            return i2c::I2C_Error_Type::TIMEOUT;
        }
    }

    // Set direction and address
    i2c_.set_direction_address(i2c::Transfer_Direction::TRANSMIT, address);

    // Wait for address to be sent or error
    timeout = I2C_TIMEOUT_DEFAULT;
    bool addsend = false;
    bool aerr = false;

    do {
        addsend = i2c_.get_flag(i2c::Status_Flags::FLAG_ADDSEND);
        aerr = i2c_.get_flag(i2c::Status_Flags::FLAG_AERR);

        if (--timeout == 0U) {
            // Generate stop condition
            i2c_.generate_stop_condition();

            // Wait for stop condition with timeout
            uint32_t stopTimeout = I2C_TIMEOUT_DEFAULT;
            while (i2c_.get_stop_condition() != 0U) {
                if (--stopTimeout == 0U) {
                    return i2c::I2C_Error_Type::TIMEOUT;
                }
            }

            return i2c::I2C_Error_Type::TIMEOUT;
        }
    } while (!(addsend || aerr)); // Continue until either flag is set

    // Handle result based on which flag was set
    if (addsend) {
        i2c_.clear_flag(i2c::Clear_Flags::FLAG_ADDSEND);
        i2c_.generate_stop_condition();

        // Wait for stop condition with timeout
        timeout = I2C_TIMEOUT_DEFAULT;
        while (i2c_.get_stop_condition() != 0U) {
            if (--timeout == 0U) {
                return i2c::I2C_Error_Type::TIMEOUT;
            }
        }

        return i2c::I2C_Error_Type::OK;
    } else {
        // Must be aerr
        i2c_.clear_flag(i2c::Clear_Flags::FLAG_AERR);
        i2c_.generate_stop_condition();

        // Wait for stop condition with timeout
        timeout = I2C_TIMEOUT_DEFAULT;
        while (i2c_.get_stop_condition() != 0U) {
            if (--timeout == 0U) {
                return i2c::I2C_Error_Type::TIMEOUT;
            }
        }

        return i2c::I2C_Error_Type::NACK_ADDRESS;
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
    // Wait until the I2C bus is not busy or timeout occurs
    uint32_t timeout = I2C_TIMEOUT_DEFAULT;

    while (i2c_.get_flag(i2c::Status_Flags::FLAG_I2CBSY)) {
        if (--timeout == 0U) {
            return i2c::I2C_Error_Type::BUSY;
        }
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
    // Configure NVIC priorities and enable interrupts based on I2C instance
    switch (base_) {
        case i2c::I2C_Base::I2C0_BASE:
            // Set priorities for I2C0 event and error interrupts
            CORTEX_I.set_nvic_priority(I2C0_EV_IRQn, 2U, 3U);
            CORTEX_I.set_nvic_priority(I2C0_ER_IRQn, 2U, 2U);
            // Enable I2C0 event and error interrupts
            NVIC_EnableIRQ(I2C0_EV_IRQn);
            NVIC_EnableIRQ(I2C0_ER_IRQn);
            break;
        case i2c::I2C_Base::I2C1_BASE:
            // Set priorities for I2C1 event and error interrupts
            CORTEX_I.set_nvic_priority(I2C1_EV_IRQn, 2U, 3U);
            CORTEX_I.set_nvic_priority(I2C1_ER_IRQn, 2U, 2U);
            // Enable I2C1 event and error interrupts
            NVIC_EnableIRQ(I2C1_EV_IRQn);
            NVIC_EnableIRQ(I2C1_ER_IRQn);
            break;
        default:
            // Invalid I2C base, do nothing
            return;
    }

    // Enable I2C peripheral interrupts
    i2c_.set_interrupt_enable(i2c::Interrupt_Type::INTR_ERR, true);
    i2c_.set_interrupt_enable(i2c::Interrupt_Type::INTR_EV, true);
    i2c_.set_interrupt_enable(i2c::Interrupt_Type::INTR_BUF, true);
}

/**
 * @brief Error handler for I2C events.
 *
 * This function is called when an error occurs on the I2C bus. It checks
 * the interrupt flags and clears them if set.
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
