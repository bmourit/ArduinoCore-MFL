/*
 * SoftwareSerial.cpp (formerly NewSoftSerial.cpp)
 *
 * Multi-instance software serial library for Arduino/Wiring
 * -- Interrupt-driven receive and other improvements by ladyada
 *    (http://ladyada.net)
 * -- Tuning, circular buffer, derivation from class Print/Stream,
 *    multi-instance support, porting to 8MHz processors,
 *    various optimizations, PROGMEM delay tables, inverse logic and
 *    direct port writing by Mikal Hart (http://www.arduiniana.org)
 * -- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
 * -- 20MHz processor support by Garrett Mace (http://www.macetech.com)
 * -- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)
 * -- MFL platform adaptation by B. Mouritsen
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
 *
 * The latest version of this library can always be found at
 * http://arduiniana.org.
 */

#include <Arduino.h>
#include <GeneralTimer.h>
#include <PinOps.hpp>
#include <variant.h>
#include "SoftwareSerial.h"

constexpr timer::TIMER_Base SoftwareSerial::TIMER_SERIAL_BASE;
SoftwareSerial *SoftwareSerial::activeListener = nullptr;
SoftwareSerial *volatile SoftwareSerial::txActive = nullptr;
SoftwareSerial *volatile SoftwareSerial::rxActive = nullptr;
int32_t SoftwareSerial::txTickCount = 0;            // OVERSAMPLE ticks needed for a bit
int32_t volatile SoftwareSerial::rxTickCount = 0;   // OVERSAMPLE ticks needed for a bit
uint32_t SoftwareSerial::txBuffer = 0;
int32_t SoftwareSerial::txBitCount = 0;
uint32_t SoftwareSerial::rxBuffer = 0;
int32_t SoftwareSerial::rxBitCount = -1;            // Waiting for start bit
uint32_t SoftwareSerial::currentSpeed = 0;

SoftwareSerial::SoftwareSerial(pin_size_t rxPin, pin_size_t txPin, bool inverseLogic /* = false */) :
    rxPin_(rxPin),
    txPin_(txPin),
    rxPort_(port_pin_map[rxPin].port),
    rxPinNumber_(port_pin_map[rxPin].pin),
    txPort_(port_pin_map[txPin].port),
    txPinNumber_(port_pin_map[txPin].pin),
    inverseLogic_(inverseLogic),
    halfDuplex_(rxPin == txPin),
    outputPending_(false),
    speed_(0),
    bufferOverflow_(false)
{
    if ((rxPin >= TOTAL_PIN_COUNT) || (txPin >= TOTAL_PIN_COUNT) || 
            rxPort_ == gpio::GPIO_Base::INVALID || txPort_ == gpio::GPIO_Base::INVALID ||
            rxPinNumber_ == gpio::Pin_Number::INVALID || txPinNumber_ == gpio::Pin_Number::INVALID) {
        _ErrorHandler("ERROR: invalid pin number", -1);
    }
}

SoftwareSerial::~SoftwareSerial() {
    end();
}

/**
 * @brief Set the speed of the SoftwareSerial port.
 *
 * This function should be called before any other methods. It will set the speed
 * of the SoftwareSerial port by configuring the timer and interrupts.
 *
 * @param speed The baud rate to use, in bits per second.
 *
 * @note The actual speed used may be slightly different from the requested
 * speed, due to the limitations of the timer.
 *
 * @note The timer is configured to generate an interrupt every time the counter
 * reaches the compare value. This means that the timer will generate an interrupt
 * at the rate of the requested speed.
 */
void SoftwareSerial::setSpeed(uint32_t speed) {
    if (speed == currentSpeed) {
        return;  // Early return if speed hasn't changed
    }

    auto& timer = GeneralTimer::get_instance(TIMER_SERIAL_BASE);
    timer.stop();  // Stop timer before reconfiguring

    if (speed != 0) {
        // Calculate prescaler and compare value for the timer
        uint32_t clockFreq = timer.getTimerClockFrequency();
        int prescaler = 1;
        uint32_t compareValue = clockFreq / (speed * OVERSAMPLE);

        // Find appropriate prescaler to fit compareValue within 16-bit range
        while (compareValue >= UINT16_MAX) {
            prescaler *= 2;
            compareValue = (clockFreq / prescaler) / (speed * OVERSAMPLE);

            // Safety check to prevent infinite loop
            if (prescaler > 65536) {
                // Prescaler too large, use maximum possible values
                prescaler = 65536;
                compareValue = UINT16_MAX - 1;
                break;
            }
        }

        // Configure the timer
        timer.setPrescaler(prescaler);
        timer.setRolloverValue(compareValue);
        timer.setCounter(0, TimerFormat::TICK);
        timer.attachInterrupt(&handleInterrupt);
        timer.start();
    } else {
        timer.detachInterrupt();
    }

    currentSpeed = speed;
}


/**
 * @brief Activates the current SoftwareSerial instance as the active listener.
 *
 * This function sets the current instance as the active listener, allowing it to 
 * receive data on its RX pin. If there is an existing active listener, it will be 
 * stopped first. The function ensures that any ongoing transmission is completed 
 * before changing the active listener to avoid data corruption. It then prepares 
 * the instance for receiving data by resetting the receive tick count and bit 
 * count, and setting the speed for the timer.
 *
 * @return True if the current instance successfully becomes the active listener, 
 *         false if it is already the active listener.
 */
bool SoftwareSerial::listen() {
    if (activeListener == this) {
        return false;   // Early return if already listening
    }

    // Wait for any transmit to complete as we may change speed
    while (txActive) {
    }

    // Stop the previous listener
    if (activeListener) {
        activeListener->stopListening();
    }

    // Initialize receive state
    rxTickCount = 1;    // Next interrupt will decrease rxTickCount to 0 which means RX pin level will be considered
    rxBitCount = -1;    // Waiting for start bit

    // Configure timer with our speed
    setSpeed(speed_);

    // Set this instance as the active listener
    activeListener = this;

    // In full-duplex mode, immediately set as active receiver
    if (!halfDuplex_) {
        rxActive = this;
    }

    return true;
}

/**
 * @brief Disables the current SoftwareSerial instance as the active listener.
 *
 * This function stops the current instance from listening on its RX pin and 
 * allows another instance to become the active listener. It first waits for any 
 * ongoing transmission to complete to avoid data corruption. It then sets the 
 * active listener to nullptr and turns off the interrupts. The function returns 
 * true if the current instance was the active listener and false otherwise.
 */
bool SoftwareSerial::stopListening() {
    if (activeListener != this) {
        return false;
    }

    // Wait for any output to complete
    while (txActive) {
    }

    // In half-duplex mode, switch back to TX mode
    if (halfDuplex_) {
        setRXTX(false);
    }

    // Clear listener state
    activeListener = nullptr;
    rxActive = nullptr;

    // Stop the timer interrupts
    setSpeed(0);

    return true;
}

/**
 * @brief Initializes the SoftwareSerial instance with the specified baud rate.
 *
 * This function sets the baud rate for the SoftwareSerial instance and prepares
 * the pins for transmission and reception. If the instance is not in half-duplex
 * mode, it also activates the instance as the active listener for data reception.
 *
 * @param speed The baud rate to use for the SoftwareSerial instance. If the
 *              FORCE_BAUD_RATE macro is defined, it overrides this parameter.
 */
void SoftwareSerial::begin(long speed) {
#ifdef FORCE_BAUD_RATE
    speed = FORCE_BAUD_RATE;
#endif

    speed_ = speed;

    // Configure pins based on mode
    setTX();  // Always configure TX pin

    if (!halfDuplex_) {
        // In full-duplex mode, configure RX pin and start listening
        setRX();
        listen();
    }
    // In half-duplex mode, TX is already configured and we'll switch to RX when needed
}

/**
 * @brief Disables the SoftwareSerial instance as the active listener.
 *
 * This function is used to stop the SoftwareSerial instance from listening on
 * its RX pin and allows another instance to become the active listener. It
 * first waits for any ongoing transmission to complete and then stops the
 * interrupts. The function returns true if the current instance was the active
 * listener and false otherwise.
 */
void SoftwareSerial::end() {
    stopListening();
}

/**
 * @brief Reads a byte from the receive buffer.
 *
 * This function retrieves a byte from the receive buffer. If the buffer
 * is empty, it returns -1. Otherwise, it returns the next byte in the buffer.
 *
 * @return The byte read from the buffer, or -1 if the buffer is empty.
 */
int SoftwareSerial::read() {
    return rxBuffer_.read_char();
}

/**
 * @brief Retrieves the number of bytes available in the receive buffer.
 *
 * This function returns the number of bytes available in the receive buffer.
 *
 * @return The number of bytes available in the receive buffer.
 */
int SoftwareSerial::available() {
    return rxBuffer_.available();
}

/**
 * @brief Writes a single byte to the transmit buffer.
 *
 * This function waits for any previous transmission to complete, adds start and
 * stop bits to the byte, and then starts the transmission process. If the
 * instance is in half-duplex mode, it also sets the pin as an output.
 *
 * @param[in] b The byte to transmit.
 *
 * @return The number of bytes written (always 1).
 */
size_t SoftwareSerial::write(uint8_t data) {
    // Signal that we're preparing to transmit
    outputPending_ = true;

    // Wait for any previous transmission to complete
    while (txActive) {
    }

    // Format data with start and stop bits
    // Start bit (0) is added by shifting left, stop bit (1) is OR'd as bit 9
    txBuffer = data << 1 | 0x200;

    // Apply inverse logic if needed
    if (inverseLogic_) {
        txBuffer = ~txBuffer;
    }

    // Initialize transmission parameters
    txBitCount = 0;
    txTickCount = OVERSAMPLE;

    // Ensure timer is running at the correct speed
    setSpeed(speed_);

    // In half-duplex mode, switch to TX
    if (halfDuplex_) {
        setRXTX(false);
    }

    // Transmission is ready to start
    outputPending_ = false;
    txActive = this;

    return 1;   // Always returns 1 byte written
}

/**
 * @brief Discards all data in the receive buffer.
 *
 * This function is used to clear the receive buffer and discard any data that
 * has been received but not yet processed. It is usually called to clear the
 * buffer after some data has been received and processed.
 */
void SoftwareSerial::flush() {
    noInterrupts();
    rxBuffer_.clear();
    interrupts();
}

/**
 * @brief Peeks at the next byte in the receive buffer without removing it.
 *
 * This function accesses the next byte in the receive buffer without removing
 * it, allowing you to see what byte will be read next. If no data is available,
 * the function returns -1.
 * 
 * @return The next byte in the buffer, or -1 if the buffer is empty.
 */
int SoftwareSerial::peek() {
    return rxBuffer_.peek();
}

/**
 * @brief Sets the interrupt priority for the timer interrupt used by the SoftwareSerial
 *
 * The interrupt priority is used to determine the order in which interrupts are serviced.
 * A lower priority means that the interrupt will be serviced later than interrupts with
 * higher priorities. A higher priority means that the interrupt will be serviced sooner.
 *
 * @param preemptPriority The preemption priority of the interrupt.
 * @param subPriority The sub-priority of the interrupt.
 */
void SoftwareSerial::setInterruptPriority(uint32_t preemptPriority, uint32_t subPriority) {
    auto& timer = GeneralTimer::get_instance(TIMER_SERIAL_BASE);
    timer.setInterruptPriority(preemptPriority, subPriority);
}
