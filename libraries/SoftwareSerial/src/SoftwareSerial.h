/*
 * SoftwareSerial.h (formerly NewSoftSerial.h)
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

#pragma once

#include <Arduino.h>
#include <api/RingBuffer.h>

#define OVERSAMPLE  3               // In RX, Timer will generate interruption OVERSAMPLE time during a bit. Thus OVERSAMPLE ticks in a bit. (interrupt not synchonized with edge).
#define HALFDUPLEX_SWITCH_DELAY 5   // In bit-periods

#ifndef _SS_BUFFER_SIZE
    #define _SS_BUFFER_SIZE 64
#endif

// It's best to define TIMER_SERIAL in variant.h. If not defined, we could choose one here.
// Choose the simplest available because we only need an update interrupt
#ifndef TIMER_SERIAL
    #ifdef SS_TIMER
        #define TIMER_SERIAL SS_TIMER
    #else
        #define TIMER_SERIAL 3  // Default to timer 3
    #endif
#endif

class SoftwareSerial : public Stream {
public:
    // Constructor
    SoftwareSerial(pin_size_t rxPin, pin_size_t txPin, bool inverseLogic = false);
    virtual ~SoftwareSerial();

    // Stream implementation
    int read();
    virtual size_t write(uint8_t data) override;
    int available() override;
    void flush() override;
    int peek() override;
    operator bool() { return true; }
    using Print::write;

    // SoftwareSerial methods
    void begin(long speed);
    void end();
    bool listen();
    bool isListening() { return activeListener == this; }
    bool stopListening();
    bool overflow() {
        bool value = bufferOverflow_;
        if (value) {
            bufferOverflow_ = false;
        }
        return value;
    }
    void setInterruptPriority(uint32_t preemptPriority, uint32_t subPriority);

private:
    // Pin configuration
    pin_size_t rxPin_;
    pin_size_t txPin_;
    gpio::GPIO_Base rxPort_;
    gpio::Pin_Number rxPinNumber_;
    gpio::GPIO_Base txPort_;
    gpio::Pin_Number txPinNumber_;
    bool inverseLogic_;
    bool halfDuplex_;
    bool outputPending_;

    // Serial parameters
    uint32_t speed_;
    bool bufferOverflow_;

    // Receive buffer
    arduino::RingBufferN<_SS_BUFFER_SIZE> rxBuffer_;

    // Private methods
    void setSpeed(uint32_t speed);

    inline void setTX() {
        gpio::fast_write_pin(txPort_, txPinNumber_, inverseLogic_ ? false : true);
        setPinOpsFast(txPort_, txPinNumber_, gpio::Pin_Mode::OUTPUT_PUSHPULL);
    }

    inline void setRX() {
        setPinOpsFast(rxPort_, rxPinNumber_, inverseLogic_ ? gpio::Pin_Mode::INPUT_PULLDOWN : gpio::Pin_Mode::INPUT_PULLUP);    // Pullup for normal logic
    }

    inline void setRXTX(bool input) {
        if (!halfDuplex_) return;

        if (input) {
            // Switch to RX mode
            if (rxActive != this) {
                setRX();
                rxBitCount = -1;    // Waiting for start bit
                rxTickCount = 2;    // Next interrupt will be discarded. 2 interrupts required to consider RX pin level
                rxActive = this;
            }
        } else {
            // Switch to TX mode
            if (rxActive == this) {
                setTX();
                rxActive = nullptr;
            }
        }
    }

    inline void send() {
        // If txTickCount > 0, interrupt is discarded
        if (--txTickCount > 0) return;

        // Only when txTickCount reaches 0 we set TX pin
        if (txBitCount < 10) {  // Transmission not finished (10 = 1 start + 8 bits + 1 stop)
            // Send data (including start and stop bits)
            if (txBuffer & 1) {
                gpio::fast_write_pin(txPort_, txPinNumber_, true);
            } else {
                gpio::fast_write_pin(txPort_, txPinNumber_, false);
            }

            txBuffer >>= 1;
            txTickCount = OVERSAMPLE;  // Wait OVERSAMPLE ticks to send next bit
            txBitCount++;
        } else {  // Transmission finished
            txTickCount = 1;

            if (outputPending_) {
                txActive = nullptr;
            } else if (txBitCount > 10 + OVERSAMPLE * HALFDUPLEX_SWITCH_DELAY) {
                // When in half-duplex mode, wait for HALFDUPLEX_SWITCH_DELAY bit-periods 
                // after the byte has been transmitted before allowing the switch to RX mode
                if (halfDuplex_ && activeListener == this) {
                    setRXTX(true);
                }
                txActive = nullptr;
            }
        }
    }

    inline void receive() {
        rxTickCount = rxTickCount - 1;
        // If rxTickCount > 0, interrupt is discarded
        if (rxTickCount > 0) return;

        // Only when rxTickCount reaches 0, RX pin is considered
        bool inbit = gpio::fast_read_pin(rxPort_, rxPinNumber_);
        if (inverseLogic_) {
            inbit = !inbit;
        }

        if (rxBitCount == -1) {
            // Waiting for start bit
            if (!inbit) {
                // Got start bit
                rxBitCount = 0;
                // Wait 1 bit (OVERSAMPLE ticks) + 1 tick to sample RX pin in the middle of the edge
                rxTickCount = OVERSAMPLE + 1;
                rxBuffer = 0;
            } else {
                // Waiting for start bit, but didn't get right level
                // Wait for next interrupt to check RX pin level
                rxTickCount = 1;
            }
        } else if (rxBitCount >= 8) {
            // Waiting for stop bit
            if (inbit) {
                // Stop bit read complete, add to buffer
                if (!rxBuffer_.isFull()) {
                    // Save new data in buffer
                    rxBuffer_.store_char(rxBuffer);
                } else {
                    bufferOverflow_ = true;
                }
            }

            // Full frame received. Restart waiting for start bit at next interrupt
            rxTickCount = 1;
            rxBitCount = -1;
        } else {
            // Data bits (0-7)
            rxBuffer >>= 1;
            if (inbit) {
                rxBuffer |= 0x80;
            }

            // Prepare for next bit
            rxBitCount++;
            // Wait OVERSAMPLE ticks before sampling next bit
            rxTickCount = OVERSAMPLE;
        }
    }

    // Interrupt handler
    static inline void handleInterrupt() {
        // Process receive first to minimize latency for incoming data
        SoftwareSerial* rx = rxActive;
        if (rx) {
            rx->receive();
        }

        SoftwareSerial* tx = txActive;
        if (tx) {
            tx->send();
        }
    }

    // Static members for interrupt-based communication
    static constexpr timer::TIMER_Base TIMER_SERIAL_BASE = static_cast<timer::TIMER_Base>(TIMER_SERIAL);
    static SoftwareSerial *activeListener;
    static SoftwareSerial *volatile txActive;
    static SoftwareSerial *volatile rxActive;
    static int32_t txTickCount;
    static int32_t volatile rxTickCount;
    static uint32_t txBuffer;
    static int32_t txBitCount;
    static uint32_t rxBuffer;
    static int32_t rxBitCount;
    static uint32_t currentSpeed;
};
