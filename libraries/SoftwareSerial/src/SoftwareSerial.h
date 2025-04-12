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
    virtual int read() override;
    virtual size_t write(uint8_t data) override;
    virtual int available() override;
    virtual void flush() override;
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
    void setTX();
    void setRX();
    void setRXTX(bool input);
    void send();
    void receive();

    // Static interrupt handler
    static void handleInterrupt();

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
