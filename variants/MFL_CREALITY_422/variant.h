/*
    Copyright (c) 2025 Arduino LLC.  All rights reserved.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    See the GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

#include <stdint.h>

// Pin numbers
#define DIGITAL_PIN_COUNT 51
#define ANALOG_PIN_COUNT  16
#define TOTAL_PIN_COUNT   DIGITAL_PIN_COUNT

#define ADC_TEMP    (TOTAL_PIN_COUNT + 1)
#define ADC_VREF    (ADC_TEMP + 1)
#define MAX_PIN_NUM (ADC_VREF)
#define INVALID_PIN (ADC_VREF + 1)

// On-board LED pin number
#ifndef LED_BUILTIN
    #define LED_BUILTIN 27
#endif

#define LED_GREEN     LED_BUILTIN

// On board user button
#ifndef USER_BTN
    #define USER_BTN    45
#endif

#ifndef SERIAL_USE_DMA_RX
    #define SERIAL_USE_DMA_RX
#endif

#ifndef ADC_CHANNEL_TEMPSENSOR
    #define ADC_CHANNEL_TEMPSENSOR  16
#endif
#ifndef ADC_CHANNEL_VREFINT
    #define ADC_CHANNEL_VREFINT     17
#endif

// SPI Definitions
#define SUPPORT_SPI0
#define PIN_SPI_SS      4
#define PIN_SPI_MOSI    7
#define PIN_SPI_MISO    6
#define PIN_SPI_SCK     5

inline constexpr uint8_t MOSI = PIN_SPI_MOSI;
inline constexpr uint8_t MISO = PIN_SPI_MISO;
inline constexpr uint8_t SCK = PIN_SPI_SCK;
inline constexpr uint8_t SS = PIN_SPI_SS;

// I2C Definitions
#define SUPPORT_I2C0
#define PIN_WIRE_SDA    23
#define PIN_WIRE_SCL    22

inline constexpr uint8_t SDA = PIN_WIRE_SDA;
inline constexpr uint8_t SCL = PIN_WIRE_SCL;

#ifndef TIMER_TONE
    #define TIMER_TONE  2  // TIMER_TONE must be defined in this file
#endif
#ifndef TIMER_SERVO
    #define TIMER_SERVO 1  // TIMER_SERVO must be defined in this file
#endif

// Default U(S)ARTx Serial number (0-4)
#define DEFAULT_HARDWARE_SERIAL 0

// Serial
#define PIN_SERIAL_RX   10
#define PIN_SERIAL_TX   9

#ifndef PWM_FREQUENCY
    #define PWM_FREQUENCY       1000
#endif
#ifndef PWM_MAX_DUTY_CYCLE
    #define PWM_MAX_DUTY_CYCLE  4095
#endif

// C++ only
#ifdef __cplusplus
    // These serial port names are intended to allow libraries and architecture-neutral
    // sketches to automatically default to the correct port name for a particular type
    // of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
    // the first hardware serial port whose RX/TX pins are not dedicated to another use.
    //
    // SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
    //
    // SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
    //
    // SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
    //
    // SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
    //
    // SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
    //                            pins are NOT connected to anything by default.
    #ifndef SERIAL_PORT_MONITOR
        #define SERIAL_PORT_MONITOR   Serial
    #endif
    #ifndef SERIAL_PORT_HARDWARE
        #define SERIAL_PORT_HARDWARE  Serial0
    #endif

#endif  // __cplusplus
