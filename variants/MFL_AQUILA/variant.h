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

#include "GPIO.hpp"

// GPIOA
inline constexpr uint8_t PA0 = 0;
inline constexpr uint8_t PA1 = 1;
inline constexpr uint8_t PA2 = 2;
inline constexpr uint8_t PA3 = 3;
inline constexpr uint8_t PA4 = 4;
inline constexpr uint8_t PA5 = 5;
inline constexpr uint8_t PA6 = 6;
inline constexpr uint8_t PA7 = 7;
inline constexpr uint8_t PA8 = 8;
inline constexpr uint8_t PA9 = 9;
inline constexpr uint8_t PA10 = 10;
inline constexpr uint8_t PA11 = 11;
inline constexpr uint8_t PA12 = 12;
inline constexpr uint8_t PA13 = 13;
inline constexpr uint8_t PA14 = 14;
inline constexpr uint8_t PA15 = 15;

// GPIOB
inline constexpr uint8_t PB0 = 16;
inline constexpr uint8_t PB1 = 17;
inline constexpr uint8_t PB2 = 18;
inline constexpr uint8_t PB3 = 19;
inline constexpr uint8_t PB4 = 20;
inline constexpr uint8_t PB5 = 21;
inline constexpr uint8_t PB6 = 22;
inline constexpr uint8_t PB7 = 23;
inline constexpr uint8_t PB8 = 24;
inline constexpr uint8_t PB9 = 25;
inline constexpr uint8_t PB10 = 26;
inline constexpr uint8_t PB11 = 27;
inline constexpr uint8_t PB12 = 28;
inline constexpr uint8_t PB13 = 29;
inline constexpr uint8_t PB14 = 30;
inline constexpr uint8_t PB15 = 31;

// GPIOC
inline constexpr uint8_t PC0 = 32;
inline constexpr uint8_t PC1 = 33;
inline constexpr uint8_t PC2 = 34;
inline constexpr uint8_t PC3 = 35;
inline constexpr uint8_t PC4 = 36;
inline constexpr uint8_t PC5 = 37;
inline constexpr uint8_t PC6 = 38;
inline constexpr uint8_t PC7 = 39;
inline constexpr uint8_t PC8 = 40;
inline constexpr uint8_t PC9 = 41;
inline constexpr uint8_t PC10 = 42;
inline constexpr uint8_t PC11 = 43;
inline constexpr uint8_t PC12 = 44;
inline constexpr uint8_t PC13 = 45;
inline constexpr uint8_t PC14 = 46;
inline constexpr uint8_t PC15 = 47;

// GPIOD
inline constexpr uint8_t PD0 = 48;
inline constexpr uint8_t PD1 = 49;
inline constexpr uint8_t PD2 = 50;
inline constexpr uint8_t PD3 = 51;

// Pin numbers
#define DIGITAL_PIN_COUNT   51
#define ANALOG_PIN_COUNT    16
#define TOTAL_PIN_COUNT     DIGITAL_PIN_COUNT

#define ADC_TEMP            (TOTAL_PIN_COUNT + 1)
#define ADC_VREF            (ADC_TEMP + 1)
#define MAX_PIN_NUM         (ADC_VREF)
#define INVALID_PIN         (ADC_VREF + 1)

// On-board LED pin number
#ifndef LED_BUILTIN
    #define LED_BUILTIN PB11
#endif

#define LED_GREEN   LED_BUILTIN

// On board user button
#ifndef USER_BTN
    #define USER_BTN    PC13
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
#define PIN_SPI_SS      PA4
#define PIN_SPI_MOSI    PA7
#define PIN_SPI_MISO    PA6
#define PIN_SPI_SCK     PA5

inline constexpr uint8_t MOSI = PIN_SPI_MOSI;
inline constexpr uint8_t MISO = PIN_SPI_MISO;
inline constexpr uint8_t SCK = PIN_SPI_SCK;
inline constexpr uint8_t SS = PIN_SPI_SS;

// I2C Definitions
#define SUPPORT_I2C0
#define PIN_WIRE_SDA    PB7
#define PIN_WIRE_SCL    PB6

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
#define PIN_SERIAL_RX   PA10
#define PIN_SERIAL_TX   PA9

#ifndef PWM_FREQUENCY
    #define PWM_FREQUENCY       1000
#endif
#ifndef PWM_MAX_DUTY_CYCLE
    #define PWM_MAX_DUTY_CYCLE  4095
#endif

inline bool isPinNumberValid(pin_size_t pin) { return ((pin >= 0U) && (pin <= DIGITAL_PIN_COUNT)); }
inline pin_size_t digitalPinToInterrupt(pin_size_t pin) { return pin; }
inline pin_size_t analogInputToDigitalPin(pin_size_t pin) { return pin; }
inline gpio::GPIO_Base digitalPinToPort(pin_size_t pin) { return getPortFromPin(pin); }

uint32_t portOutputRegister(gpio::GPIO_Base port) { return reinterpret_cast<uint32_t>(portToInstance(port).reg_address(gpio::GPIO_Regs::OCTL)); }
uint32_t portInputRegister(gpio::GPIO_Base port) { return reinterpret_cast<uint32_t>(portToInstance(port).reg_address(gpio::GPIO_Regs::ISTAT)); }
uint32_t portSetRegister(gpio::GPIO_Base port) { return reinterpret_cast<uint32_t>(portToInstance(port).reg_address(gpio::GPIO_Regs::BOP)); }
uint32_t portClearRegister(gpio::GPIO_Base port) { return reinterpret_cast<uint32_t>(portToInstance(port).reg_address(gpio::GPIO_Regs::BC)); }

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
