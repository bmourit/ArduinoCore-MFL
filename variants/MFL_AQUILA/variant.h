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

// GPIOA
#define PA0     0
#define PA1     1
#define PA2     2
#define PA3     3
#define PA4     4
#define PA5     5
#define PA6     6
#define PA7     7
#define PA8     8
#define PA9     9
#define PA10    10
#define PA11    11
#define PA12    12
#define PA13    13
#define PA14    14
#define PA15    15

// GPIOB
#define PB0     16
#define PB1     17
#define PB2     18
#define PB3     19
#define PB4     20
#define PB5     21
#define PB6     22
#define PB7     23
#define PB8     24
#define PB9     25
#define PB10    26
#define PB11    27
#define PB12    28
#define PB13    29
#define PB14    30
#define PB15    31

// GPIOC
#define PC0     32
#define PC1     33
#define PC2     34
#define PC3     35
#define PC4     36
#define PC5     37
#define PC6     38
#define PC7     39
#define PC8     40
#define PC9     41
#define PC10    42
#define PC11    43
#define PC12    44
#define PC13    45
#define PC14    46
#define PC15    47

// GPIOD
#define PD0     48
#define PD1     49
#define PD2     50

// Pin numbers
#define DIGITAL_PIN_COUNT   51
#define ANALOG_PIN_COUNT    16
#define TOTAL_PIN_COUNT     DIGITAL_PIN_COUNT

#define ADC_TEMP            (TOTAL_PIN_COUNT + 1)
#define ADC_VREF            (ADC_TEMP + 1)
#define MAX_PIN_NUM         (ADC_VREF)
#define INVALID_PIN_NUMBER  (ADC_VREF + 1)

// On-board LED pin number
#ifndef LED_BUILTIN
    #define LED_BUILTIN PB11
#endif

#define LED_GREEN   LED_BUILTIN

// On board user button
#ifndef USER_BTN
    #define USER_BTN    PC13
#endif

#ifndef ADC_CHANNEL_TEMPSENSOR
    #define ADC_CHANNEL_TEMPSENSOR  16
#endif
#ifndef ADC_CHANNEL_VREFINT
    #define ADC_CHANNEL_VREFINT     17
#endif

//
// SPI
//
// Each define will create a global instance
// Leave undefined to call your own
//
#define BOARD_USE_SPI0
//#define BOARD_USE_SPI1
//#define BOARD_USE_SPI2

// SPI0
#define PIN_SPI_SS      PA4
#define PIN_SPI_MOSI    PA7
#define PIN_SPI_MISO    PA6
#define PIN_SPI_SCK     PA5

// SPI1
#define PIN_SPI1_SS     PB12
#define PIN_SPI1_MOSI   PB15
#define PIN_SPI1_MISO   PB14
#define PIN_SPI1_SCK    PB13

// SPI2
#define PIN_SPI2_SS     PA15
#define PIN_SPI2_MOSI   PB5
#define PIN_SPI2_MISO   PB4
#define PIN_SPI2_SCK    PB3

//
// I2C
//
// Each define will create a global instance
// Leave undefined to call your own
//
#define BOARD_USE_I2C0
//#define BOARD_USE_I2C1

// I2C0
#define PIN_WIRE_SDA    PB7
#define PIN_WIRE_SCL    PB6

// I2C1
#define PIN_WIRE1_SDA   PB11
#define PIN_WIRE1_SCL   PB10

// Timer
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
