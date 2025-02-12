//
// MFL gd32f30x GPIO peripheral register access in C++
//
// Copyright (C) 2025 B. Mouritsen <bnmguy@gmail.com>. All rights reserved.
//
// This file is part of the Microcontroller Firmware Library (MFL).
//
// MFL is free software: you can redistribute it and/or modify it under the terms of the
// GNU Lesser General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// MFL is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License along with MFL.
// If not, see <https://www.gnu.org/licenses/>.
//

#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <array>

#include "CONFIG.hpp"
#include "rcu_config.hpp"


namespace gpio {


///////////////////////////// BASE ADDRESS /////////////////////////////

enum class GPIO_Base : uint32_t {
    GPIOA_BASE,
    GPIOB_BASE,
    GPIOC_BASE,
    GPIOD_BASE,
    INVALID
};

static constexpr uintptr_t GPIO_baseAddress[] = {
    0x40010800U, // GPIOA
    0x40010C00U, // GPIOB
    0x40011000U, // GPIOC
    0x40011400U  // GPIOD
};


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class GPIO_Regs : uint32_t {
    CTL0 = 0x00U,
    CTL1 = 0x04U,
    ISTAT = 0x08U,
    OCTL = 0x0CU,
    BOP = 0x10U,
    BC = 0x14U,
    LOCK = 0x18U,
    SPD = 0x3CU
};

enum class AFIO_Regs : uint32_t {
    EC = 0x00U,
    PCF0 = 0x04U,
    EXTISS0 = 0x08U,
    EXTISS1 = 0x0CU,
    EXTISS2 = 0x10U,
    EXTISS3 = 0x14U,
    PCF1 = 0x1CU,
    CPSCTL = 0x20U
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class EC_Bits : uint32_t {
    PIN = REG_BIT_DEF(0, 3),
    PORT = REG_BIT_DEF(4, 6),
    EOE = 7
};

enum class CPSCTL_Bits : uint8_t {
    CPS_EN = 0,
    CPS_RDY = 8
};


///////////////////////////// ENUMS /////////////////////////////

enum class Pin_Mode : uint8_t {
    ANALOG,
    INPUT_FLOATING,
    INPUT_PULLUP,
    INPUT_PULLDOWN,
    OUTPUT_PUSHPULL,
    OUTPUT_OPENDRAIN,
    ALT_PUSHPULL,
    ALT_OPENDRAIN,
    INVALID
};

enum class Output_Speed : uint8_t {
    INVALID = 0U,
    SPEED_10MHZ = 1U,
    SPEED_2MHZ = 2U,
    SPEED_50MHZ = 3U,
    SPEED_MAX = 4U
};

enum class Event_Port : uint8_t {
    EVENT_AT_GPIOA,
    EVENT_AT_GPIOB,
    EVENT_AT_GPIOC,
    EVENT_AT_GPIOD,
    INVALID
};

enum class Source_Port : uint8_t {
    SOURCE_IS_GPIOA,
    SOURCE_IS_GPIOB,
    SOURCE_IS_GPIOC,
    SOURCE_IS_GPIOD,
    INVALID
};

enum class Pin_Number : uint8_t {
    PIN_0,
    PIN_1,
    PIN_2,
    PIN_3,
    PIN_4,
    PIN_5,
    PIN_6,
    PIN_7,
    PIN_8,
    PIN_9,
    PIN_10,
    PIN_11,
    PIN_12,
    PIN_13,
    PIN_14,
    PIN_15,
    INVALID
};

enum class Pin_Remap_Select : uint32_t {
    SPI0_NO_REMAP,
    SPI0_REMAP,
    I2C0_NO_REMAP,
    I2C0_REMAP,
    USART0_NO_REMAP,
    USART0_REMAP,
    USART1_NO_REMAP,
    USART1_REMAP,
    USART2_NO_REMAP,
    USART2_PARTIAL_REMAP,
    USART2_FULL_REMAP,
    TIMER0_NO_REMAP,
    TIMER0_PARTIAL_REMAP,
    TIMER0_FULL_REMAP,
    TIMER1_NO_REMAP,
    TIMER1_PARTIAL_REMAP,
    TIMER1_FULL_REMAP,
    TIMER2_NO_REMAP,
    TIMER2_PARTIAL_REMAP,
    TIMER2_FULL_REMAP,
    TIMER3_NO_REMAP,
    TIMER3_REMAP,
    CAN_NO_REMAP,
    CAN_PARTIAL_REMAP,
    CAN_FULL_REMAP,
    PD01_NO_REMAP,
    PD01_REMAP,
    TIMER4CH4_NO_IREMAP,
    TIMER4CH4_IREMAP,
    ADC0_ETRGINS_NO_REMAP,
    ADC0_ETRGINS_REMAP,
    ADC0_ETRGREG_NO_REMAP,
    ADC0_ETRGREG_REMAP,
    ADC1_ETRGINS_NO_REMAP,
    ADC1_ETRGINS_REMAP,
    ADC1_ETRGREG_NO_REMAP,
    ADC1_ETRGREG_REMAP,
    FULL_SWJ_REMAP,
    FULL_SWJ_NO_NJRST_REMAP,
    SWJ_DP_ONLY_REMAP,
    SWJ_ALL_DISABLED_REMAP,
    SPI2_NO_REMAP,
    SPI2_REMAP,
    TIMER8_NO_REMAP,
    TIMER8_REMAP,
    TIMER9_NO_REMAP,
    TIMER9_REMAP,
    TIMER10_NO_REMAP,
    TIMER10_REMAP,
    TIMER12_NO_REMAP,
    TIMER12_REMAP,
    TIMER13_NO_REMAP,
    TIMER13_REMAP,
    EXMC_NADV_NO_REMAP,
    EXMC_NADV_REMAP,
    CTC_NO_REMAP0,
    CTC_REMAP0,
    CTC_NO_REMAP1,
    CTC_REMAP1,
    NO_REMAP = 60U
};

enum class Remap_Type : uint8_t {
    NONE = 0U,
    REMAP = 1U,
    PARTIAL = 2U,
    FULL = 3U,
    // SWJ only
    FULL_SWJ = 0U,
    FULL_SWJ_NO_NJRST = 1U,
    SWJ_DP_ONLY = 2U,
    ALL_DISABLED = 4U
};

enum class GPIO_Error_Type : uint8_t {
    OK,
    INVALID_PIN,
    INVALID_PORT,
    INVALID_MODE,
    INITIALIZATION_FAILED,
    INVALID_SELECTION
};


///////////////////////////// STRUCTURES /////////////////////////////

struct GPIO_Clock_Config {
    rcu::RCU_PCLK clock_reg;
    rcu::RCU_PCLK_Reset reset_reg;
};

static inline constexpr std::array<GPIO_Clock_Config, 4> GPIO_pclk_index {{
        {rcu::RCU_PCLK::PCLK_GPIOA, rcu::RCU_PCLK_Reset::PCLK_GPIOARST},
        {rcu::RCU_PCLK::PCLK_GPIOB, rcu::RCU_PCLK_Reset::PCLK_GPIOBRST},
        {rcu::RCU_PCLK::PCLK_GPIOC, rcu::RCU_PCLK_Reset::PCLK_GPIOCRST},
        {rcu::RCU_PCLK::PCLK_GPIOD, rcu::RCU_PCLK_Reset::PCLK_GPIODRST}
    }};

struct Remap_Info {
    AFIO_Regs register_offset;
    uint32_t bit_info;
    Remap_Type type;
};

static inline constexpr std::array<Remap_Info, 59> remap_index {{
        {AFIO_Regs::PCF0, REG_BIT_DEF(0, 0), Remap_Type::NONE},       			// SPI0 no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(0, 0), Remap_Type::REMAP},       			// SPI0 remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(1, 1), Remap_Type::NONE},       			// I2C0 no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(1, 1), Remap_Type::REMAP},       			// I2C0 remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(2, 2), Remap_Type::NONE},       			// USART0 no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(2, 2), Remap_Type::REMAP},      			// USART0 remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(3, 3), Remap_Type::NONE},       			// USART1 no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(3, 3), Remap_Type::REMAP},       			// USART1 remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(4, 5), Remap_Type::NONE},       			// USART2 no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(4, 5), Remap_Type::PARTIAL},    			// USART2 partial remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(4, 5), Remap_Type::FULL},       			// USART2 full remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(6, 7), Remap_Type::NONE},       			// TIMER0 no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(6, 7), Remap_Type::PARTIAL},    			// TIMER0 partial remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(6, 7), Remap_Type::FULL},       			// TIMER0 full remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(8, 9), Remap_Type::NONE},       			// TIMER1 no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(8, 9), Remap_Type::PARTIAL},    			// TIMER1 partial remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(8, 9), Remap_Type::FULL},       			// TIMER1 full remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(10, 11), Remap_Type::NONE},     			// TIMER2 no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(10, 11), Remap_Type::PARTIAL},  			// TIMER2 partial remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(10, 11), Remap_Type::FULL},     			// TIMER2 full remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(12, 12), Remap_Type::NONE},     			// TIMER3 no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(12, 12), Remap_Type::REMAP},     			// TIMER3 remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(13, 14), Remap_Type::NONE},     			// CAN no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(13, 14), Remap_Type::PARTIAL},  			// CAN partial remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(13, 14), Remap_Type::FULL},     			// CAN full remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(15, 15), Remap_Type::NONE},  				// PD01 no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(15, 15), Remap_Type::REMAP},     			// PD01 remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(16, 16), Remap_Type::NONE},  				// TIMER4CH3 no iremap
        {AFIO_Regs::PCF0, REG_BIT_DEF(16, 16), Remap_Type::REMAP},				// TIMER4CH3 iremap
        {AFIO_Regs::PCF0, REG_BIT_DEF(17, 17), Remap_Type::NONE},  				// ADC0 ETRGINS no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(17, 17), Remap_Type::REMAP},				// ADC0 ETRGINS remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(18, 18), Remap_Type::NONE},  				// ADC0 ETRGREG no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(18, 18), Remap_Type::REMAP},				// ADC0 ETRGREG remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(19, 19), Remap_Type::NONE},  				// ADC1 ETRGINS no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(19, 19), Remap_Type::REMAP},				// ADC1 ETRGINS remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(20, 20), Remap_Type::NONE},  				// ADC1 ETRGREG no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(20, 20), Remap_Type::REMAP},				// ADC1 ETRGREG remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(24, 26), Remap_Type::FULL_SWJ},  			// FULL SWJ (JTAG-DP + SW-DP)
        {AFIO_Regs::PCF0, REG_BIT_DEF(24, 26), Remap_Type::FULL_SWJ_NO_NJRST},	// FULL SWJ without NJTRST (JTAG-DP + SW-DP)
        {AFIO_Regs::PCF0, REG_BIT_DEF(24, 26), Remap_Type::SWJ_DP_ONLY},		// SW-DP Only (JTAG-DP disabled)
        {AFIO_Regs::PCF0, REG_BIT_DEF(24, 26), Remap_Type::ALL_DISABLED},		// All disabled (SW_DP and JTAG-DP both disabled)
        {AFIO_Regs::PCF0, REG_BIT_DEF(28, 28), Remap_Type::NONE},				// SPI2 no remap
        {AFIO_Regs::PCF0, REG_BIT_DEF(28, 28), Remap_Type::REMAP},				// SPI2 remap
        {AFIO_Regs::PCF1, REG_BIT_DEF(5, 5), Remap_Type::NONE},					// TIMER8 no remap
        {AFIO_Regs::PCF1, REG_BIT_DEF(5, 5), Remap_Type::REMAP},				// TIMER8 remap
        {AFIO_Regs::PCF1, REG_BIT_DEF(6, 6), Remap_Type::NONE},					// TIMER9 no remap
        {AFIO_Regs::PCF1, REG_BIT_DEF(6, 6), Remap_Type::REMAP},				// TIMER9 remap
        {AFIO_Regs::PCF1, REG_BIT_DEF(7, 7), Remap_Type::NONE},					// TIMER10 no remap
        {AFIO_Regs::PCF1, REG_BIT_DEF(7, 7), Remap_Type::REMAP},				// TIMER10 remap
        {AFIO_Regs::PCF1, REG_BIT_DEF(8, 8), Remap_Type::NONE},					// TIMER12 no remap
        {AFIO_Regs::PCF1, REG_BIT_DEF(8, 8), Remap_Type::REMAP},				// TIMER12 remap
        {AFIO_Regs::PCF1, REG_BIT_DEF(9, 9), Remap_Type::NONE},					// TIMER13 no remap
        {AFIO_Regs::PCF1, REG_BIT_DEF(9, 9), Remap_Type::REMAP},				// TIMER13 remap
        {AFIO_Regs::PCF1, REG_BIT_DEF(10, 10), Remap_Type::NONE},				// EXMC NADV no remap
        {AFIO_Regs::PCF1, REG_BIT_DEF(10, 10), Remap_Type::REMAP},				// EXMC_NADV remap
        {AFIO_Regs::PCF1, REG_BIT_DEF(11, 12), Remap_Type::NONE},				// CTC no remap0
        {AFIO_Regs::PCF1, REG_BIT_DEF(11, 12), Remap_Type::REMAP},				// CTC remap0
        {AFIO_Regs::PCF1, REG_BIT_DEF(11, 12), Remap_Type::NONE},				// CTC no remap1
        {AFIO_Regs::PCF1, REG_BIT_DEF(11, 12), Remap_Type::FULL}				// CTC remap1
    }};

struct Exti_Source_Info {
    AFIO_Regs reg;
    uint32_t bit_info;
};

static inline constexpr std::array<Exti_Source_Info, 16> source_index {{
        {AFIO_Regs::EXTISS0, REG_BIT_DEF(0, 3)},
        {AFIO_Regs::EXTISS0, REG_BIT_DEF(4, 7)},
        {AFIO_Regs::EXTISS0, REG_BIT_DEF(8, 11)},
        {AFIO_Regs::EXTISS0, REG_BIT_DEF(12, 15)},
        {AFIO_Regs::EXTISS1, REG_BIT_DEF(0, 3)},
        {AFIO_Regs::EXTISS1, REG_BIT_DEF(4, 7)},
        {AFIO_Regs::EXTISS1, REG_BIT_DEF(8, 11)},
        {AFIO_Regs::EXTISS1, REG_BIT_DEF(12, 15)},
        {AFIO_Regs::EXTISS2, REG_BIT_DEF(0, 3)},
        {AFIO_Regs::EXTISS2, REG_BIT_DEF(4, 7)},
        {AFIO_Regs::EXTISS2, REG_BIT_DEF(8, 11)},
        {AFIO_Regs::EXTISS2, REG_BIT_DEF(12, 15)},
        {AFIO_Regs::EXTISS3, REG_BIT_DEF(0, 3)},
        {AFIO_Regs::EXTISS3, REG_BIT_DEF(4, 7)},
        {AFIO_Regs::EXTISS3, REG_BIT_DEF(8, 11)},
        {AFIO_Regs::EXTISS3, REG_BIT_DEF(12, 15)},
    }};


///////////////////////////// CONSTANTS /////////////////////////////

static inline constexpr uint32_t LockValue = 0x00010000U;

// For option fast bit manipulation functions
static inline constexpr uint32_t GPIOA_BASE_ADDRESS = 0x40010800U;
static inline constexpr uint32_t GPIOB_BASE_ADDRESS = 0x40010C00U;
static inline constexpr uint32_t GPIOC_BASE_ADDRESS = 0x40011000U;
static inline constexpr uint32_t GPIOD_BASE_ADDRESS = 0x40011400U;

static inline constexpr uint32_t CTL0_OFFSET = 0x00U;
static inline constexpr uint32_t CTL1_OFFSET = 0x04U;
static inline constexpr uint32_t ISTAT_OFFSET = 0x08U;
static inline constexpr uint32_t OCTL_OFFSET = 0x0CU;
static inline constexpr uint32_t BOP_OFFSET = 0x10U;
static inline constexpr uint32_t BC_OFFSET = 0x14U;
static inline constexpr uint32_t SPD_OFFSET = 0x3CU;


///////////////////////////// UNUSED DOCUMENTATION ONLY /////////////////////////////

/*
enum class CTL0_Bits : uint32_t {
    MD0 = REG_BIT_DEF(0, 1),
    CTL0 = REG_BIT_DEF(2, 3),
    MD1 = REG_BIT_DEF(4, 5),
    CTL1 = REG_BIT_DEF(6, 7),
    MD2 = REG_BIT_DEF(8, 9),
    CTL2 = REG_BIT_DEF(10, 11),
    MD3 = REG_BIT_DEF(12, 13),
    CTL3 = REG_BIT_DEF(14, 15),
    MD4 = REG_BIT_DEF(16, 17),
    CTL4 = REG_BIT_DEF(18, 19),
    MD5 = REG_BIT_DEF(20, 21),
    CTL5 = REG_BIT_DEF(22, 23),
    MD6 = REG_BIT_DEF(24, 25),
    CTL6 = REG_BIT_DEF(26, 27),
    MD7 = REG_BIT_DEF(28, 29),
    CTL7 = REG_BIT_DEF(30, 31)
};

enum class CTL1_Bits : uint32_t {
    MD8 = REG_BIT_DEF(0, 1),
    CTL8 = REG_BIT_DEF(2, 3),
    MD9 = REG_BIT_DEF(4, 5),
    CTL9 = REG_BIT_DEF(6, 7),
    MD10 = REG_BIT_DEF(8, 9),
    CTL10 = REG_BIT_DEF(10, 11),
    MD11 = REG_BIT_DEF(12, 13),
    CTL11 = REG_BIT_DEF(14, 15),
    MD12 = REG_BIT_DEF(16, 17),
    CTL12 = REG_BIT_DEF(18, 19),
    MD13 = REG_BIT_DEF(20, 21),
    CTL13 = REG_BIT_DEF(22, 23),
    MD14 = REG_BIT_DEF(24, 25),
    CTL14 = REG_BIT_DEF(26, 27),
    MD15 = REG_BIT_DEF(28, 29),
    CTL15 = REG_BIT_DEF(30, 31)
};

enum class ISTAT_Bits : uint32_t {
    ISTAT0 = REG_BIT_DEF(0, 0),
    ISTAT1 = REG_BIT_DEF(1, 1),
    ISTAT2 = REG_BIT_DEF(2, 2),
    ISTAT3 = REG_BIT_DEF(3, 3),
    ISTAT4 = REG_BIT_DEF(4, 4),
    ISTAT5 = REG_BIT_DEF(5, 5),
    ISTAT6 = REG_BIT_DEF(6, 6),
    ISTAT7 = REG_BIT_DEF(7, 7),
    ISTAT8 = REG_BIT_DEF(8, 8),
    ISTAT9 = REG_BIT_DEF(9, 9),
    ISTAT10 = REG_BIT_DEF(10, 10),
    ISTAT11 = REG_BIT_DEF(11, 11),
    ISTAT12 = REG_BIT_DEF(12, 12),
    ISTAT13 = REG_BIT_DEF(13, 13),
    ISTAT14 = REG_BIT_DEF(14, 14),
    ISTAT15 = REG_BIT_DEF(15, 15)
};

enum class OCTL_Bits : uint32_t {
    OCTL0 = REG_BIT_DEF(0, 0),
    OCTL1 = REG_BIT_DEF(1, 1),
    OCTL2 = REG_BIT_DEF(2, 2),
    OCTL3 = REG_BIT_DEF(3, 3),
    OCTL4 = REG_BIT_DEF(4, 4),
    OCTL5 = REG_BIT_DEF(5, 5),
    OCTL6 = REG_BIT_DEF(6, 6),
    OCTL7 = REG_BIT_DEF(7, 7),
    OCTL8 = REG_BIT_DEF(8, 8),
    OCTL9 = REG_BIT_DEF(9, 9),
    OCTL10 = REG_BIT_DEF(10, 10),
    OCTL11 = REG_BIT_DEF(11, 11),
    OCTL12 = REG_BIT_DEF(12, 12),
    OCTL13 = REG_BIT_DEF(13, 13),
    OCTL14 = REG_BIT_DEF(14, 14),
    OCTL15 = REG_BIT_DEF(15, 15)
};

enum class BOP_Bits : uint32_t {
    BOP0 = REG_BIT_DEF(0, 0),
    BOP1 = REG_BIT_DEF(1, 1),
    BOP2 = REG_BIT_DEF(2, 2),
    BOP3 = REG_BIT_DEF(3, 3),
    BOP4 = REG_BIT_DEF(4, 4),
    BOP5 = REG_BIT_DEF(5, 5),
    BOP6 = REG_BIT_DEF(6, 6),
    BOP7 = REG_BIT_DEF(7, 7),
    BOP8 = REG_BIT_DEF(8, 8),
    BOP9 = REG_BIT_DEF(9, 9),
    BOP10 = REG_BIT_DEF(10, 10),
    BOP11 = REG_BIT_DEF(11, 11),
    BOP12 = REG_BIT_DEF(12, 12),
    BOP13 = REG_BIT_DEF(13, 13),
    BOP14 = REG_BIT_DEF(14, 14),
    BOP15 = REG_BIT_DEF(15, 15),
    BOP_CR0 = REG_BIT_DEF(16, 16),
    BOP_CR1 = REG_BIT_DEF(17, 17),
    BOP_CR2 = REG_BIT_DEF(18, 18),
    BOP_CR3 = REG_BIT_DEF(19, 19),
    BOP_CR4 = REG_BIT_DEF(20, 20),
    BOP_CR5 = REG_BIT_DEF(21, 21),
    BOP_CR6 = REG_BIT_DEF(22, 22),
    BOP_CR7 = REG_BIT_DEF(23, 23),
    BOP_CR8 = REG_BIT_DEF(24, 24),
    BOP_CR9 = REG_BIT_DEF(25, 25),
    BOP_CR10 = REG_BIT_DEF(26, 26),
    BOP_CR11 = REG_BIT_DEF(27, 27),
    BOP_CR12 = REG_BIT_DEF(28, 28),
    BOP_CR13 = REG_BIT_DEF(29, 29),
    BOP_CR14 = REG_BIT_DEF(30, 30),
    BOP_CR15 = REG_BIT_DEF(31, 31)
};

enum class BC_Bits : uint32_t {
    BC_CR0 = REG_BIT_DEF(0, 0),
    BC_CR1 = REG_BIT_DEF(1, 1),
    BC_CR2 = REG_BIT_DEF(2, 2),
    BC_CR3 = REG_BIT_DEF(3, 3),
    BC_CR4 = REG_BIT_DEF(4, 4),
    BC_CR5 = REG_BIT_DEF(5, 5),
    BC_CR6 = REG_BIT_DEF(6, 6),
    BC_CR7 = REG_BIT_DEF(7, 7),
    BC_CR8 = REG_BIT_DEF(8, 8),
    BC_CR9 = REG_BIT_DEF(9, 9),
    BC_CR10 = REG_BIT_DEF(10, 10),
    BC_CR11 = REG_BIT_DEF(11, 11),
    BC_CR12 = REG_BIT_DEF(12, 12),
    BC_CR13 = REG_BIT_DEF(13, 13),
    BC_CR14 = REG_BIT_DEF(14, 14),
    BC_CR15 = REG_BIT_DEF(15, 15)
};

enum class LOCK_Bits : uint32_t {
    LK0 = REG_BIT_DEF(0, 0),
    LK1 = REG_BIT_DEF(1, 1),
    LK2 = REG_BIT_DEF(2, 2),
    LK3 = REG_BIT_DEF(3, 3),
    LK4 = REG_BIT_DEF(4, 4),
    LK5 = REG_BIT_DEF(5, 5),
    LK6 = REG_BIT_DEF(6, 6),
    LK7 = REG_BIT_DEF(7, 7),
    LK8 = REG_BIT_DEF(8, 8),
    LK9 = REG_BIT_DEF(9, 9),
    LK10 = REG_BIT_DEF(10, 10),
    LK11 = REG_BIT_DEF(11, 11),
    LK12 = REG_BIT_DEF(12, 12),
    LK13 = REG_BIT_DEF(13, 13),
    LK14 = REG_BIT_DEF(14, 14),
    LK15 = REG_BIT_DEF(15, 15),
    LKK = REG_BIT_DEF(16, 16)
};

enum class SPD_Bits : uint32_t {
    SPD0 = REG_BIT_DEF(0, 0),
    SPD1 = REG_BIT_DEF(1, 1),
    SPD2 = REG_BIT_DEF(2, 2),
    SPD3 = REG_BIT_DEF(3, 3),
    SPD4 = REG_BIT_DEF(4, 4),
    SPD5 = REG_BIT_DEF(5, 5),
    SPD6 = REG_BIT_DEF(6, 6),
    SPD7 = REG_BIT_DEF(7, 7),
    SPD8 = REG_BIT_DEF(8, 8),
    SPD9 = REG_BIT_DEF(9, 9),
    SPD10 = REG_BIT_DEF(10, 10),
    SPD11 = REG_BIT_DEF(11, 11),
    SPD12 = REG_BIT_DEF(12, 12),
    SPD13 = REG_BIT_DEF(13, 13),
    SPD14 = REG_BIT_DEF(14, 14),
    SPD15 = REG_BIT_DEF(15, 15)
};

enum class PCF0_Bits : uint32_t {
    SPI0_REMAP = REG_BIT_DEF(0, 0),
    I2C0_REMAP = REG_BIT_DEF(1, 1),
    USART0_REMAP = REG_BIT_DEF(2, 2),
    USART1_REMAP = REG_BIT_DEF(3, 3),
    USART2_REMAP = REG_BIT_DEF(4, 5),
    TIMER0_REMAP = REG_BIT_DEF(6, 7),
    TIMER1_REMAP = REG_BIT_DEF(8, 9),
    TIMER2_REMAP = REG_BIT_DEF(10, 11),
    TIMER3_REMAP = REG_BIT_DEF(12, 12),
    CAN_REMAP = REG_BIT_DEF(13, 14),
    PD01_REMAP = REG_BIT_DEF(15, 15),
    TIMER4CH3_IREMAP = REG_BIT_DEF(16, 16),
    ADC0_ETRGINS_REMAP = REG_BIT_DEF(17, 17),
    ADC0_ETRGREG_REMAP = REG_BIT_DEF(18, 18),
    ADC1_ETRGINS_REMAP = REG_BIT_DEF(19, 19),
    ADC1_ETRGREG_REMAP = REG_BIT_DEF(20, 20),
    SWJ_CFG = REG_BIT_DEF(24, 26),
    SPI2_REMAP = REG_BIT_DEF(28, 28)
};

enum class EXTISS0_Bits : uint32_t {
    PIN0_EXTI0 = REG_BIT_DEF(0, 3),
    PIN1_EXTI1 = REG_BIT_DEF(4, 7),
    PIN2_EXTI2 = REG_BIT_DEF(8, 11),
    PIN3_EXTI3 = REG_BIT_DEF(12, 15)
};

enum class EXTISS1_Bits : uint32_t {
    PIN4_EXTI4 = REG_BIT_DEF(0, 3),
    PIN5_EXTI5 = REG_BIT_DEF(4, 7),
    PIN6_EXTI6 = REG_BIT_DEF(8, 11),
    PIN7_EXTI7 = REG_BIT_DEF(12, 15)
};

enum class EXTISS2_Bits : uint32_t {
    PIN8_EXTI8 = REG_BIT_DEF(0, 3),
    PIN9_EXTI9 = REG_BIT_DEF(4, 7),
    PIN10_EXTI10 = REG_BIT_DEF(8, 11),
    PIN11_EXTI11 = REG_BIT_DEF(12, 15)
};

enum class EXTISS3_Bits : uint32_t {
    PIN12_EXTI12 = REG_BIT_DEF(0, 3),
    PIN13_EXTI13 = REG_BIT_DEF(4, 7),
    PIN14_EXTI14 = REG_BIT_DEF(8, 11),
    PIN15_EXTI15 = REG_BIT_DEF(12, 15)
};

enum class PCF1_Bits : uint32_t {
    TIMER8_REMAP = REG_BIT_DEF(5, 5),
    TIMER9_REMAP = REG_BIT_DEF(6, 6),
    TIMER10_REMAP = REG_BIT_DEF(7, 7),
    TIMER12_REMAP = REG_BIT_DEF(8, 8),
    TIMER13_REMAP = REG_BIT_DEF(9, 9),
    EXMC_NADV = REG_BIT_DEF(10, 10),
    CTC_REMAP = REG_BIT_DEF(11, 12)
};
*/


} // namespace gpio
