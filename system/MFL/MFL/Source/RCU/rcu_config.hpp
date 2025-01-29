//
// MFL gd32f30x RCU peripheral register access in C+
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

namespace rcu {


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class RCU_Regs : uint32_t {
    CTL = 0x00U,
    CFG0 = 0x04U,
    INTR = 0x08U,
    APB2RST = 0x0CU,
    APB1RST = 0x10U,
    AHBEN = 0x14U,
    APB2EN = 0x18U,
    APB1EN = 0x1CU,
    BDCTL = 0x20U,
    RSTSCK = 0x24U,
    CFG1 = 0x2CU,
    DSV = 0x34U,
    ADDCTL = 0xC0U,
    ADDINTR = 0xCCU,
    ADDAPB1RST = 0xE0U,
    ADDAPB1EN = 0xE4U
};


///////////////////////////// STARTUP FUNCTIONALITY /////////////////////////////

static inline const unsigned char AHBPrescaler[] = {
    0x00U,
    0x00U,
    0x00U,
    0x00U,
    0x00U,
    0x00U,
    0x00U,
    0x00U,
    0x01U,
    0x02U,
    0x03U,
    0x04U,
    0x06U,
    0x07U,
    0x08U,
    0x09U
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class CTL_Bits : uint32_t {
    IRC8MEN = 0,
    IRC8MSTB = 1,
    IRC8MADJ = REG_BIT_DEF(3, 7),
    IRC8MCALIB = REG_BIT_DEF(8, 15),
    HXTALEN = 16,
    HXTALSTB = 17,
    HXTALBPS = 18,
    CKMEN = 19,
    PLLEN = 24,
    PLLSTB = 25
};

enum class CFG0_Bits : uint32_t {
    SCS = REG_BIT_DEF(0, 1),
    SCSS = REG_BIT_DEF(2, 3),
    AHBPSC = REG_BIT_DEF(4, 7),
    APB1PSC = REG_BIT_DEF(8, 10),
    APB2PSC = REG_BIT_DEF(11, 13),
    ADCPSC = REG_BIT_DEF(14, 15),
    PLLSEL = 16,
    PREDV0 = 17,
    PLLMF = REG_BIT_DEF(18, 21),
    USBDPSC = REG_BIT_DEF(22, 23),
    CKOUT0SEL = REG_BIT_DEF(24, 26),
    PLLMF_4 = 27,
    ADCPSC_2 = 28,
    PLLMF_5 = 30,
    USBDPSC_2 = 31
};

enum class INTR_Bits : uint32_t {
    IRC40KSTBIF = 0,
    LXTALSTBIF = 1,
    IRC8MSTBIF = 2,
    HXTALSTBIF = 3,
    PLLSTBIF = 4,
    CKMIF = 7,
    IRC40KSTBIE = 8,
    LXTALSTBIE = 9,
    IRC8MSTBIE = 10,
    HXTALSTBIE = 11,
    PLLSTBIE = 12,
    IRC40KSTBIC = 16,
    LXTALSTBIC = 17,
    IRC8MSTBIC = 18,
    HXTALSTBIC = 19,
    PLLSTBIC = 20,
    CKMIC = 23,
    CLEAR_ALL = REG_BIT_DEF(16, 20)
};

enum class APB2RST_Bits : uint8_t {
    AFRST = 0,
    PARST = 2,
    PBRST = 3,
    PCRST = 4,
    PDRST = 5,
    PERST = 6,
    PFRST = 7,
    PGRST = 8,
    ADC0RST = 9,
    ADC1RST = 10,
    TIMER0RST = 11,
    SPI0RST = 12,
    TIMER7RST = 13,
    USART0RST = 14,
    ADC2RST = 15
};

enum class APB1RST_Bits : uint8_t {
    TIMER1RST = 0,
    TIMER2RST = 1,
    TIMER3RST = 2,
    TIMER4RST = 3,
    TIMER5RST = 4,
    TIMER6RST = 5,
    WWDGTRST = 11,
    SPI1RST = 14,
    SPI2RST = 15,
    USART1RST = 17,
    USART2RST = 18,
    UART3RST = 19,
    UART4RST = 20,
    I2C0RST = 21,
    I2C1RST = 22,
    USBDRST = 23,
    CAN0RST = 25,
    BKPIRST = 27,
    PMURST = 28,
    DACRST = 29
};

enum class AHBEN_bit : uint8_t {
    DMA0EN = 0,
    DMA1EN = 1,
    SRAMSPEN = 2,
    FMCSPEN = 4,
    CRCEN = 6,
    EXMCEN = 8,
    SDIOEN = 10
};

enum class APB2EN_Bits : uint8_t {
    AFEN = 0,
    PAEN = 2,
    PBEN = 3,
    PCEN = 4,
    PDEN = 5,
    PEEN = 6,
    PFEN = 7,
    PGEN = 8,
    ADC0EN = 9,
    ADC1EN = 10,
    TIMER0EN = 11,
    SPI0EN = 12,
    TIMER7EN = 13,
    USART0EN = 14,
    ADC2EN = 15
};

enum class APB1EN_Bits : uint8_t {
    TIMER1EN = 0,
    TIMER2EN = 1,
    TIMER3EN = 2,
    TIMER4EN = 3,
    TIMER5EN = 4,
    TIMER6EN = 5,
    WWDGTEN = 11,
    SPI1EN = 14,
    SPI2EN = 15,
    USART1EN = 17,
    USART2EN = 18,
    UART3EN = 19,
    UART4EN = 20,
    I2C0EN = 21,
    I2C1EN = 22,
    USBDEN = 23,
    CAN0EN = 25,
    BKPIEN = 27,
    PMUEN = 28,
    DACEN = 29
};

enum class BDCTL_Bits : uint32_t {
    LXTALEN = 0,
    LXTALSTB = 1,
    LXTALBPS = 2,
    LXTALDRI = REG_BIT_DEF(3, 4),
    RTCSRC = REG_BIT_DEF(8, 9),
    RTCEN = 15,
    BKPRST = 16
};

enum class RSTSCK_Bits : uint8_t {
    IRC40KEN = 0,
    IRC40KSTB = 1,
    RSTFC = 24,
    EPRSTF = 26,
    PORRSTF = 27,
    SWRSTF = 28,
    FWDGTRSTF = 29,
    WWDGTRSTF = 30,
    LPRSTF = 31
};

enum class CFG1_Bits : uint8_t {
    ADCPSC_3 = 29,
    PLLPRESEL = 30
};

enum class DSV_Bits : uint32_t {
    DSLPVS = REG_BIT_DEF(0, 2)
};

enum class ADDCTL_Bits : uint32_t {
    CK48MSEL = 0,
    IRC48MEN = 16,
    IRC48MSTB = 17,
    IRC48MCAL = REG_BIT_DEF(24, 31)
};

enum class ADDINTR_Bits : uint8_t {
    IRC48MSTBIF = 6,
    IRC48MSTBIE = 14,
    IRC48MSTBIC = 22
};

enum class ADDAPB1RST_Bits : uint8_t {
    CTCRST = 27
};

enum class ADDAPB1EN_Bits : uint8_t {
    CTCEN = 27
};

///////////////////////////// PCLKS /////////////////////////////

enum class RCU_PCLK : uint8_t {
    PCLK_DMA0,
    PCLK_DMA1,
    PCLK_CRC,
    PCLK_EXMC,
    PCLK_SDIO,
    PCLK_TIMER1,
    PCLK_TIMER2,
    PCLK_TIMER3,
    PCLK_TIMER4,
    PCLK_TIMER5,
    PCLK_TIMER6,
    PCLK_WWDGT,
    PCLK_SPI1,
    PCLK_SPI2,
    PCLK_USART1,
    PCLK_USART2,
    PCLK_UART3,
    PCLK_UART4,
    PCLK_I2C0,
    PCLK_I2C1,
    PCLK_USBD,
    PCLK_CAN0,
    PCLK_BKPI,
    PCLK_PMU,
    PCLK_DAC,
    PCLK_RTC,
    PCLK_CTC,
    PCLK_AF,
    PCLK_GPIOA,
    PCLK_GPIOB,
    PCLK_GPIOC,
    PCLK_GPIOD,
    PCLK_GPIOE,
    PCLK_GPIOF,
    PCLK_GPIOG,
    PCLK_ADC0,
    PCLK_ADC1,
    PCLK_TIMER0,
    PCLK_SPI0,
    PCLK_TIMER7,
    PCLK_USART0,
    PCLK_ADC2
};

struct index_to_bits {
    RCU_Regs register_offset;
    uint32_t bit_info;
};

static inline constexpr std::array<index_to_bits, 42> pclk_index {{
    {RCU_Regs::AHBEN, REG_BIT_DEF(0, 0)},
    {RCU_Regs::AHBEN, REG_BIT_DEF(1, 1)},
    {RCU_Regs::AHBEN, REG_BIT_DEF(6, 6)},
    {RCU_Regs::AHBEN, REG_BIT_DEF(8, 8)},
    {RCU_Regs::AHBEN, REG_BIT_DEF(10, 10)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(0, 0)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(1, 1)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(2, 2)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(3, 3)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(4, 4)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(5, 5)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(11, 11)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(14, 14)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(15, 15)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(17, 17)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(18, 18)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(19, 19)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(20, 20)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(21, 21)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(22, 22)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(23, 23)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(25, 25)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(27, 27)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(28, 28)},
    {RCU_Regs::APB1EN, REG_BIT_DEF(29, 29)},
    {RCU_Regs::BDCTL, REG_BIT_DEF(15, 15)},
    {RCU_Regs::ADDAPB1EN, REG_BIT_DEF(27, 27)},
    {RCU_Regs::APB2EN, REG_BIT_DEF(0, 0)},
    {RCU_Regs::APB2EN, REG_BIT_DEF(2, 2)},
    {RCU_Regs::APB2EN, REG_BIT_DEF(3, 3)},
    {RCU_Regs::APB2EN, REG_BIT_DEF(4, 4)},
    {RCU_Regs::APB2EN, REG_BIT_DEF(5, 5)},
    {RCU_Regs::APB2EN, REG_BIT_DEF(6, 6)},
    {RCU_Regs::APB2EN, REG_BIT_DEF(7, 7)},
    {RCU_Regs::APB2EN, REG_BIT_DEF(8, 8)},
    {RCU_Regs::APB2EN, REG_BIT_DEF(9, 9)},
    {RCU_Regs::APB2EN, REG_BIT_DEF(10, 10)},
    {RCU_Regs::APB2EN, REG_BIT_DEF(11, 11)},
    {RCU_Regs::APB2EN, REG_BIT_DEF(12, 12)},
    {RCU_Regs::APB2EN, REG_BIT_DEF(13, 13)},
    {RCU_Regs::APB2EN, REG_BIT_DEF(14, 14)},
    {RCU_Regs::APB2EN, REG_BIT_DEF(15, 15)}
}};

///////////////////////////// PCLK RESET /////////////////////////////

enum class RCU_PCLK_Reset : uint8_t {
    PCLK_TIMER1RST,
    PCLK_TIMER2RST,
    PCLK_TIMER3RST,
    PCLK_TIMER4RST,
    PCLK_TIMER5RST,
    PCLK_TIMER6RST,
    PCLK_WWDGTRST,
    PCLK_SPI1RST,
    PCLK_SPI2RST,
    PCLK_USART1RST,
    PCLK_USART2RST,
    PCLK_UART3RST,
    PCLK_UART4RST,
    PCLK_I2C0RST,
    PCLK_I2C1RST,
    PCLK_USBDRST,
    PCLK_CAN0RST,
    PCLK_BKPIRST,
    PCLK_PMURST,
    PCLK_DACRST,
    PCLK_CTCRST,
    PCLK_AFRST,
    PCLK_GPIOARST,
    PCLK_GPIOBRST,
    PCLK_GPIOCRST,
    PCLK_GPIODRST,
    PCLK_GPIOERST,
    PCLK_GPIOFRST,
    PCLK_GPIOGRST,
    PCLK_ADC0RST,
    PCLK_ADC1RST,
    PCLK_TIMER0RST,
    PCLK_SPI0RST,
    PCLK_TIMER7RST,
    PCLK_USART0RST,
    PCLK_ADC2RST
};

static inline constexpr std::array<index_to_bits, 36> pclk_reset_index {{
    {RCU_Regs::APB1RST, REG_BIT_DEF(0, 0)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(1, 1)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(2, 2)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(3, 3)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(4, 4)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(5, 5)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(11, 11)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(14, 14)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(15, 15)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(17, 17)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(18, 18)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(19, 19)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(20, 20)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(21, 21)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(22, 22)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(23, 23)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(25, 25)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(27, 27)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(28, 28)},
    {RCU_Regs::APB1RST, REG_BIT_DEF(29, 29)},
    {RCU_Regs::ADDAPB1RST, REG_BIT_DEF(27, 27)},
    {RCU_Regs::APB2RST, REG_BIT_DEF(0, 0)},
    {RCU_Regs::APB2RST, REG_BIT_DEF(2, 2)},
    {RCU_Regs::APB2RST, REG_BIT_DEF(3, 3)},
    {RCU_Regs::APB2RST, REG_BIT_DEF(4, 4)},
    {RCU_Regs::APB2RST, REG_BIT_DEF(5, 5)},
    {RCU_Regs::APB2RST, REG_BIT_DEF(6, 6)},
    {RCU_Regs::APB2RST, REG_BIT_DEF(7, 7)},
    {RCU_Regs::APB2RST, REG_BIT_DEF(8, 8)},
    {RCU_Regs::APB2RST, REG_BIT_DEF(9, 9)},
    {RCU_Regs::APB2RST, REG_BIT_DEF(10, 10)},
    {RCU_Regs::APB2RST, REG_BIT_DEF(11, 11)},
    {RCU_Regs::APB2RST, REG_BIT_DEF(12, 12)},
    {RCU_Regs::APB2RST, REG_BIT_DEF(13, 13)},
    {RCU_Regs::APB2RST, REG_BIT_DEF(14, 14)},
    {RCU_Regs::APB2RST, REG_BIT_DEF(15, 15)}
}};


///////////////////////////// SLEEP /////////////////////////////

enum class RCU_PCLK_Sleep : uint8_t {
    PCLK_SLEEP_SRAM,
    PCLK_SLEEP_FMC
};

static inline constexpr std::array<index_to_bits, 2> pclk_sleep_index {{
    {RCU_Regs::AHBEN, REG_BIT_DEF(2, 2)},
    {RCU_Regs::AHBEN, REG_BIT_DEF(4, 4)}
}};


///////////////////////////// INTERRUPTS AND FLAGS /////////////////////////////

enum class Status_Flags : uint8_t {
    FLAG_IRC8MSTB,
    FLAG_HXTALSTB,
    FLAG_PLLSTB,
    FLAG_LXTALSTB,
    FLAG_IRC40KSTB,
    FLAG_IRC48MSTB,
    FLAG_EPRST,
    FLAG_PORRST,
    FLAG_SWRST,
    FLAG_FWDGTRST,
    FLAG_WWDGTRST,
    FLAG_LPRST
};

static inline constexpr std::array<index_to_bits, 12> status_flag_index {{
    {RCU_Regs::CTL, REG_BIT_DEF(1, 1)},
    {RCU_Regs::CTL, REG_BIT_DEF(17, 17)},
    {RCU_Regs::CTL, REG_BIT_DEF(25, 25)},
    {RCU_Regs::BDCTL, REG_BIT_DEF(1, 1)},
    {RCU_Regs::RSTSCK, REG_BIT_DEF(1, 1)},
    {RCU_Regs::ADDCTL, REG_BIT_DEF(17, 17)},
    {RCU_Regs::RSTSCK, REG_BIT_DEF(26, 26)},
    {RCU_Regs::RSTSCK, REG_BIT_DEF(27, 27)},
    {RCU_Regs::RSTSCK, REG_BIT_DEF(28, 28)},
    {RCU_Regs::RSTSCK, REG_BIT_DEF(29, 29)},
    {RCU_Regs::RSTSCK, REG_BIT_DEF(30, 30)},
    {RCU_Regs::RSTSCK, REG_BIT_DEF(31, 31)}
}};

enum class Interrupt_Flags : uint8_t {
    INTR_FLAG_IRC40KSTB,
    INTR_FLAG_LXTALSTB,
    INTR_FLAG_IRC8MSTB,
    INTR_FLAG_HXTALSTB,
    INTR_FLAG_PLLSTB,
    INTR_FLAG_CKM,
    INTR_FLAG_IRC48MSTB
};

static inline constexpr std::array<index_to_bits, 7> interrupt_flag_index {{
    {RCU_Regs::INTR, REG_BIT_DEF(0, 0)},
    {RCU_Regs::INTR, REG_BIT_DEF(1, 1)},
    {RCU_Regs::INTR, REG_BIT_DEF(2, 2)},
    {RCU_Regs::INTR, REG_BIT_DEF(3, 3)},
    {RCU_Regs::INTR, REG_BIT_DEF(4, 4)},
    {RCU_Regs::INTR, REG_BIT_DEF(7, 7)},
    {RCU_Regs::ADDINTR, REG_BIT_DEF(6, 6)}
}};

enum class Clear_Flags : uint8_t {
    INTR_FLAG_IRC40KSTB_CLR,
    INTR_FLAG_LXTALSTB_CLR,
    INTR_FLAG_IRC8MSTB_CLR,
    INTR_FLAG_HXTALSTB_CLR,
    INTR_FLAG_PLLSTB_CLR,
    INTR_FLAG_CKM_CLR,
    INTR_FLAG_IRC48MSTB_CLR
};

static inline constexpr std::array<index_to_bits, 7> clear_flag_index {{
    {RCU_Regs::INTR, REG_BIT_DEF(16, 16)},
    {RCU_Regs::INTR, REG_BIT_DEF(17, 17)},
    {RCU_Regs::INTR, REG_BIT_DEF(18, 18)},
    {RCU_Regs::INTR, REG_BIT_DEF(19, 19)},
    {RCU_Regs::INTR, REG_BIT_DEF(20, 20)},
    {RCU_Regs::INTR, REG_BIT_DEF(23, 23)},
    {RCU_Regs::ADDINTR, REG_BIT_DEF(22, 22)}
}};

enum class Interrupt_Type : uint8_t {
    INTR_IRC40KSTB,
    INTR_LXTALSTB,
    INTR_IRC8MSTB,
    INTR_HXTALSTB,
    INTR_PLLSTB,
    INTR_IRC48MSTB
};

static inline constexpr std::array<index_to_bits, 6> interrupt_type_index {{
    {RCU_Regs::INTR, REG_BIT_DEF(8, 8)},
    {RCU_Regs::INTR, REG_BIT_DEF(9, 9)},
    {RCU_Regs::INTR, REG_BIT_DEF(10, 10)},
    {RCU_Regs::INTR, REG_BIT_DEF(11, 11)},
    {RCU_Regs::INTR, REG_BIT_DEF(12, 12)},
    {RCU_Regs::ADDINTR, REG_BIT_DEF(14, 14)}
}};

///////////////////////////// OSCI /////////////////////////////

enum class OSCI_Select : uint8_t {
    HXTAL,
    LXTAL,
    IRC8M,
    IRC48M,
    IRC40K,
    PLL_CK
};

static inline constexpr std::array<index_to_bits, 6> osci_select_index {{
    {RCU_Regs::CTL, REG_BIT_DEF(16, 16)},
    {RCU_Regs::BDCTL, REG_BIT_DEF(0, 0)},
    {RCU_Regs::CTL, REG_BIT_DEF(0, 0)},
    {RCU_Regs::ADDCTL, REG_BIT_DEF(16, 16)},
    {RCU_Regs::RSTSCK, REG_BIT_DEF(0, 0)},
    {RCU_Regs::CTL, REG_BIT_DEF(24, 24)}
}};


///////////////////////////// SYSTEM CLOCK /////////////////////////////

enum class Clock_Frequency : uint8_t {
    CK_SYS,
    CK_AHB,
    CK_APB1,
    CK_APB2
};

// SCS and SCSS
enum class System_Clock_Source : uint8_t {
    SOURCE_IRC8M,
    SOURCE_HXTAL,
    SOURCE_PLL,
    SOURCE_INVALID
};

struct System_Clock_Source_Mapping {
    uint32_t value;
    System_Clock_Source source;
};

static inline constexpr std::array<System_Clock_Source_Mapping, 3> source_mapping {{
    {0, System_Clock_Source::SOURCE_IRC8M},
    {1, System_Clock_Source::SOURCE_HXTAL},
    {2, System_Clock_Source::SOURCE_PLL}
}};

///////////////////////////// AHB?APB1/APB2 BUSES /////////////////////////////

enum class AHB_Prescaler : uint8_t {
    CKSYS_DIV1 = 0,
    CKSYS_DIV2 = 8,
    CKSYS_DIV4 = 9,
    CKSYS_DIV8 = 10,
    CKSYS_DIV16 = 11,
    CKSYS_DIV64 = 12,
    CKSYS_DIV128 = 13,
    CKSYS_DIV256 = 14,
    CKSYS_DIV512 = 15,
    INVALID = 16
};

// APB1 and APB2
enum class APB_Prescaler : uint8_t {
    CKAHB_DIV1 = 0,
    CKAHB_DIV2 = 4,
    CKAHB_DIV4 = 5,
    CKAHB_DIV8 = 6,
    CKAHB_DIV16 = 7,
    INVALID = 8
};


///////////////////////////// ADC /////////////////////////////

enum class ADC_Prescaler : uint8_t {
    CKAPB2_DIV2 = 0,
    CKAPB2_DIV4 = 1,
    CKAPB2_DIV6 = 2,
    CKAPB2_DIV8 = 3,
    CKAPB2_DIV2B = 4,
    CKAPB2_DIV12 = 5,
    CKAPB2_DIV8B = 6,
    CKAPB2_DIV16 = 7,
    CKAHB_DIV5 = 8,
    CKAHB_DIV6 = 9,
    CKAHB_DIV10 = 10,
    CKAHB_DIV20 = 11,
    INVALID = 12
};


///////////////////////////// PLL /////////////////////////////

enum class PLL_Source : uint8_t {
    PLLSRC_IRC8M_DIV2,
    PLLSRC_HXTAL_IRC48M,
    PLLSRC_INVALID
};

struct PLL_Source_Mapping {
    bool value;
    PLL_Source source;
};

static inline constexpr std::array<PLL_Source_Mapping, 2> pll_mapping {{
    {false, PLL_Source::PLLSRC_IRC8M_DIV2},
    {true, PLL_Source::PLLSRC_HXTAL_IRC48M}
}};

enum class PLLMF_Select : uint8_t {
    PLL_MUL2,
    PLL_MUL3,
    PLL_MUL4,
    PLL_MUL5,
    PLL_MUL6,
    PLL_MUL7,
    PLL_MUL8,
    PLL_MUL9,
    PLL_MUL10,
    PLL_MUL11,
    PLL_MUL12,
    PLL_MUL13,
    PLL_MUL14,
    PLL_MUL15,
    PLL_MUL16,
    PLL_MUL16B,
    PLL_MUL17,
    PLL_MUL18,
    PLL_MUL19,
    PLL_MUL20,
    PLL_MUL21,
    PLL_MUL22,
    PLL_MUL23,
    PLL_MUL24,
    PLL_MUL25,
    PLL_MUL26,
    PLL_MUL27,
    PLL_MUL28,
    PLL_MUL29,
    PLL_MUL30,
    PLL_MUL31,
    PLL_MUL32
};

enum class PLL_Presel : uint8_t {
    PLLPRESRC_HXTAL,
    PLLPRESRC_IRC48M,
    PLLPRESRC_INVALID
};

struct PLL_Presel_Mapping {
    bool value;
    PLL_Presel source;
};

static inline constexpr std::array<PLL_Presel_Mapping, 2> pll_presel_mapping {{
    {false, PLL_Presel::PLLPRESRC_HXTAL},
    {true, PLL_Presel::PLLPRESRC_IRC48M}
}};


///////////////////////////// USB /////////////////////////////

enum class USB_Prescaler : uint8_t {
    DIV1_5,
    DIV1,
    DIV2_5,
    DIV2,
    DIV3,
    DIV3_5,
    DIV4
};


///////////////////////////// CKOUT0 /////////////////////////////

enum class CKOUT0_Source : uint8_t {
    CKOUT0SRC_NONE = 0,
    CKOUT0SRC_CKSYS = 4,
    CKOUT0SRC_IRC8M = 5,
    CKOUT0SRC_HXTAL = 6,
    CKOUT0SRC_CKPLL_DIV2 = 7
};


///////////////////////////// LXTAL /////////////////////////////

enum class LXTAL_Drive : uint8_t {
    LXTAL_LOWDRI,
    LXTAL_MED_LOWDRI,
    LXTAL_MED_HIGHDRI,
    LXTAL_HIGHDRI
};


///////////////////////////// RTC /////////////////////////////

enum class RTC_Source : uint8_t {
    RTCSRC_NONE,
    RTCSRC_LXTAL,
    RTCSRC_IRC40K,
    RTCSRC_HXTAL_DIV_128
};


///////////////////////////// DEEPSLEEP /////////////////////////////

enum class DeepSleep_Voltage : uint8_t {
    DEEPSLEEP_V_0,
    DEEPSLEEP_V_1,
    DEEPSLEEP_V_2,
    DEEPSLEEP_V_3
};


///////////////////////////// CK48M /////////////////////////////

enum class CK48M_Source : uint8_t {
    CK48MSRC_CKPLL,
    CK48MSRC_IRC48M
};


///////////////////////////// CONSTANTS /////////////////////////////

static inline constexpr uint32_t CoreClockFrequency = 120'000'000U;
static inline constexpr uint32_t HXTAL_VALUE = 8'000'000U;
static inline constexpr uint32_t IRC8M_VALUE = 8'000'000U;
static inline constexpr uint32_t IRC48M_VALUE = 48'000'000U;
static inline constexpr uint32_t IRC40K_VALUE = 40'000U;
static inline constexpr uint32_t LXTAL_VALUE = 32'768U;
static inline constexpr uint32_t OSC_STARTUP_TIMEOUT = 65'535U;
static inline constexpr uint32_t HXTAL_STARTUP_TIMEOUT = 65'535U;
static inline constexpr uint32_t LXTAL_STARTUP_TIMEOUT = 65'535U;
static inline constexpr uint32_t IRC8M_STARTUP_TIMEOUT = 1'280U;

static inline constexpr uint8_t AHB_EXP[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
static inline constexpr uint8_t APB1_EXP[8] = {0, 0, 0, 0, 1, 2, 3, 4};
static inline constexpr uint8_t APB2_EXP[8] = {0, 0, 0, 0, 1, 2, 3, 4};

} // namespace rcu
