//
// MFL gd32f30x ADC peripheral register access in C++
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

namespace adc {


///////////////////////////// BASE ADDRESS /////////////////////////////

enum class ADC_Base : uint32_t {
    ADC0_BASE,
    ADC1_BASE,
    ADC2_BASE,
    INVALID
};

static inline constexpr uintptr_t ADC_baseAddress[] = {
    0x40012400U, // ADC0
    0x40012800U, // ADC1
    0x40013C00U  // ADC2
};


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class ADC_Regs : uint32_t {
    STAT = 0x00U,
    CTL0 = 0x04U,
    CTL1 = 0x08U,
    SAMPT0 = 0x0CU,
    SAMPT1 = 0x10U,
    // NOTE:
    //  IOFF0-IOFF3
    //  These registers have no documentation in the GD32F30x manual.
    //  However, they are included here due to the following:
    //      A) The equivilant STM32 manual documents the registers.
    //      B) Testing has show drivers for this MCU sucessfuly use the registers.
    IOFF0 = 0x14U,
    IOFF1 = 0x18U,
    IOFF2 = 0x1CU,
    IOFF3 = 0x20U,
    WDHT = 0x24U,
    WDLT = 0x28U,
    RSQ0 = 0x2CU,
    RSQ1 = 0x30U,
    RSQ2 = 0x34U,
    // NOTE:
    //  ISQ
    //  This register has no documentation in the GD32F30x manual.
    //  See note above about IOFF0-IOFF3 for more details.
    ISQ = 0x38U,
    IDATA0 = 0x3CU,
    IDATA1 = 0x40U,
    IDATA2 = 0x44U,
    IDATA3 = 0x48U,
    RDATA = 0x4CU,
    OVSAMPCTL = 0x80U
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class CTL0_Bits : uint32_t {
    WDCHSEL = REG_BIT_DEF(0, 4),
    EOCIE = 5,
    WDEIE = 6,
    EOICIE = 7,
    SM = 8,
    WDSC = 9,
    ICA = 10,
    DISRC = 11,
    DISIC = 12,
    DISNUM = REG_BIT_DEF(13, 15),
    SYNCM = REG_BIT_DEF(16, 19),
    IWDEN = 22,
    RWDEN = 23,
    DRES = REG_BIT_DEF(24, 25)
};

enum class CTL1_Bits : uint32_t {
    ADCON = 0,
    CTN = 1,
    CLB = 2,
    RSTCLB = 3,
    DMA = 8,
    DAL = 11,
    ETSIC = REG_BIT_DEF(12, 14),
    ETEIC = 15,
    ETSRC = REG_BIT_DEF(17, 19),
    ETERC = 20,
    SWICST = 21,
    SWRCST = 22,
    TSVREN = 23
};

enum class SAMPTX_Bits : uint32_t {
    SPT0_10 = REG_BIT_DEF(0, 2),
    SPT1_11 = REG_BIT_DEF(3, 5),
    SPT2_12 = REG_BIT_DEF(6, 8),
    SPT3_13 = REG_BIT_DEF(9, 11),
    SPT4_14 = REG_BIT_DEF(12, 14),
    SPT5_15 = REG_BIT_DEF(15, 17),
    SPT6_16 = REG_BIT_DEF(18, 20),
    SPT7_17 = REG_BIT_DEF(21, 23),
    SPT8 = REG_BIT_DEF(24, 26),
    SPT9 = REG_BIT_DEF(27, 29)
};

enum class IOFFX_Bits : uint32_t {
    IOFF = REG_BIT_DEF(0, 11)
};

enum class WDHT_Bits : uint32_t {
    WDHT = REG_BIT_DEF(0, 11)
};

enum class WDLT_Bits : uint32_t {
    WDLT = REG_BIT_DEF(0, 11)
};

enum class RSQX_Bits : uint32_t {
    RSQ_0_6_12 = REG_BIT_DEF(0, 4),
    RSQ_1_7_13 = REG_BIT_DEF(5, 9),
    RSQ_2_8_14 = REG_BIT_DEF(10, 14),
    RSQ_3_9_15 = REG_BIT_DEF(15, 19),
    // RSQ2 and RSQ1 only
    RSQ_4_10 = REG_BIT_DEF(20, 24),
    RSQ_5_11 = REG_BIT_DEF(25, 29),
    // RSQ0 only
    RL = REG_BIT_DEF(20, 23)
};

enum class ISQ_Bits : uint32_t {
    ISQ_0_6_12 = REG_BIT_DEF(0, 4),
    ISQ_1_7_13 = REG_BIT_DEF(5, 9),
    ISQ_2_8_14 = REG_BIT_DEF(10, 14),
    ISQ_3_9_15 = REG_BIT_DEF(15, 19),
    // ISQ2 amd ISQ1 only
    ISQ_4_10 = REG_BIT_DEF(20, 24),
    ISQ_5_11 = REG_BIT_DEF(25, 29),
    // ISQ0 only
    IL = REG_BIT_DEF(20, 21)
};

enum class IDATAX_Bits : uint32_t {
    IDATAN = REG_BIT_DEF(0, 15)
};

enum class RDATA_Bits : uint32_t {
    RDATA = REG_BIT_DEF(0, 15),
    ADC1RDTR = REG_BIT_DEF(16, 31)
};

enum class OVSAMPCTL_Bits : uint32_t {
    OVSEN = 0,
    OVSR = REG_BIT_DEF(2, 4),
    OVSS = REG_BIT_DEF(5, 8),
    TOVS = 9,
    DRES = REG_BIT_DEF(12, 13)
};


///////////////////////////// ENUMS /////////////////////////////

enum class Sync_Mode : uint8_t {
    SM_FREE,	// Free (independant) mode
    SM_RPIP,	// Regular parallel inserted parallel mode
    SM_RPIR,	// Regular parallel trigger rotation mode
    SM_IPRFUF,	// Inserted parallel follow up fast mode
    SM_IPRFUS,	// Inserted parallel follow-up slow mode
    SM_IP,		// Inserted parallel mode only
    SM_RP,		// Regular parallel mode only
    SM_RFUF,	// Regular follow up fast mode only
    SM_RFUS, 	// Regular follow up slow mode only
    SM_ITR		// Inserted trigger rotation mode only
};

enum class Functional_Mode : uint8_t {
    SCAN_MODE,
    INSERTED_CH_MODE,
    CONTINUOUS_MODE,
    SCAN_INSERTED,
    SCAN_CONTINUOUS,
    SCAN_INSERTED_CONTINUOUS,
    INSERTED_CONTINUOUS
};

enum class Data_Alignment : uint8_t {
    DATA_ALIGN_RIGHT,
    DATA_ALIGN_LEFT
};

enum class External_Trigger_Source : uint8_t {
    ADC0_1_REGULAR_TIMER0_CH0 = 0,		// timer 0 CH0 event select
    ADC0_1_REGULAR_TIMER0_CH1 = 1,		// timer 0 CH1 event select
    ADC0_1_REGULAR_TIMER0_CH2 = 2,	 	// timer 0 CH2 event select
    ADC0_1_REGULAR_TIMER1_CH1 = 3, 		// timer 1 CH1 event select
    ADC0_1_REGULAR_TIMER2_TRGO = 4, 	// timer 2 TRGO event select
    ADC0_1_REGULAR_TIMER3_CH3 = 5, 		// timer 3 CH3 event select
    ADC0_1_REGULAR_TIMER7_TRGO = 6,		// timer 7 TRGO event select
    ADC0_1_REGULAR_EXTI_11 = 6, 		// external interrupt line 11
    ADC0_1_REGULAR_SOFTWARE = 7, 		// software trigger
    ADC2_REGULAR_TIMER2_CH0 = 0,		// timer 2 CH0 event select
    ADC2_REGULAR_TIMER1_CH2 = 1,		// timer 1 CH2 event select
    ADC2_REGULAR_TIMER0_CH2 = 2,		// timer 0 CH2 event select
    ADC2_REGULAR_TIMER7_CH0 = 3,		// timer 7 CH0 event select
    ADC2_REGULAR_TIMER7_TRGO = 4,		// timer 7 TRGO event select
    ADC2_REGULAR_TIMER4_CH0 = 5,		// timer 4 CH0 event select
    ADC2_REGULAR_TIMER4_CH2 = 6,		// timer 4 CH2 event select
    ADC0_1_INSERTED_TIMER0_TRGO = 0,	// timer 0 TRGO event select
    ADC0_1_INSERTED_TIMER0_CH3 = 1,		// timer 0 CH3 event select
    ADC0_1_INSERTED_TIMER1_TRGO = 2,	// timer 1 TRGO event select
    ADC0_1_INSERTED_TIMER1_CH0 = 3,		// timer 1 CH0 event select
    ADC0_1_INSERTED_TIMER2_CH3 = 4,		// timer 2 CH3 event select
    ADC0_1_INSERTED_TIMER3_TRGO = 5,	// timer 3 TRGO event select
    ADC0_1_INSERTED_EXTI_15 = 6,		// external interrupt line 15
    ADC0_1_INSERTED_TIMER7_CH3 = 6,		// timer 7 CH3 event select
    ADC0_1_2_INSERTED_SOFTWARE = 7,		// software trigger
    ADC2_INSERTED_TIMER0_TRGO = 0,		// timer 0 TRGO event select
    ADC2_INSERTED_TIMER0_CH3 = 1,		// timer 0 CH3 event select
    ADC2_INSERTED_TIMER3_CH2 = 2,		// timer 3 CH2 event select
    ADC2_INSERTED_TIMER7_CH1 = 3,		// timer 7 CH1 event select
    ADC2_INSERTED_TIMER7_CH3 = 4,		// timer 7 CH3 event select
    ADC2_INSERTED_TIMER4_TRGO = 5,		// timer 4 TRGO event select
    ADC2_INSERTED_TIMER4_CH3 = 6		// timer 4 CH3 event select
};

enum class ADC_Sample_Time : uint8_t {
    SAMPLETIME_1_5_CYCLES,		// 1.5 cycles
    SAMPLETIME_7_5_CYCLES,		// 7.5 cycles
    SAMPLETIME_13_5_CYCLES,		// 13.5 cycles
    SAMPLETIME_28_5_CYCLES,		// 28.5 cycles
    SAMPLETIME_41_5_CYCLES,		// 41.5 cycles
    SAMPLETIME_55_5_CYCLES,		// 55.5 cycles
    SAMPLETIME_71_5_CYCLES,		// 71.5 cycles
    SAMPLETIME_239_5_CYCLES 	// 239.5 cycles
};

enum class ADC_Resolution : uint8_t {
    RESOLUTION_12BIT,
    RESOLUTION_10BIT,
    RESOLUTION_8BIT,
    RESOLUTION_6BIT
};

enum class Oversampling_Shift : uint8_t {
    OVERSAMPLING_SHIFT_NONE,
    OVERSAMPLING_SHIFT_1BIT,
    OVERSAMPLING_SHIFT_2BIT,
    OVERSAMPLING_SHIFT_3BIT,
    OVERSAMPLING_SHIFT_4BIT,
    OVERSAMPLING_SHIFT_5BIT,
    OVERSAMPLING_SHIFT_6BIT,
    OVERSAMPLING_SHIFT_7BIT,
    OVERSAMPLING_SHIFT_8BIT
};

enum class Oversampling_Ratio : uint8_t {
    OVERSAMPLING_RATIO_MUL2,
    OVERSAMPLING_RATIO_MUL4,
    OVERSAMPLING_RATIO_MUL8,
    OVERSAMPLING_RATIO_MUL16,
    OVERSAMPLING_RATIO_MUL32,
    OVERSAMPLING_RATIO_MUL64,
    OVERSAMPLING_RATIO_MUL128,
    OVERSAMPLING_RATIO_MUL256
};

enum class Oversampling_Conversion : uint8_t {
    OVERSAMPLING_CONVERT_ALL,
    OVERSAMPLING_CONVERT_ONE
};

enum class Channel_Group_Type : uint8_t {
    REGULAR_CHANNEL = 1U,
    INSERTED_CHANNEL = 2U,
    REGULAR_INSERTED_CHANNEL = 3U,
    CHANNEL_DISCON_DISABLE = 4U
};

enum class Inserted_Channel : uint8_t {
    INSERTED_CHANNEL_0,
    INSERTED_CHANNEL_1,
    INSERTED_CHANNEL_2,
    INSERTED_CHANNEL_3
};

enum class ADC_Channel : uint8_t {
    CHANNEL_0,
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_3,
    CHANNEL_4,
    CHANNEL_5,
    CHANNEL_6,
    CHANNEL_7,
    CHANNEL_8,
    CHANNEL_9,
    CHANNEL_10,
    CHANNEL_11,
    CHANNEL_12,
    CHANNEL_13,
    CHANNEL_14,
    CHANNEL_15,
    CHANNEL_16,
    CHANNEL_17,
    INVALID
};

enum class Status_Flags : uint8_t {
    FLAG_WDE = 0,
    FLAG_EOC = 1,
    FLAG_EOIC = 2,
    FLAG_STIC = 3,
    FLAG_STRC = 4
};

enum class Interrupt_Flags : uint8_t {
    INTR_FLAG_WDE = 0,
    INTR_FLAG_EOC = 1,
    INTR_FLAG_EOIC = 2
};

enum class Interrupt_Type : uint8_t {
    INTR_WDE = 5,
    INTR_EOC = 6,
    INTR_EOIC = 7
};

enum class ADC_Error_Type : uint8_t {
    OK,
    INVALID_ADC,
    INVALID_OPERATION,
    INITIALIZATION_FAILED,
    INVALID_SELECTION
};


///////////////////////////// CONSTANTS /////////////////////////////

static inline constexpr uint32_t Calibration_Delay_Cycles = 14U;


///////////////////////////// STRUCTURES /////////////////////////////

struct ADC_Clock_Config {
    rcu::RCU_PCLK clock_reg;
    rcu::RCU_PCLK_Reset reset_reg;
};

static inline constexpr std::array<ADC_Clock_Config, 3> ADC_pclk_index {{
        {rcu::RCU_PCLK::PCLK_ADC0, rcu::RCU_PCLK_Reset::PCLK_ADC0RST},
        {rcu::RCU_PCLK::PCLK_ADC1, rcu::RCU_PCLK_Reset::PCLK_ADC1RST},
        {rcu::RCU_PCLK::PCLK_ADC2, rcu::RCU_PCLK_Reset::PCLK_ADC2RST}
    }};


///////////////////////////// UNUSED DOCUMENTATION ONLY /////////////////////////////

/*
enum class STAT_Bits : uint32_t {
    WDE = REG_BIT_DEF(0, 0),
    EOC = REG_BIT_DEF(1, 1),
    EOIC = REG_BIT_DEF(2, 2),
    STIC = REG_BIT_DEF(3, 3),
    STRC = REG_BIT_DEF(4, 4)
};
*/


}	// namespace adc
