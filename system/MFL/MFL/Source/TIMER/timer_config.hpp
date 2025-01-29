//
// MFL gd32f30x TIMER peripheral register access in C++
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

namespace timer {


///////////////////////////// BASE ADDRESS /////////////////////////////

enum class TIMER_Base : uint32_t {
    TIMER0_BASE,
    TIMER1_BASE,
    TIMER2_BASE,
    TIMER3_BASE,
    TIMER4_BASE,
    TIMER5_BASE,
    TIMER6_BASE,
    TIMER7_BASE,
    INVALID
};

static inline constexpr uintptr_t TIMER_baseAddress[] = {
    0x40012C00U, // TIMER0
    0x40000000U, // TIMER1
    0x40000400U, // TIMER2
    0x40000800U, // TIMER3
    0x40000C00U, // TIMER4
    0x40001000U, // TIMER5
    0x40001400U, // TIMER6
    0x40013400U  // TIMER7
};


///////////////////////////// REGISTER OFFSETS /////////////////////////////

enum class TIMER_Regs : uint32_t {
    CTL0 = 0x00U,
    CTL1 = 0x04U,
    SMCFG = 0x08U,
    DMAINTEN = 0x0CU,
    INTF = 0x10U,
    SWEVG = 0x14U,
    CHCTL0 = 0x18U,
    CHCTL1 = 0x1CU,
    CHCTL2 = 0x20U,
    CNT = 0x24U,
    PSC = 0x28U,
    CAR = 0x2CU,
    CREP = 0x30U,
    CH0CV = 0x34U,
    CH1CV = 0x38U,
    CH2CV = 0x3CU,
    CH3CV = 0x40U,
    CCHP = 0x44U,
    DMACFG = 0x48U,
    DMATB = 0x4CU,
    IRMP = 0x50U,
    CFG = 0xFCU
};


///////////////////////////// REGISTER BITS /////////////////////////////

enum class CTL0_Bits : uint32_t {
    CEN = 0,
    UPDIS = 1,
    UPS = 2,
    SPM = 3,
    DIR = 4,
    CAM = REG_BIT_DEF(5, 6),
    ARSE = 7,
    CKDIV = REG_BIT_DEF(8, 9)
};

enum class CTL1_Bits : uint32_t {
    CCSE = 0,
    CCUC = 2,
    DMAS = 3,
    MMC = REG_BIT_DEF(4, 6),
    TI0S = 7,
    ISO0 = 8,
    ISO0N = 9,
    ISO1 = 10,
    ISO1N = 11,
    ISO2 = 12,
    ISO2N = 13,
    ISO3 = 14
};

enum class SMCFG_Bits : uint32_t {
    SMC = REG_BIT_DEF(0, 2),
    TRGS = REG_BIT_DEF(4, 6),
    MSM = 7,
    ETFC = REG_BIT_DEF(8, 11),
    ETPSC = REG_BIT_DEF(12, 13),
    SMC1 = 14,
    ETP = 15
};

enum class DMAINTEN_Bits : uint32_t {
    UPIE = 0,
    CH0IE = 1,
    CH1IE = 2,
    CH2IE = 3,
    CH3IE = 4,
    CMTIE = 5,
    TRGIE = 6,
    BRKIE = 7,
    UPDEN = 8,
    CH0DEN = 9,
    CH1DEN = 10,
    CH2DEN = 11,
    CH3DEN = 12,
    CMTDEN = 13,
    TRGDEN = 14
};

enum class INTF_Bits : uint32_t {
    UPIF = 0,
    CH0IF = 1,
    CH1IF = 2,
    CH2IF = 3,
    CH3IF = 4,
    CMTIF = 5,
    TRGIF = 6,
    BRKIF = 7,
    CH0OF = 9,
    CH1OF = 10,
    CH2OF = 11,
    CH3OF = 12
};

enum class SWEVG_Bits : uint32_t {
    UPG = 0,
    CH0G = 1,
    CH1G = 2,
    CH2G = 3,
    CH3G = 4,
    CMTG = 5,
    TRGG = 6,
    BRKG = 7
};

enum class CHCTL0_Bits : uint32_t {
    // Output compare mode
    CH0MS = REG_BIT_DEF(0, 1),
    CH0COMFEN = 2,
    CH0COMSEN = 3,
    CH0COMCTL = REG_BIT_DEF(4, 6),
    CH0COMCEN = 7,
    CH1MS = REG_BIT_DEF(8, 9),
    CH1COMFEN = 10,
    CH1COMSEN = 11,
    CH1COMCTL = REG_BIT_DEF(12, 14),
    CH1COMCEN = 15,
    // Input capture mode
    CH0CAPPSC = REG_BIT_DEF(2, 3),
    CH0CAPFLT = REG_BIT_DEF(4, 7),
    CH1CAPPSC = REG_BIT_DEF(10, 11),
    CH1CAPFLT = REG_BIT_DEF(12, 15)
};

enum class CHCTL1_Bits : uint32_t {
    // Output compare mode
    CH2MS = REG_BIT_DEF(0, 1),
    CH2COMFEN = 2,
    CH2COMSEN = 3,
    CH2COMCTL = REG_BIT_DEF(4, 6),
    CH2COMCEN = 7,
    CH3MS = REG_BIT_DEF(8, 9),
    CH3COMFEN = 10,
    CH3COMSEN = 11,
    CH3COMCTL = REG_BIT_DEF(12, 14),
    CH3COMCEN = 15,
    // Input campture mode
    CH2CAPPSC = REG_BIT_DEF(2, 3),
    CH2CAPFLT = REG_BIT_DEF(4, 7),
    CH3CAPPSC = REG_BIT_DEF(10, 11),
    CH3CAPFLT = REG_BIT_DEF(12, 15)
};

enum class CHCTL2_Bits : uint32_t {
    CH0EN = 0,
    CH0P = 1,
    CH0NEN = 2,
    CH0NP = 3,
    CH1EN = 4,
    CH1P = 5,
    CH1NEN = 6,
    CH1NP = 7,
    CH2EN = 8,
    CH2P = 9,
    CH2NEN = 10,
    CH2NP = 11,
    CH3EN = 12,
    CH3P = 13
};

enum class CNT_Bits : uint32_t {
    CNT = REG_BIT_DEF(0, 15)
};

enum class PSC_Bits : uint32_t {
    PSC = REG_BIT_DEF(0, 15)
};

enum class CAR_Bits : uint32_t {
    CARL = REG_BIT_DEF(0, 15)
};

enum class CREP_Bits : uint32_t {
    CREP = REG_BIT_DEF(0, 7)
};

enum class CHXCV_Bits : uint32_t {
    CHXCV = REG_BIT_DEF(0, 15)
};

enum class CCHP_Bits : uint32_t {
    DTCFG = REG_BIT_DEF(0, 7),
    PROT = REG_BIT_DEF(8, 9),
    IOS = 10,
    ROS = 11,
    BRKEN = 12,
    BRKP = 13,
    OAEN = 14,
    POEN = 15
};

enum class DMACFG_Bits : uint32_t {
    DMATA = REG_BIT_DEF(0, 4),
    DMATC = REG_BIT_DEF(8, 12)
};

enum class DMATB_Bits : uint32_t {
    DMATB = REG_BIT_DEF(0, 15)
};

enum class CFG_Bits : uint32_t {
    OUTSEL = 0,
    CHVSEL = 1
};


///////////////////////////// ENUMS /////////////////////////////

enum class DMA_Select : uint32_t {
    DMA_UP = 8,
    DMA_CH0 = 9,
    DMA_CH1 = 10,
    DMA_CH2 = 11,
    DMA_CH3 = 12,
    DMA_CMT = 13,
    DMA_TRG = 14
};

enum class Status_Flags : uint32_t {
    FLAG_UPIF = 0,
    FLAG_CH0 = 1,
    FLAG_CH1 = 2,
    FLAG_CH2 = 3,
    FLAG_CH3 = 4,
    FLAG_CMT = 5,
    FLAG_TRG = 6,
    FLAG_BRK = 7,
    FLAG_CH0O = 9,
    FLAG_CH1O = 10,
    FLAG_CH2O = 11,
    FLAG_CH3O = 12,
    INVALID = 15
};

enum class Interrupt_Flags : uint32_t {
    INTR_FLAG_UP = 0,
    INTR_FLAG_CH0 = 1,
    INTR_FLAG_CH1 = 2,
    INTR_FLAG_CH2 = 3,
    INTR_FLAG_CH3 = 4,
    INTR_FLAG_CMT = 5,
    INTR_FLAG_TRG = 6,
    INTR_FLAG_BRK = 7,
    INVALID = 15
};

enum class Interrupt_Type : uint32_t {
    INTR_UPIE = 0,
    INTR_CH0IE = 1,
    INTR_CH1IE = 2,
    INTR_CH2IE = 3,
    INTR_CH3IE = 4,
    INTR_CMTIE = 5,
    INTR_TRGIE = 6,
    INTR_BRKIE = 7,
    INVALID = 15
};

enum class DMA_Request : uint8_t {
    CHANNEL_EVENT,
    UPDATE_EVENT
};

// DMA transfer access
enum class DMA_Transfer_Address : uint8_t {
    DMATA_CTL0,
    DMATA_CTL1,
    DMATA_SMCFG,
    DMATA_DMAINTEN,
    DMATA_INTF,
    DMATA_SWEVG,
    DMATA_CHCTL0,
    DMATA_CHCTL1,
    DMATA_CHCTL2,
    DMATA_CNT,
    DMATA_PSC,
    DMATA_CAR,
    DMATA_CREP,
    DMATA_CH0CV,
    DMATA_CH1CV,
    DMATA_CH2CV,
    DMATA_CH3CV,
    DMATA_CCHP,
    DMATA_DMACFG,
    DMATA_DMATB
};

// DMA transfer count (burst length)
enum class DMA_Burst_Length : uint8_t {
    DMATC_TRANSFER_COUNT1,
    DMATC_TRANSFER_COUNT2,
    DMATC_TRANSFER_COUNT3,
    DMATC_TRANSFER_COUNT4,
    DMATC_TRANSFER_COUNT5,
    DMATC_TRANSFER_COUNT6,
    DMATC_TRANSFER_COUNT7,
    DMATC_TRANSFER_COUNT8,
    DMATC_TRANSFER_COUNT9,
    DMATC_TRANSFER_COUNT10,
    DMATC_TRANSFER_COUNT11,
    DMATC_TRANSFER_COUNT12,
    DMATC_TRANSFER_COUNT13,
    DMATC_TRANSFER_COUNT14,
    DMATC_TRANSFER_COUNT15,
    DMATC_TRANSFER_COUNT16,
    DMATC_TRANSFER_COUNT17,
    DMATC_TRANSFER_COUNT18
};

enum class Event_Source : uint32_t {
    EVENT_SRC_UPG = 0,
    EVENT_SRC_CH0G = 1,
    EVENT_SRC_CH1G = 2,
    EVENT_SRC_CH2G = 3,
    EVENT_SRC_CH3G = 4,
    EVENT_SRC_CMTG = 5,
    EVENT_SRC_TRGG = 6,
    EVENT_SRC_BRKG = 7
};

// Center aligned mode
enum class Center_Align : uint8_t {
    EDGE,
    CENTER_DOWN,
    CENTER_UP,
    CENTER_BOTH
};

enum class PSC_Reload : uint8_t {
    RELOAD_NOW,
    RELOAD_UPDATE
};

enum class Count_Direction : uint8_t {
    UP,
    DOWN
};

enum class Division_Ratio : uint8_t {
    DIV1,
    DIV2,
    DIV4
};

// Single pulse mode
enum class Pulse_Mode : uint8_t {
    REPEAT_PULSE,
    SINGLE_PULSE
};

enum class Update_Source : uint8_t {
    GLOBAL_SOURCE,
    REGULAR_SOURCE
};

// Run off-state mode
enum class ROS_State : uint8_t {
    ROS_DISABLE,
    ROS_ENABLE
};

// Idle off-state mode
enum class IOS_State : uint8_t {
    IOS_DISABLE,
    IOS_ENABLE
};

// Break input polarity
enum class Break_Polarity : uint8_t {
    BREAK_LOW,
    BREAK_HIGH
};

// Output polarity 
enum class Output_Polarity : uint8_t {
    OUTPUT_HIGH,
    OUTPUT_LOW
};

// Output auto
enum class Output_Auto : uint8_t {
    OUTPUT_AUTO_DISABLE,
    OUTPUT_AUTO_ENABLE
};

// Companion channel protect
enum class CCHP_Protect : uint8_t {
    CCHP_OFF,
    CCHP_PROT0,
    CCHP_PROT1,
    CCHP_PROT2
};

// Break input
enum class Break_Input : uint8_t {
    BREAK_DISABLE,
    BREAK_ENABLE
};

enum class Timer_Channel : uint8_t {
    CH0,
    CH1,
    CH2,
    CH3,
    INVALID
};

// Generic polarity
enum class Polarity_Select : uint8_t {
    HIGH_RISING,
    LOW_FALLING
};

enum class Idle_State : uint8_t {
    IDLE_LOW,
    IDLE_HIGH
};

enum class Output_Compare_Mode : uint8_t {
    OC_TIMING_MODE,
    OC_OUTPUT_MODE,
    OC_CLEAR_MODE,
    OC_TOGGLE_MODE,
    OC_FORCE_LOW_MODE,
    OC_FORCE_HIGH_MODE,
    OC_PWM_MODE0,
    OC_PWM_MODE1
};

enum class Output_Compare_State : uint8_t {
    OC_DISABLE,
    OC_ENABLE
};

enum class Output_Compare_Shadow : uint8_t {
    OC_SHADOW_DISABLE,
    OC_SHADOW_ENABLE
};

enum class Output_Compare_Fast : uint8_t {
    OC_FAST_DISABLE,
    OC_FAST_ENABLE
};

enum class Output_Compare_Clear : uint8_t {
    OC_CLEAR_DISABLE,
    OC_CLEAR_ENABLE
};

// Channel control shadow update
enum class Shadow_Update : uint8_t {
    SHADOW_CCU,
    SHADOW_CCUTRI
};

//
// Input_Capture_Select
//
// For CH0MS:
//      Generate trigger from input source connected to CI0FE0
// For CH0MS:
//      Generate trigger from input source connected to CI1FE0
// ITS:
//      Generate trigger from selected internal trigger source
//
enum class Input_Capture_Select : uint8_t {
    IO_OUTPUT,
    IO_INPUT_CI0FE0,
    IO_INPUT_CI1FE0,
    IO_INPUT_ITS
};

enum class Input_Capture_Prescaler : uint8_t {
    DIV1,
    DIV2,
    DIV4,
    DIV8
};

enum class Trigger_Select : uint8_t {
    ITI0,
    ITI1,
    ITI2,
    ITI3,
    CI0F_ED,
    CI0FE0,
    CI1FE1,
    ETIFP
};

enum class Master_Control : uint8_t {
    RESET,
    ENABLE,
    UPDATE,
    CH0,
    O0CPRE,
    O1CPRE,
    O2CPRE,
    O3CPRE
};

enum class Slave_Control : uint8_t {
    DISABLE = 0,
    RESTART = 4,
    PAUSE = 5,
    EVENT = 6,
    EXTERNAL0 = 7
};

enum class Decode_Mode : uint8_t {
    ENCODER0 = 1,
    ENCODER1 = 2,
    ENCODER2 = 3
};

enum class External_Trigger_Prescaler : uint8_t {
    OFF,
    DIV2,
    DIV4,
    DIV8
};

enum class Hall_Interface : uint8_t {
    HALL_ENABLE,
    HALL_DISABLE
};

enum class TIMER_Error_Type : uint8_t {
    OK,
    INVALID_TIMER,
    INVALID_OPERATION,
    INITIALIZATION_FAILED,
    INVALID_SELECTION
};


///////////////////////////// STRUCTURES /////////////////////////////

struct TIMER_Clock_Config {
    rcu::RCU_PCLK clock_reg;
    rcu::RCU_PCLK_Reset reset_reg;
};

static inline constexpr std::array<TIMER_Clock_Config, 8> TIMER_pclk_index {{
    {rcu::RCU_PCLK::PCLK_TIMER0, rcu::RCU_PCLK_Reset::PCLK_TIMER0RST},
    {rcu::RCU_PCLK::PCLK_TIMER1, rcu::RCU_PCLK_Reset::PCLK_TIMER1RST},
    {rcu::RCU_PCLK::PCLK_TIMER2, rcu::RCU_PCLK_Reset::PCLK_TIMER2RST},
    {rcu::RCU_PCLK::PCLK_TIMER3, rcu::RCU_PCLK_Reset::PCLK_TIMER3RST},
    {rcu::RCU_PCLK::PCLK_TIMER4, rcu::RCU_PCLK_Reset::PCLK_TIMER4RST},
    {rcu::RCU_PCLK::PCLK_TIMER5, rcu::RCU_PCLK_Reset::PCLK_TIMER5RST},
    {rcu::RCU_PCLK::PCLK_TIMER6, rcu::RCU_PCLK_Reset::PCLK_TIMER6RST},
    {rcu::RCU_PCLK::PCLK_TIMER7, rcu::RCU_PCLK_Reset::PCLK_TIMER7RST}
}};

struct TIMER_Break {
    uint16_t dead_time;
    Break_Input break_state;
    Break_Polarity break_polarity;
    Output_Auto output_auto_state;
    ROS_State ros_state;
    IOS_State ios_state;
    CCHP_Protect protection;
};

struct TIMER_Input_Capture {
    uint16_t digital_filter;
    Polarity_Select polarity;
    Input_Capture_Select source_select;
    Input_Capture_Prescaler prescaler;
};

struct TIMER_Output_Compare {
    Output_Compare_State state;
    Output_Compare_State companion_state;
    Polarity_Select polarity;
    Polarity_Select companion_polarity;
    Idle_State idle_state;
    Idle_State companion_idle_state;
};

struct TIMER_Config {
    uint32_t prescaler;
    uint32_t period;
    uint32_t repetition_count;
    Division_Ratio divider;
    Center_Align align;
    Count_Direction counting_direction;
};


///////////////////////////// INITIALIZATION DEFAULTS /////////////////////////////

static inline const TIMER_Config default_config = {
    0U,
    0xFFFFFFFFU,
    0U,
    Division_Ratio::DIV1,
    Center_Align::EDGE,
    Count_Direction::UP
};

static inline const TIMER_Break default_break = {
    0U,
    Break_Input::BREAK_DISABLE,
    Break_Polarity::BREAK_LOW,
    Output_Auto::OUTPUT_AUTO_DISABLE,
    ROS_State::ROS_DISABLE,
    IOS_State::IOS_DISABLE,
    CCHP_Protect::CCHP_OFF
};

static inline const TIMER_Input_Capture default_capture = {
    0U,
    Polarity_Select::HIGH_RISING,
    Input_Capture_Select::IO_INPUT_CI0FE0,
    Input_Capture_Prescaler::DIV1
};

static inline const TIMER_Output_Compare default_compare = {
    Output_Compare_State::OC_DISABLE,
    Output_Compare_State::OC_DISABLE,
    Polarity_Select::HIGH_RISING,
    Polarity_Select::HIGH_RISING,
    Idle_State::IDLE_LOW,
    Idle_State::IDLE_LOW
};

} // namespace timer
