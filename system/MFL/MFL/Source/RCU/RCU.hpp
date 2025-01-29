//
// MFL gd32f30x RCU peripheral register access in C++
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

#include <stdint.h>

#include "rcu_config.hpp"
#include "RegRW.hpp"

namespace rcu {

class RCU {
public:
    static RCU& get_instance();

    // Reset
    void reset();
    // Peripheral clocks
    void set_pclk_enable(RCU_PCLK pclk, bool enable);
    // Check if pclk is enabled
    bool get_pclk(RCU_PCLK pclk);
    // peripheral clock sleep
    void set_pclk_sleep_enable(RCU_PCLK_Sleep pclk, bool enable);
    // peripheral clock reset
    void set_pclk_reset_enable(RCU_PCLK_Reset pclk, bool enable);
    // Backup clock reset
    void set_backup_reset_enable(bool enable);
    // System clock source
    void set_system_source(System_Clock_Source system_source);
    System_Clock_Source get_system_source();
    // AHB/ABP1/APB2 prescaler
    void set_ahb_prescaler(AHB_Prescaler prescaler);
    AHB_Prescaler get_ahb_prescaler();
    void set_apb1_prescaler(APB_Prescaler prescaler);
    APB_Prescaler get_apb1_prescaler();
    void set_apb2_prescaler(APB_Prescaler prescaler);
    APB_Prescaler get_apb2_prescaler();
    // CKOUT0 source
    void set_ckout0_source(CKOUT0_Source source);
    // PLL
    void set_pll_config(PLL_Source source, PLLMF_Select multiplier);
    PLL_Source get_pll_source();
    void set_pll_presel(PLL_Presel presel);
    PLL_Presel get_pll_presel();
    // PREDV0
    void set_predv0_config(bool div);
    // ADC prescaler
    void set_adc_prescaler(ADC_Prescaler prescaler);
    ADC_Prescaler get_adc_prescaler();
    // USB prescaler
    void set_usb_prescaler(USB_Prescaler prescaler);
    // RTC source
    void set_rtc_source(RTC_Source source);
    // CK48M source
    void set_ck48m_source(CK48M_Source source);
    // LXTAL campability
    void set_lxtal_drive_capability(LXTAL_Drive drive);
    // OSCI
    void set_osci_enable(OSCI_Select osci, bool enable);
    bool is_osci_stable(OSCI_Select osci);
    // Clock frequency
    uint32_t get_clock_source_frequency(System_Clock_Source source);
    uint32_t calculate_pll_frequency();
    uint32_t get_pll_multiplier();
    uint32_t get_clock_frequency(Clock_Frequency clock);
    // Bypass
    void set_bypass_mode_enable(OSCI_Select osci, bool enable);
    // Adjust
    void set_irc8m_adjustment_value(uint32_t value);
    // Monitor
    void set_hxtal_monitor_enable(bool enable);
    // Deep sleep
    void set_deep_sleep_voltage(DeepSleep_Voltage voltage);
    // Interrupts and flags
    bool get_flag(Status_Flags flag);
    void clear_all_reset_flags();
    bool get_interrupt_flag(Interrupt_Flags flag);
    void clear_interrupt_flag(Clear_Flags flag);
    void set_interrupt_enable(Interrupt_Type type, bool enable);

    // System clock handling
    uint32_t get_system_clock() { return SystemCoreClock; }
    void update_system_clock();

    static inline constexpr uintptr_t RCU_baseAddress = 0x40021000U;

    inline volatile uint32_t* reg_address(RCU_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(RCU_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    RCU();

    // Prevent copying or assigning
    RCU(const RCU&) = delete;
    RCU& operator=(const RCU&) = delete;

    uint32_t SystemCoreClock;

    // Get value helpers
    inline bool get_value(Status_Flags flag) {
        const auto& info = status_flag_index[static_cast<size_t>(flag)];
        uint32_t reg_value = read_register<uint32_t>(*this, info.register_offset);
        const uint32_t width = info.bit_info & 0xFFU;
        const uint32_t bitno = info.bit_info >> 16U;
        reg_value >>= bitno;
        reg_value &= ((1U << width) - 1U);
        return reg_value;
    }

    inline bool get_value(Interrupt_Flags flag) {
        const auto& info = interrupt_type_index[static_cast<size_t>(flag)];
        uint32_t reg_value = read_register<uint32_t>(*this, info.register_offset);
        const uint32_t width = info.bit_info & 0xFFU;
        const uint32_t bitno = info.bit_info >> 16U;
        reg_value >>= bitno;
        reg_value &= ((1U << width) - 1U);
        return reg_value;
    }
};


} // namespace rcu

extern rcu::RCU& RCU_I;
