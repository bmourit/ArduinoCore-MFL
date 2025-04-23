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

#include <stdint.h>
#include <array>

#include "adc_config.hpp"
#include "ErrorTypes.hpp"
#include "RegRW.hpp"
#include "RCU.hpp"

namespace adc {

class ADC {
public:
    static Result<ADC, ADC_Error_Type> get_instance(ADC_Base Base);

    // Reset
    void reset();
    // Enable
    void enable();
    void disable();
    void set_enable(bool enable);
    bool is_enabled();
    // Configuration
    void calibration_enable();
    void set_dma_enable(bool enable);
    void set_temperature_voltage_reference_enable(bool enable);
    void set_resolution(ADC_Resolution resolution);
    void set_group_channel_discontinuous_mode(Channel_Group_Type channel_group, uint8_t length);
    void set_mode(Sync_Mode mode);
    void set_functional_mode(Functional_Mode function, bool enable);
    void set_data_alignment(Data_Alignment align);
    void set_channel_length(Channel_Group_Type channel_group, uint32_t length);
    void set_regular_channel_sequence(uint8_t rank, ADC_Channel channel, ADC_Sample_Time sample_time);
    void set_inserted_channel_sequence(uint8_t rank, ADC_Channel channel, ADC_Sample_Time sample_time);
    void set_inserted_channel_offset(Inserted_Channel inserted_channel, uint16_t offset);
    // Triggers
    void set_external_trigger_enable(Channel_Group_Type channel_group, bool enable);
    void set_external_group_source(Channel_Group_Type channel_group, External_Trigger_Source source);
    void set_software_trigger_group(Channel_Group_Type channel_group);
    // Read data
    uint32_t get_regular_data();
    uint32_t get_inserted_data(Inserted_Channel inserted_channel);
    uint32_t get_sync_mode_data();
    // Watchdog
    void single_channel_watchdog_enable(ADC_Channel channel);
    void group_channel_watchdog_enable(Channel_Group_Type channel_group);
    void watchdog_disable();
    void set_watchdog_threshold(uint16_t low, uint16_t high);
    // Oversample
    void set_oversampling_configuration(Oversampling_Conversion mode, Oversampling_Shift shift, Oversampling_Ratio ratio);
    void set_oversampling_enable(bool enable);
    // Interrupts and flags
    bool get_flag(Status_Flags flag);
    void clear_flag(Status_Flags flag);
    bool get_interrupt_flag(Interrupt_Flags flag);
    void clear_interrupt_flag(Interrupt_Flags flag);
    void set_interrupt_enable(Interrupt_Type type, bool enable);

    // Operational mode specific
    uint32_t start_regular_single_conversion(ADC_Channel channel, ADC_Sample_Time sample, ADC_Resolution resolution, bool calibrate = false);

    // Accessor methods
    inline ADC_Base get_base() { return base_; }

    inline volatile uint32_t* reg_address(ADC_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(base_address_ + static_cast<uint32_t>(reg));
    }
    inline volatile uint32_t* reg_address(ADC_Regs reg, uint32_t extra_offset) const {
        return reinterpret_cast<volatile uint32_t*>(base_address_ + static_cast<uint32_t>(reg) + extra_offset);
    }

private:
    static std::array<bool, static_cast<size_t>(ADC_Base::INVALID)> clock_enabled_;
    explicit ADC(ADC_Base Base);

    ADC_Base base_;
    ADC_Clock_Config ADC_pclk_info_;
    uint32_t base_address_;
    uint32_t prescaler_;

    inline void set_sampling_time(ADC_Channel channel, ADC_Sample_Time sample_time) {
        if (channel == ADC_Channel::INVALID) {
            return;
        }

        const uint32_t chan = static_cast<uint32_t>(channel);
        const uint32_t sampt = static_cast<uint32_t>(sample_time);
        const bool use_sampt0 = (chan >= 10U);

        const ADC_Regs reg = use_sampt0 ? ADC_Regs::SAMPT0 : ADC_Regs::SAMPT1;
        const uint32_t shift = use_sampt0 ? (chan - 10U) * 3U : chan * 3U;
        const uint32_t mask = ~(0x7U << shift);

        uint32_t reg_value = read_register<uint32_t>(*this, reg);
        reg_value = (reg_value & mask) | (sampt << shift);
        write_register(*this, reg, reg_value);

        // Handle temperature temperature/reference voltage channels
        if (chan == 16U || chan == 17U) {
            set_temperature_voltage_reference_enable(true);
            const uint32_t system_clock = RCU_I.get_system_clock();
            uint32_t delay_cycles;

            // Calculate delay cycles based on the system clock frequency
            if (system_clock >= 100'000'000U) {
                delay_cycles = system_clock / 100'000U;
            } else if (system_clock >= 10'000'000U) {
                delay_cycles = system_clock / 10'000U;
            } else {
                delay_cycles = system_clock / 1'000U;
            }

            // Delay the calculated number of cycles
            for (uint32_t i = 0; i < delay_cycles; ++i) {
                __asm__ volatile("" ::: "memory");
            }
        }
    }

    template <ADC_Base Base>
    friend ADC& get_instance_for_base();

public:
    // Inlined Operational mode specific methods
    inline void setup_regular_conversion() {
        // Make sure flag is cleared
        clear_flag(Status_Flags::FLAG_EOC);
        // Internal channels not supported yet in this function
        write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::TSVREN), false);
        // In single conversion mode length is forced to zero
        write_bit_range(*this, ADC_Regs::RSQ0, static_cast<uint32_t>(RSQX_Bits::RL), Clear);
        // Disable conflicting modes
        write_bits_sequence(*this, ADC_Regs::CTL0,
                            static_cast<uint32_t>(CTL0_Bits::DISRC), false,
                            static_cast<uint32_t>(CTL0_Bits::SM), false);
        write_bit_range(*this, ADC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SYNCM), Clear);
        write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::CTN), false);
    }

    inline void cleanup_regular_conversion() {
        write_register(*this, ADC_Regs::RSQ2, Clear);
        write_register(*this, ADC_Regs::RSQ1, Clear);
        write_register(*this, ADC_Regs::RSQ0, Clear);
        write_register(*this, ADC_Regs::SAMPT0, Clear);
        write_register(*this, ADC_Regs::SAMPT1, Clear);
        write_bit_range(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ETSRC), Clear);
        write_bits_sequence(*this, ADC_Regs::CTL1,
                            static_cast<uint32_t>(CTL1_Bits::ETERC), false,
                            static_cast<uint32_t>(CTL1_Bits::SWRCST), false);
    }

    inline void calibration_delay() {
        // Hardware requires delay before starting calibration
        if (prescaler_ == 0U) {
            return;
        }

        // Get clock frequencies
        const uint32_t system_clock = RCU_I.get_system_clock();
        const uint32_t apb2_clock = RCU_I.get_clock_frequency(rcu::Clock_Frequency::CK_APB2);

        // Calculate delay cycles
        const uint32_t wait_count = (system_clock * prescaler_ / apb2_clock) * Calibration_Delay_Cycles;

        // Delay required number of cycles
        volatile uint32_t i = wait_count;
        while (i != 0) {
            i = i - 1U;
        }
    }

    inline uint32_t get_prescaler_value() {
        const rcu::ADC_Prescaler adc_prescaler = RCU_I.get_adc_prescaler();
        const uint8_t index = static_cast<uint8_t>(adc_prescaler);

        // Array of prescaler values corresponding to ADC_Prescaler values
        static constexpr uint32_t prescaler_values[] = {
            2U, 4U, 6U, 8U, 2U, 12U, 8U, 16U, 5U, 6U, 10U, 20U, 0U
        };

        // Bounds check to prevent array out-of-bounds access
        if (index < (sizeof(prescaler_values) / sizeof(prescaler_values[0]))) {
            return prescaler_values[index];
        }

        return 0U;  // Default for any unexpected values
    }
};


} // namespace adc
