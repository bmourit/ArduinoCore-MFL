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

namespace adc {

class RCU;

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

    // Operational mode specific methods
    inline void setup_regular_conversion();
    inline void cleanup_regular_conversion();
    inline void calibration_delay();
    inline uint32_t get_prescaler_value();
    uint32_t start_regular_single_conversion(ADC_Channel channel, ADC_Sample_Time sample, ADC_Resolution resolution, bool calibrate = false);

    // Accessor methods
    ADC_Base get_base() { return base_; }

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

    inline void set_sampling_time(ADC_Channel channel, ADC_Sample_Time sample_time);
    inline uint32_t configure_channel(uint32_t reg, uint8_t rank, ADC_Channel channel);

    template <ADC_Base Base>
    friend ADC& get_instance_for_base();
};

} // namespace adc
