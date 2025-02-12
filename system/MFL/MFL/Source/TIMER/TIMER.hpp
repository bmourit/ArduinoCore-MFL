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

#include <stdint.h>
#include <array>

#include "timer_config.hpp"
#include "ErrorTypes.hpp"
#include "RegRW.hpp"

namespace timer {

class RCU;

class TIMER {
public:
    static Result<TIMER, TIMER_Error_Type> get_instance(TIMER_Base Base);

    // Initialization
    void init(TIMER_Config config = default_config);
    void reset();
    // Enable
    void enable();
    void disable();
    void set_enable(bool enable);
    // Events
    void update_event_enable();
    void update_event_disable();
    void set_update_event_enable(bool enable);
    void generate_software_event(Event_Source event);
    void set_update_source(Update_Source source);
    // Counter
    void count_direction_up();
    void count_direction_down();
    void counter_alignment(Center_Align alignment);
    void set_counter_value(uint16_t counter);
    uint32_t get_counter();
    // Precaler
    void set_prescaler_reload(uint16_t prescaler, PSC_Reload reload);
    uint16_t get_prescaler();
    // Auto reload and repetition
    void set_auto_reload_shadow_enable(bool enable);
    void repetition_value_config(uint16_t repeat);
    // Set and get the auto reload value
    void set_auto_reload(uint16_t auto_reload);
    uint16_t get_auto_reload();

    void set_pulse_mode(Pulse_Mode pulse);
    // DMA
    void dma_enable(DMA_Select dma);
    void dma_disable(DMA_Select dma);
    void set_dma_enable(DMA_Select dma, bool enable);
    void set_dma_request_source(DMA_Request request);
    void dma_transfer_config(DMA_Transfer_Address address, DMA_Burst_Length length);
    // Break
    void break_init(TIMER_Break break_config = default_break);
    void break_enable();
    void break_disable();
    void set_break_enable(bool enable);
    // Output
    void output_auto_enable();
    void output_auto_disable();
    void set_output_auto_enable(bool enable);
    void set_primary_output_enable(bool enable);
    void set_channel_shadow_enable(bool enable);
    void channel_shadow_update_configure(Shadow_Update update);
    // Output compare
    void output_compare_init(Timer_Channel channel, TIMER_Output_Compare compare_config = default_compare);
    void set_output_mode(Timer_Channel channel, Output_Compare_Mode mode);
    void set_output_shadow(Timer_Channel channel, Output_Compare_Shadow shadow);
    void set_output_fast(Timer_Channel channel, Output_Compare_Fast fast);
    void set_output_clear(Timer_Channel channel, Output_Compare_Clear clear);
    void set_output_polarity(Timer_Channel channel, Output_Polarity polarity);
    void set_complement_output_polarity(Timer_Channel channel, Output_Polarity polarity);
    void set_channel_output_enable(Timer_Channel channel, bool enable);
    void set_compliment_output_enable(Timer_Channel channel, bool enable);
    // Input capture
    void input_capture_init(Timer_Channel channel, TIMER_Input_Capture capture_config = default_capture);
    void set_input_capture_prescaler(Timer_Channel channel, Input_Capture_Prescaler prescaler);
    void input_pwm_capture_enable(Timer_Channel channel);
    // Hall
    void set_hall_mode_enable(bool enable);
    // Trigger
    void set_input_trigger(Trigger_Select trigger);
    void external_trigger_configure(External_Trigger_Prescaler prescaler, Polarity_Select polarity, uint32_t filter);
    // Set and get input capture or output compare value
    void set_capture_compare(Timer_Channel channel, uint32_t cc_value);
    uint32_t get_capture_compare(Timer_Channel channel);
    // Mode
    void set_master_output_trigger(Master_Control mode);
    void set_slave(Slave_Control mode);
    void set_master_slave_enable(bool enable);
    // Quadrature decoder
    void quadrature_decoder_configure(Decode_Mode mode, Polarity_Select polarity1, Polarity_Select polarity2);
    // Clocks
    void set_internal_clock();
    void set_clock_from_internal_trigger(Trigger_Select trigger);
    void set_clock_from_external_trigger(Trigger_Select trigger, Polarity_Select polarity, uint32_t filter);
    void set_clock_mode0(External_Trigger_Prescaler prescaler, Polarity_Select polarity, uint32_t filter);
    void set_clock_mode1(External_Trigger_Prescaler prescaler, Polarity_Select polarity, uint32_t filter);
    void clock_mode1_disable();
    // Config
    void set_write_chxval_enable(bool enable);
    void set_output_value_enable(bool enable);
    // Flags
    bool get_flag(Status_Flags flag);
    void clear_flag(Status_Flags flag);
    // Interrupt flags
    bool get_interrupt_flag(Interrupt_Flags flag);
    void clear_interrupt_flag(Interrupt_Flags flag);
    void clear_all_interrupt_flags();
    // Interrupts
    void set_interrupt_enable(Interrupt_Type type, bool enable);
    void disable_all_interrupts();

    // Accessor methods
    TIMER_Base get_base() { return base_; }
    TIMER_Config& get_config() { return config_; }
    TIMER_Break& get_break_config() { return break_config_; }
    TIMER_Input_Capture& get_capture_config() { return capture_config_; }
    TIMER_Output_Compare& get_compare_config() { return compare_config_; }

    inline volatile uint32_t* reg_address(TIMER_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(base_address_ + static_cast<uint32_t>(reg));
    }

private:
    static std::array<bool, static_cast<size_t>(TIMER_Base::INVALID)> clock_enabled_;
    explicit TIMER(TIMER_Base Base);

    TIMER_Base base_;
    TIMER_Clock_Config TIMER_pclk_info_;
    uint32_t base_address_;
    TIMER_Config config_;
    TIMER_Break break_config_;
    TIMER_Input_Capture capture_config_;
    TIMER_Output_Compare compare_config_;

    inline bool is_advanced_timer() const {
        return (base_ == TIMER_Base::TIMER0_BASE || base_ == TIMER_Base::TIMER7_BASE);
    }
    inline bool is_general_purpose_timer() const {
        return (base_ == TIMER_Base::TIMER1_BASE || base_ == TIMER_Base::TIMER2_BASE
                || base_ == TIMER_Base::TIMER3_BASE || base_ == TIMER_Base::TIMER4_BASE);
    }
    inline bool is_basic_timer() const {
        return (base_ == TIMER_Base::TIMER5_BASE || base_ == TIMER_Base::TIMER6_BASE);
    }

    template <TIMER_Base Base>
    friend TIMER& get_instance_for_base();
};

} // namespace timer
