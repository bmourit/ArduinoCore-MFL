//
// MFL gd32f30x DAC peripheral register access in C++
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

#include "dac_config.hpp"
#include "RegRW.hpp"

namespace dac {

class RCU;

class DAC {
public:
    static DAC& get_instance();

    void reset();
    void enable(Internal_Device instance);
    void disable(Internal_Device instance);
    void set_enable(Internal_Device instance, bool enable);
    void set_dma_enable(Internal_Device instance, bool enable);
    void set_output_buffer_enable(Internal_Device instance, bool enable);
    uint16_t get_output_value(Internal_Device instance);
    void set_data(Internal_Device instance, Data_Align align, uint16_t data);
    void set_trigger_enable(Internal_Device instance, bool enable);
    void set_trigger_source(Internal_Device instance, Trigger_Source source);
    void set_software_trigger_enable(Internal_Device instance, bool enable);
    void set_wave_mode(Internal_Device instance, Wave_Type type);
    void set_wave_bit_width(Internal_Device instance, Bit_Width width);
    void set_lfsr_noise(Internal_Device instance, LFSR_Noise noise);
    void set_triangle_amplitude(Internal_Device instance, Triangle_Amplitude amplitude);
    void set_dual_mode_enable(bool enable);
    void set_dual_software_trigger_enable(bool enable);
    void set_dual_output_buffer_enable(bool enable);
    void set_dual_data(Data_Align align, uint16_t data0, uint16_t data1);

    static inline constexpr uintptr_t DAC_baseAddress = 0x40007400U;

    inline volatile uint32_t* reg_address(DAC_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(DAC_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    DAC();

    mutable bool is_clock_enabled_;
};

} // namespace dac
