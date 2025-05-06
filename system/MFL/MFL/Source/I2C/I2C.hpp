//
// MFL gd32f30x I2C peripheral register access in C++
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

#include <cstdint>
#include <array>

#include "i2c_config.hpp"
#include "ErrorTypes.hpp"
#include "RegRW.hpp"
#include "Utility.hpp"

namespace i2c {

class I2C {
public:
    static auto get_instance(I2C_Base Base) -> Result<I2C, I2C_Error_Type>;

    // Reset
    void reset();

    // Enable
    void set_enable(bool enable);

    // Clock
    auto set_clock_speed_duty(uint32_t speed, Duty_Cycle duty) -> I2C_Error_Type;

    // ACK
    void set_ack_enable(bool enable);
    void set_ack_position(ACK_Select position);

    // Address
    void set_address_format(uint32_t address, Address_Format format, Bus_Mode mode);
    void set_direction_address(Transfer_Direction direction, uint32_t address);
    void set_dual_address_enable(uint32_t address, bool enable);

    // Start/stop condition
    void generate_start_condition();
    auto get_start_condition() -> uint32_t;
    void generate_stop_condition();
    auto get_stop_condition() -> uint32_t;

    // Transmition/reception
    void transmit_data(uint8_t data);
    auto receive_data() -> uint8_t;

    // DMA
    void set_dma_enable(bool enable);
    void set_dma_transfer_end(bool is_end);

    // Stretch
    void set_stretch_low(Stretch_Low stretch);

    // General call
    void set_general_call_respond(bool respond);

    // Software reset
    void set_software_reset_enable(bool reset);

    // PEC
    void set_pec_calculate(bool enable);
    void set_pec_transfer_enable(bool enable);
    auto get_pec() -> uint8_t;

    // SMBUS
    void set_smbus_type(Bus_Type type);
    void set_smbus_alert_enable(bool enable);
    void set_smbus_arp_enable(bool enable);

    // Flags
    auto get_flag(Status_Flags flag) -> bool;
    void clear_flag(Clear_Flags flag);

    // Interrupt flags
    auto get_interrupt_flag(Interrupt_Flags flag) -> bool;
    void clear_interrupt_flag(Clear_Flags flag);

    // Interrupts
    void set_interrupt_enable(Interrupt_Type type, bool enable);

    // Accessor methods
    [[gnu::always_inline]] inline auto get_base() -> I2C_Base { return base_; }

    [[nodiscard]] inline auto reg_address(I2C_Regs reg) const -> volatile uint32_t* {
        const auto idx = static_cast<uint32_t>(base_);
        return reinterpret_cast<volatile uint32_t*>(I2C_baseAddress[idx] + static_cast<uint32_t>(reg));
    }

private:
    static std::array<bool, static_cast<size_t>(I2C_Base::INVALID)> clock_enabled_;
    explicit I2C(I2C_Base Base);

    I2C_Base base_;
    I2C_Clock_Config I2C_pclk_info_;

    template <I2C_Base Base>
    friend auto get_instance_for_base() -> I2C&;
};

} // namespace i2c
