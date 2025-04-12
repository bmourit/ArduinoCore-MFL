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

#include <stdint.h>
#include <array>

#include "i2c_config.hpp"
#include "ErrorTypes.hpp"
#include "RegRW.hpp"
#include "Utility.hpp"

namespace i2c {

class RCU;
class GPIO;

class I2C {
public:
    static Result<I2C, I2C_Error_Type> get_instance(I2C_Base Base);

    void reset();
    I2C_Error_Type set_clock_speed_duty(uint32_t speed, Duty_Cycle duty);
    void set_address_format(uint32_t address, Address_Format format, Bus_Mode mode);
    void set_smbus_type(Bus_Type type);
    void set_ack_enable(bool enable);
    void set_ack_position(ACK_Select position);
    void set_direction_address(Transfer_Direction direction, uint32_t address);
    void set_dual_address_enable(uint32_t address, bool enable);
    void set_enable(bool enable);
    void generate_start_condition();
    uint32_t get_start_condition();
    void generate_stop_condition();
    uint32_t get_stop_condition();
    void transmit_data(uint8_t data);
    uint8_t receive_data();
    void set_dma_enable(bool enable);
    void set_dma_transfer_end(bool is_end);
    void set_stretch_low(Stretch_Low stretch);
    void set_general_call_respond(bool respond);
    void set_software_reset_enable(bool reset);
    void set_pec_calculate(bool enable);
    void set_pec_transfer_enable(bool enable);
    uint8_t get_pec();
    void set_smbus_alert_enable(bool enable);
    void set_smbus_arp_enable(bool enable);
    // Flags
    bool get_flag(Status_Flags flag);
    void clear_flag(Clear_Flags flag);
    // Interrupt flags
    bool get_interrupt_flag(Interrupt_Flags flag);
    void clear_interrupt_flag(Clear_Flags flag);
    // Interrupt enable/disable
    void set_interrupt_enable(Interrupt_Type type, bool enable);

    // Accessor methods
    I2C_Base get_base() { return base_; }

    inline volatile uint32_t* reg_address(I2C_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(base_address_ + static_cast<uint32_t>(reg));
    }

private:
    static std::array<bool, static_cast<size_t>(I2C_Base::INVALID)> clock_enabled_;
    explicit I2C(I2C_Base Base);

    I2C_Base base_;
    I2C_Clock_Config I2C_pclk_info_;
    uint32_t base_address_;

    template <I2C_Base Base>
    friend I2C& get_instance_for_base();
};

} // namespace i2c
