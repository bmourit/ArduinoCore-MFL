//
// MFL gd32f30x SDIO peripheral register access in C++
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

#include "sdio_config.hpp"
#include "RegRW.hpp"

namespace sdio {

class RCU;
class GPIO;

class SDIO {
public:
    static SDIO& get_instance();

    // Reset
    void reset();
    // Initialization
    void init(SDIO_Config config = default_config);
    // Interface clock configure
    void interface_clock_configure(Clock_Edge edge, bool bypass, bool low_power, uint16_t divider);
    // Get clock divider
    uint32_t get_clock_divider();
    // Hardware flow control
    void set_hardware_flow_control_enable(bool enable);
    // Bus width
    void set_bus_width(Bus_Width width);
    // Power
    uint32_t get_power_mode();
    void set_power_mode(Power_Control power);
    // Clock
    void set_clock_enable(bool enable);
    // Set block size
    void set_block_size(Block_Size size);
    // Command and start state machine
    void set_command_state_machine_and_send(Command_Index index, uint32_t argument, Command_Response response, Wait_Type type, bool enable);
    // Command
    void set_command_state_machine(Command_Index command, uint32_t argument, Command_Response response, Wait_Type type);
    // Command state machine
    void set_command_state_machine_enable(bool enable);
    // Index and last response
    uint8_t get_command_index();
    uint32_t get_response(Response_Type type);
    // Configure data and send
    void set_data_state_machine_and_send(uint32_t timeout, uint32_t length, Block_Size size, Transfer_Mode mode, Transfer_Direction direction, bool enable);
    // Configure
    void set_data_state_machine(uint32_t timeout, uint32_t length, Block_Size size);
    void set_data_transfer(Transfer_Mode mode, Transfer_Direction direction);
    // Data state machine
    void set_data_state_machine_enable(bool enable);
    // Data read/write
    uint32_t read_fifo_word();
    void write_fifo_word(uint32_t data);
    // Count
    uint32_t get_data_count();
    uint32_t get_fifo_count();
    // DMA
    void set_dma_enable(bool enable);
    // Flags
    bool get_flag(Status_Flags flag);
    void clear_flag(Clear_Flags flag);
    void clear_all_cmd_flags();
    void clear_all_data_flags();
    bool any_flag_set(const Status_Flags* flags, size_t count);
    // Read wait
    void set_read_wait_enable(bool enable);
    void set_stop_read_wait_enable(bool enable);
    void set_read_wait_type(Read_Wait type);
    // IO
    void set_IO_operation_enable(bool enable);
    void set_IO_suspend_enable(bool enable);
    // CE-ATA
    void set_CE_ATA_command_enable(bool enable);
    void set_CE_ATA_interrupt_enable(bool enable);
    void set_CE_ATA_command_completion_enable(bool enable);
    // Clear DSM
    void clear_data_state_machine(Transfer_Direction direction);
    // Check SCR flags
    bool check_scr_flags();
    // Wait for valid flag
    bool wait_cmd_flags();
    // Interrupt flags
    bool get_interrupt_flag(Interrupt_Flags int_flag);
    void clear_interrupt_flag(Clear_Flags int_flag);
    void clear_multiple_interrupt_flags(uint32_t bitmask);
    // Interrupts
    void set_interrupt_enable(Interrupt_Type type, bool enable);

    // Accessor methods
    SDIO_Config& get_config() { return config_;}

    inline uint16_t calculate_clock_divider(uint32_t requested_clock) {
        const uint32_t sdio_pclk = RCU_I.get_clock_frequency(rcu::Clock_Frequency::CK_AHB);
        requested_clock = custom_min(requested_clock, (sdio_pclk * 8U / 3U));
        requested_clock = custom_min(requested_clock, Default_Desired_Clock);
        return static_cast<uint16_t>(sdio_pclk / requested_clock + (sdio_pclk % requested_clock != 0U) - 2U);
    }

    static inline constexpr uintptr_t SDIO_baseAddress = 0x40018000U;

    inline volatile uint32_t* reg_address(SDIO_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(SDIO_baseAddress + static_cast<uint32_t>(reg));
    }

private:
    SDIO();

    // Prevent copying or assigning
    SDIO(const SDIO&) = delete;
    SDIO& operator=(const SDIO&) = delete;

    SDIO_Config config_;
    mutable bool is_clock_enabled_;
};


} // namespace sdio
