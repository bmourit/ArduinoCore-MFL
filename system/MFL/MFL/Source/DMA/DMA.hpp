//
// MFL gd32f30x DMA peripheral register access in C++
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

#include "dma_config.hpp"
#include "ErrorTypes.hpp"
#include "RegRW.hpp"

namespace dma {

class RCU;

class DMA {
public:
    static Result<DMA, DMA_Error_Type> get_instance(DMA_Base Base, DMA_Channel Channel);

    // Init
    void init(DMA_Config config = default_config);
    // Reset
    void reset();
    // Circulation mode
    void set_circulation_mode_enable(bool enable);
    // M2M
    void set_memory_to_memory_enable(bool enable);
    // Channel
    void set_channel_enable(bool enable);
    // RX/TX addresses
    void set_data_address(Data_Type type, uint32_t address);
    // Get and set Transfer count
    void set_transfer_count(uint32_t count);
    uint32_t get_transfer_count();
    // Priority
    void set_channel_priority(Channel_Priority priority);
    // Bit width
    void set_bit_width(Data_Type type, Bit_Width width);
    // Increase mode
    void set_increase_mode_enable(Data_Type type, bool enable);
    // Direction
    void set_transfer_direction(Transfer_Direction direction);
    // Abandon transfer
    void set_transfer_abandon();
    // Clear channel
    void clear_channel();
    // Flags
    bool get_flag(Status_Flags flag);
    void clear_flag(Status_Flags flag);
    void clear_flags(uint32_t flags);
    // Interrupt flags
    bool get_interrupt_flag(Interrupt_Flags flag);
    void clear_interrupt_flag(Interrupt_Flags flag);
    // Interrupts
    void set_interrupt_enable(Interrupt_Type type, bool enable);

    // Accessor methods
    inline DMA_Base get_base() { return base_; }
    inline DMA_Channel get_channel() { return channel_; }
    inline DMA_Config get_config() { return config_; }

    // Register address
    inline volatile uint32_t* reg_address(DMA_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(base_address_ + static_cast<uint32_t>(reg));
    }

private:
    static std::array<bool, static_cast<size_t>(DMA_Base::INVALID)> clock_enabled_;
    explicit DMA(DMA_Base Base, DMA_Channel Channel);

    DMA_Base base_;
    DMA_Channel channel_;
    DMA_Clock_Config DMA_pclk_info_;
    uint32_t base_address_;
    DMA_Config config_;

    struct CachedOffsets {
        DMA_Regs ctl;
        DMA_Regs cnt;
        DMA_Regs paddr;
        DMA_Regs maddr;
    } cached_offsets_;

    void cache_register_offsets();

    // Inlined methods
    inline INTF_Bits get_channel_bits_from_flag(Status_Flags flag) {
        // Get indices with bounds checking
        const uint8_t flag_idx = static_cast<uint8_t>(flag);
        const uint8_t channel_idx = static_cast<uint8_t>(channel_);
    
        // Early return for invalid inputs
        if (flag_idx >= 4 || channel_idx >= 7) {    // 4 flag types, 7 channels (0-6)
            return INTF_Bits::INVALID;
        }

        // Use a 2D table with direct indexing for better performance
        // First dimension: flag index, Second dimension: channel number
        static constexpr INTF_Bits flag_map[4][7] = {
            {
                INTF_Bits::GIF0, INTF_Bits::GIF1, INTF_Bits::GIF2,
                INTF_Bits::GIF3, INTF_Bits::GIF4, INTF_Bits::GIF5,
                INTF_Bits::GIF6
            },
            {
                INTF_Bits::FTFIF0, INTF_Bits::FTFIF1, INTF_Bits::FTFIF2,
                INTF_Bits::FTFIF3, INTF_Bits::FTFIF4, INTF_Bits::FTFIF5,
                INTF_Bits::FTFIF6
            },
            {
                INTF_Bits::HTFIF0, INTF_Bits::HTFIF1, INTF_Bits::HTFIF2,
                INTF_Bits::HTFIF3, INTF_Bits::HTFIF4, INTF_Bits::HTFIF5,
                INTF_Bits::HTFIF6
            },
            {
                INTF_Bits::ERRIF0, INTF_Bits::ERRIF1, INTF_Bits::ERRIF2,
                INTF_Bits::ERRIF3, INTF_Bits::ERRIF4, INTF_Bits::ERRIF5,
                INTF_Bits::ERRIF6
            }
        };

        // Direct array access with pre-validated indices
        return flag_map[flag_idx][channel_idx];
    }

    inline DMA_Regs get_channel_offset_from_reg(Channel_Regs reg) {
        // Early return for invalid inputs
        if (reg == Channel_Regs::INVALID) {
            return DMA_Regs::INVALID;
        }

        const uint8_t channel_idx = static_cast<uint8_t>(channel_);
        if (channel_idx > 6) {  // 7 channels (0-6)
            return DMA_Regs::INVALID;
        }

        uint8_t base_offset;
        switch (reg) {
            case Channel_Regs::CHXCTL: base_offset = 0x08U; break;
            case Channel_Regs::CHXCNT: base_offset = 0x0CU; break;
            case Channel_Regs::CHXPADDR: base_offset = 0x10U; break;
            case Channel_Regs::CHXMADDR: base_offset = 0x14U; break;
            case Channel_Regs::INVALID:
            default: return DMA_Regs::INVALID;
        }

        // Each channel's registers are offset by 0x14 bytes from the previous channel
        const uint8_t channel_offset = channel_idx * 0x14U;
        // Calculate final register address
        const uint8_t reg_address = base_offset + channel_offset;

        // Convert to DMA_Regs enum value
        return static_cast<DMA_Regs>(reg_address);
    }

    template <DMA_Base Base, DMA_Channel Channel>
    friend DMA& get_instance_for_base();
};


} // namespace dma
