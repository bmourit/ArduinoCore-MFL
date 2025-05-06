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

#include <cstdint>
#include <array>

#include "dma_config.hpp"
#include "ErrorTypes.hpp"
#include "RegRW.hpp"

namespace dma {

class DMA {
public:
    static auto get_instance(DMA_Base Base, DMA_Channel Channel) -> Result<DMA, DMA_Error_Type>;

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
    auto get_transfer_count() -> uint32_t;
    void set_transfer_count(uint32_t count);

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
    auto get_flag(Status_Flags flag) -> bool;
    void clear_flag(Status_Flags flag);
    void clear_flags(uint32_t flags);

    // Interrupt flags
    auto get_interrupt_flag(Interrupt_Flags flag) -> bool;
    void clear_interrupt_flag(Interrupt_Flags flag);

    // Interrupts
    void set_interrupt_enable(Interrupt_Type type, bool enable);

    // Accessor methods
    [[gnu::always_inline]] inline auto get_base() -> DMA_Base { return base_; }
    [[gnu::always_inline]] inline auto get_channel() -> DMA_Channel { return channel_; }
    [[gnu::always_inline]] inline auto get_config() -> DMA_Config { return config_; }

    // Register address
    [[nodiscard]] inline auto reg_address(DMA_Regs reg) const -> volatile uint32_t* {
        const auto idx = static_cast<uint32_t>(base_);
        return reinterpret_cast<volatile uint32_t*>(DMA_baseAddress[idx] + static_cast<uint32_t>(reg));
    }

private:
    static std::array<bool, static_cast<size_t>(DMA_Base::INVALID)> clock_enabled_;
    explicit DMA(DMA_Base Base, DMA_Channel Channel);

    DMA_Base base_;
    DMA_Channel channel_;
    DMA_Clock_Config DMA_pclk_info_;
    DMA_Config config_;

    struct CachedOffsets {
        DMA_Regs ctl;
        DMA_Regs cnt;
        DMA_Regs paddr;
        DMA_Regs maddr;
    } cached_offsets_;

    void cache_register_offsets();

    // Inlined methods
    [[gnu::always_inline]] inline auto get_channel_bits_from_flag(Status_Flags flag) -> INTF_Bits {
        constexpr std::array<std::array<INTF_Bits, 7>, 4> flag_map {{
            {{
                INTF_Bits::GIF0, INTF_Bits::GIF1, INTF_Bits::GIF2,
                INTF_Bits::GIF3, INTF_Bits::GIF4, INTF_Bits::GIF5,
                INTF_Bits::GIF6
            }},
            {{
                INTF_Bits::FTFIF0, INTF_Bits::FTFIF1, INTF_Bits::FTFIF2,
                INTF_Bits::FTFIF3, INTF_Bits::FTFIF4, INTF_Bits::FTFIF5,
                INTF_Bits::FTFIF6
            }},
            {{
                INTF_Bits::HTFIF0, INTF_Bits::HTFIF1, INTF_Bits::HTFIF2,
                INTF_Bits::HTFIF3, INTF_Bits::HTFIF4, INTF_Bits::HTFIF5,
                INTF_Bits::HTFIF6
            }},
            {{
                INTF_Bits::ERRIF0, INTF_Bits::ERRIF1, INTF_Bits::ERRIF2,
                INTF_Bits::ERRIF3, INTF_Bits::ERRIF4, INTF_Bits::ERRIF5,
                INTF_Bits::ERRIF6
            }}
        }};

        const auto flag_idx = static_cast<uint8_t>(flag);
        const auto channel_idx = static_cast<uint8_t>(channel_);

        if (flag_idx >= flag_map.size() || channel_idx >= flag_map[0].size()) {
            return INTF_Bits::INVALID;
        }

        return flag_map[flag_idx][channel_idx];
    }

    [[gnu::always_inline]] inline auto get_channel_offset_from_reg(Channel_Regs reg) -> DMA_Regs {
        // Early return for invalid inputs
        if (reg == Channel_Regs::INVALID) {
            return DMA_Regs::INVALID;
        }

        const auto channel_idx = static_cast<uint8_t>(channel_);
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
    friend auto get_instance_for_base() -> DMA&;
};

} // namespace dma
