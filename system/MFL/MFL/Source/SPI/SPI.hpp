//
// MFL gd32f30x SPI peripheral register access in C++
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

#include "spi_config.hpp"
#include "ErrorTypes.hpp"
#include "RegRW.hpp"

namespace spi {

class RCU;

class SPI {
public:
    static Result<SPI, SPI_Error_Type> get_instance(SPI_Base Base);

    // Initialize
    void init(SPI_Config config = default_config);
    // Reset
    void reset();
    // Enable or disable
    void set_enable(bool enable);
    // NSS
    void set_nss_output_enable(bool enabled);
    void nss_internal_high();
    void nss_internal_low();
    // DMA
    void set_dma_enable(DMA_Direction dma, bool enabled);
    // Configuration
    void data_frame_format_config(Frame_Format frame_format);
    void bidirectional_transfer_config(Direction_Mode transfer_direction);
    // Data
    void data_transmit(uint16_t data);
    uint16_t data_receive();
    // CRC
    void set_crc_enable(bool enabled);
    void set_crc_next();
    uint16_t get_crc(CRC_Direction crc);
    void clear_crc_error();
    void set_crc_polynomial(uint16_t crc_poly);
    uint16_t get_crc_polynomial();
    // NSSP
    void set_nssp_mode_enable(bool enabled);
    // QUAD
    void set_quad_mode_enable(bool enabled);
    void quad_write_enable();
    void quad_read_enable();
    void set_quad_io23_output_enable(bool enabled);
    // Interrupts and flags
    bool get_flag(Status_Flags flag);
    bool get_interrupt_flag(Interrupt_Flags flag);
    void set_interrupt_enable(Interrupt_Type type, bool enabled);

    // Accessor methods
    SPI_Base get_base() { return base_; }
    SPI_Config& get_config() { return config_; }

    inline volatile uint32_t* reg_address(SPI_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(base_address_ + static_cast<uint32_t>(reg));
    }

private:
    static std::array<bool, static_cast<size_t>(SPI_Base::INVALID)> clock_enabled_;
    explicit SPI(SPI_Base Base);

    SPI_Base base_;
    SPI_Clock_Config SPI_pclk_info_;
    uint32_t base_address_;
    SPI_Config config_;

    template <SPI_Base Base>
    friend SPI& get_instance_for_base();
};

} // namespace spi
