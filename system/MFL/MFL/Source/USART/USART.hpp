//
// MFL gd32f30x USART peripheral register access in C++
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

#include "usart_config.hpp"
#include "ErrorTypes.hpp"
#include "RegRW.hpp"
#include "RingBuffer.hpp"

namespace usart {

class RCU;
class GPIO;

class USART {
public:
    static Result<USART, USART_Error_Type> get_instance(USART_Base Base);

    void init(USART_Config config = default_config);
    void reset();
    void release();

    // Enable or disable
    void set_enable(bool enable);
    void enable();
    void disable();

    // Configurable parameters
    void set_baudrate(uint32_t baudrate);
    void set_parity(Parity_Mode parity);
    void set_word_length(Word_Length word_length);
    void set_stop_bits(Stop_Bits stop_bits);
    void set_direction(Direction_Mode direction);
    void set_msb(MSBF_Mode msbf);
    void set_inversion_method_enable(Inversion_Method method, bool enable);
    void set_rx_timeout_enable(bool enable);
    void set_rx_timeout_threshold(uint32_t timeout);

    // Send and receive 16bit data
    void send_data(uint16_t data);
    uint16_t receive_data16();

    // Send and receive 8bit data
    void send_data(uint8_t data);
    uint8_t receive_data8();

    // DMA send or receive
    void receive_data_dma_enable(bool enable);
    void send_data_dma_enable(bool enable);

    // Modes
    void set_wakeup_address(uint8_t address);
    void mute_mode_enable(bool enable);
    void set_mute_mode_wakeup(Wakeup_Mode method);
    void set_half_duplex_enable(bool enable);
    void set_synchronous_clock_enable(bool enable);
    void synchronous_clock_configure(Pulse_Length length, Clock_Phase phase, Clock_Polarity polarity);

    // LIN mode
    void set_lin_mode_enable(bool enable);
    void set_lin_frame_break_length(Break_Length length);
    void send_lin_frame_break();
    void set_guard_time(uint8_t guard_time);

    // Smartcard mode
    void set_smardcard_mode_enable(bool enable);
    void set_smartcard_nack_mode_enable(bool enable);
    void set_smartcard_auto_retry(uint8_t retry_count);
    void set_smartcard_block_size(uint8_t size);

    // IrDA mode
    void set_irda_mode_enable(bool enable);
    void set_irda_low_power_prescaler(uint8_t prescaler);
    void set_irda_power_mode(IrDA_Power power);

    // HWFC
    void set_hwfc_rts_enable(bool enable);
    void set_hwfc_cts_enable(bool enable);

    // Interrupts and flags
    bool get_flag(Status_Flags flag);
    void clear_flag(Status_Flags flag);
    bool get_interrupt_flag(Interrupt_Flags flag);
    void clear_interrupt_flag(Interrupt_Flags flag);
    void set_interrupt_enable(Interrupt_Type type, bool enable);
    void set_interrupt_priority(uint8_t prepriority, uint8_t subpriority);
    void prepare_receive_interrupts();

    // Callbacks
    using IRQCallback = void (*)(void);
    void register_interrupt_callback(Interrupt_Type type, IRQCallback callback);
    void unregister_interrupt_callback(Interrupt_Type type);

    // Error handler
    bool check_error_flags();
    void handle_errors();

    // Interrupt handlers
    void handle_rx_interrupt();
    void handle_tx_interrupt();
    void handle_transfer_complete_interrupt();
    void handle_interrupt();

    // Non-blocking
    bool usart_receive_interrupt(uint8_t& data);
    bool usart_transmit_interrupt(uint8_t data);

    // Blocking
    bool usart_receive_polling(uint8_t& data);
    bool usart_transmit_polling(uint8_t data);

    // DMA
    bool usart_receive_dma(uint8_t& data);
    bool usart_transmit_dma(uint8_t data);

    // DMA with interrupts
    bool usart_receive_dma_interrupt(uint8_t& data);
    bool usart_transmit_dma_interrupt(uint8_t data);

    // Accessor methods
    inline USART_Base get_base() { return base_; }
    inline USART_Config& get_config() { return config_; }
    inline RingBuffer<uint8_t, RxBufferSize>& get_rx_buffer() { return rx_buffer_; }
    inline RingBuffer<uint8_t, TxBufferSize>& get_tx_buffer() { return tx_buffer_; }
    inline uint8_t get_last_data() { return config_.last_data; }

    // Accessor wrapper methods for buffer
    inline bool read_rx_buffer(uint8_t& data) {
        if (rx_buffer_.isEmpty()) return false;
        return rx_buffer_.read(data);
    }
    inline bool write_tx_buffer(uint8_t data) {
        if (tx_buffer_.isFull()) return false;
        return tx_buffer_.write(data);
    }
    inline void modify_rx_buffer_head(uint8_t head) { rx_buffer_.setHead(head); }
    inline void modify_tx_buffer_head(uint8_t head) { tx_buffer_.setHead(head); }
    inline void modify_rx_buffer_tail(uint8_t tail) { rx_buffer_.setTail(tail); }
    inline void modify_tx_buffer_tail(uint8_t tail) { tx_buffer_.setTail(tail); }
    inline void flush_buffer(bool is_rx) { is_rx ? rx_buffer_.flush() : tx_buffer_.flush(); }
    inline size_t available_for_read(bool is_rx) { return is_rx ? rx_buffer_.availableForRead() : tx_buffer_.availableForRead(); }
    inline size_t available_for_write(bool is_rx) { return is_rx ? rx_buffer_.availableForWrite() : tx_buffer_.availableForWrite(); }
    inline bool peek_buffer(bool is_rx, uint8_t& data) { return is_rx ? rx_buffer_.peek(data) : tx_buffer_.peek(data); }
    inline uint8_t* get_buffer_data(bool is_rx) { return is_rx ? rx_buffer_.data() : tx_buffer_.data(); }
    inline bool buffer_is_empty(bool is_rx) { return is_rx ? rx_buffer_.isEmpty() : tx_buffer_.isEmpty(); }

    // Calculate register and offset
    inline volatile uint32_t* reg_address(USART_Regs reg) const {
        return reinterpret_cast<volatile uint32_t*>(base_address_ + static_cast<uint32_t>(reg));
    }

private:
    static std::array<bool, static_cast<size_t>(USART_Base::INVALID)> clock_enabled_;
    explicit USART(USART_Base Base);

    // Private member variables
    USART_Base base_;
    USART_Clock_Config USART_pclk_info_;
    uint32_t base_address_;
    USART_Config config_;
    RingBuffer<uint8_t, RxBufferSize> rx_buffer_;
    RingBuffer<uint8_t, TxBufferSize> tx_buffer_;
    IRQCallback interrupt_callbacks_[static_cast<size_t>(Interrupt_Type::INTR_COUNT)];

    template <USART_Base Base>
    friend USART& get_instance_for_base();
};

} // namespace usart
