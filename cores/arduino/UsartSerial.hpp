// gd32f30x UsartSerial driver for Arduino Core
// Copyright (c) 2025 B. Mourit <bnmguy@gmail.com>
// All rights reserved.

#pragma once

#include <array>

#include "Arduino.h"
#include "api/HardwareSerial.h"
#include "PinOpsMap.hpp"

namespace arduino {

#ifdef SERIAL_BUFFER_SIZE
    #undef SERIAL_BUFFER_SIZE
#endif
#ifdef USART_RX_BUFFER_SIZE
    #undef USART_RX_BUFFER_SIZE
#endif
#ifdef USART_TX_BUFFER_SIZE
    #undef USART_TX_BUFFER_SIZE
#endif

#define USART_COUNT 5

struct usart_to_irq {
    IRQn_Type irq_type : 7;
    uint8_t usart_index : 3;
};

inline constexpr uint8_t SERIAL_BUFFER_SIZE = 128;
inline constexpr uint8_t USART_RX_BUFFER_SIZE = SERIAL_BUFFER_SIZE;
inline constexpr uint8_t USART_TX_BUFFER_SIZE = 64;

class UsartSerial : public HardwareSerial {
public:
    static auto get_instance(usart::USART_Base Base, pin_size_t rxPin = NO_PIN, pin_size_t txPin = NO_PIN) -> UsartSerial&;

    void begin(unsigned long baudrate, uint16_t config, bool useDmaRx);
    inline void begin(unsigned long baudrate, uint16_t config) override { begin(baudrate, config, false); }
    inline void begin(unsigned long baudrate) override { begin(baudrate, SERIAL_8N1); }
    void end() override;
    int available() override;
    int availableForWrite();
    int peek() override;
    int read() override;
    void flush() override;
    size_t write(uint8_t byte) override;

    using Print::write; // Include additional write functions from Print

    operator bool() override { return true; }

    virtual void updateRxDmaBuffer();

    auto getLastReceivedData() -> uint8_t { return config_.last_data; }
    void setDmaMode(usart::USART_DMA_Config mode);

    void handleInterrupt();

    // Accessor methods
    auto get_config() -> usart::USART_Config& { return config_; }
    auto get_dma() -> dma::DMA& { return dma_; }
    auto get_dma_config() -> dma::DMA_Config& { return dmaConfig_; }

protected:
    static std::array<bool, static_cast<size_t>(usart::USART_Base::INVALID)> dataTransmitted_;
    explicit UsartSerial(usart::USART_Base Base, pin_size_t rxPin, pin_size_t txPin);

    usart::USART_Base base_;
    usart::USART& usart_;
    pin_size_t customRxPin_;
    pin_size_t customTxPin_;
    usart::USART_Config config_;
    dma::DMA_Base dmaBase_;
    dma::DMA_Channel dmaChannel_;
    dma::DMA& dma_;
    dma::DMA_Config dmaConfig_;

    static auto get_usart_instance(usart::USART_Base Base) -> usart::USART&;

    void configurePins();
    void checkDmaConfig();
    void setDmaRxEnable();

    inline auto get_parity_mode(uint16_t config) -> usart::Parity_Mode {
        uint16_t parity = config & SERIAL_PARITY_MASK;
        return (parity == SERIAL_PARITY_EVEN) ? usart::Parity_Mode::PM_EVEN :
               (parity == SERIAL_PARITY_ODD)  ? usart::Parity_Mode::PM_ODD :
               usart::Parity_Mode::PM_NONE;
    }

    inline auto get_stop_bits(uint16_t config) -> usart::Stop_Bits {
        uint16_t stop = config & SERIAL_STOP_BIT_MASK;
        return (stop == SERIAL_STOP_BIT_1_5) ? usart::Stop_Bits::STB_1_5BIT :
               (stop == SERIAL_STOP_BIT_2)   ? usart::Stop_Bits::STB_2BIT :
               usart::Stop_Bits::STB_1BIT;
    }

    inline auto get_word_length(uint16_t config) -> usart::Word_Length {
        uint16_t data = config & SERIAL_DATA_MASK;
        auto parity_mode = get_parity_mode(config);

        if (data == SERIAL_DATA_8 ) {
            return (parity_mode == usart::Parity_Mode::PM_NONE) ?
                   usart::Word_Length::WL_8BITS :
                   usart::Word_Length::WL_9BITS;
        }

        if (data == SERIAL_DATA_7 && parity_mode != usart::Parity_Mode::PM_NONE) {
            return usart::Word_Length::WL_8BITS;
        }

        return usart::Word_Length::WL_8BITS;
    }

    inline auto resolveRxPinOps() -> PinOps {
        if (customRxPin_ != NO_PIN) {
            return getPinOpsByPin(UART_RX_PinOps, customRxPin_);
        }
        return getPinOpsByPeripheral(UART_RX_PinOps, base_);
    }

    inline auto resolveTxPinOps() -> PinOps {
        if (customTxPin_ != NO_PIN) {
            return getPinOpsByPin(UART_TX_PinOps, customTxPin_);
        }
        return getPinOpsByPeripheral(UART_TX_PinOps, base_);
    }

    inline constexpr auto get_dma_base() -> dma::DMA_Base {
        return (base_ <= usart::USART_Base::USART2_BASE) ?
               dma::DMA_Base::DMA0_BASE :
               (base_ <= usart::USART_Base::UART4_BASE) ?
               dma::DMA_Base::DMA1_BASE :
               dma::DMA_Base::INVALID;
    }

    inline constexpr auto get_dma_channel() -> dma::DMA_Channel {
        constexpr std::array<dma::DMA_Channel, 5> channel_map = {
            dma::DMA_Channel::CHANNEL4,  // USART0
            dma::DMA_Channel::CHANNEL5,  // USART1
            dma::DMA_Channel::CHANNEL2,  // USART2
            dma::DMA_Channel::CHANNEL2,  // UART3
            dma::DMA_Channel::CHANNEL2   // UART4
        };
        const auto idx = static_cast<size_t>(base_);
        return (idx < channel_map.size()) ?
               channel_map[idx] :
               dma::DMA_Channel::INVALID;
    }

    inline static constexpr std::array<usart_to_irq, USART_COUNT> usart_irq {{
        {USART0_IRQn, 0U}, {USART1_IRQn, 1U}, {USART2_IRQn, 2U}, {UART3_IRQn, 3U}, {UART4_IRQn, 4U}
    }};

    inline auto usartToIrq(uint8_t usartIndex) -> IRQn_Type {
        for (const auto& index : usart_irq) {
            if (index.usart_index == usartIndex) {
                return index.irq_type;
            }
        }
        return INVALID_IRQ;
    }
};

} // namespace arduino

#if defined(DEFAULT_HARDWARE_SERIAL)
    #if DEFAULT_HARDWARE_SERIAL == 0
        #define USE_HARDWARE_SERIAL0
        extern arduino::UsartSerial& Serial0;
        extern void serialEvent0() __attribute__((weak));
        #if !defined(Serial)
            #define Serial Serial0
            #define serialEvent serialEvent0
        #endif
    #elif DEFAULT_HARDWARE_SERIAL == 1
        #define USE_HARDWARE_SERIAL1
        extern arduino::UsartSerial& Serial1;
        extern void serialEvent1() __attribute__((weak));
        #if !defined(Serial)
            #define Serial Serial1
            #define serialEvent serialEvent1
        #endif
    #elif DEFAULT_HARDWARE_SERIAL == 2
        #define USE_HARDWARE_SERIAL2
        extern arduino::UsartSerial& Serial2;
        extern void serialEvent2() __attribute__((weak));
        #if !defined(Serial)
            #define Serial Serial2
            #define serialEvent serialEvent2
        #endif
    #elif DEFAULT_HARDWARE_SERIAL == 3
        #define USE_HARDWARE_SERIAL3
        extern arduino::UsartSerial& Serial3;
        extern void serialEvent3() __attribute__((weak));
        #if !defined(Serial)
            #define Serial Serial3
            #define serialEvent serialEvent3
        #endif
    #elif DEFAULT_HARDWARE_SERIAL == 4
        #define USE_HARDWARE_SERIAL4
        extern arduino::UsartSerial& Serial4;
        extern void serialEvent4() __attribute__((weak));
        #if !defined(Serial)
            #define Serial Serial4
            #define serialEvent serialEvent4
        #endif
    #endif
#endif

#if !defined(Serial)
    #warning "No generic 'Serial' has been defined!"
#endif

#if (defined(USE_HARDWARE_SERIAL0) + defined(USE_HARDWARE_SERIAL1) + \
     defined(USE_HARDWARE_SERIAL2) + defined(USE_HARDWARE_SERIAL3) + \
     defined(USE_HARDWARE_SERIAL4)) > 1
#error "Multiple default hardware serials defined. Please select only one."
#endif
