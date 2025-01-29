// gd32f30x UsartSerial driver for Arduino Core
// Copyright (c) 2025 B. Mourit <bnmguy@gmail.com>
// All rights reserved.

#include "Arduino.h"
#include "UsartSerial.hpp"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"

namespace arduino {

inline constexpr uint32_t defaultBaudrate = 115200U;
inline constexpr uint8_t interruptPreemptPriority = 1U;
inline constexpr uint8_t interruptSubPriority = 0U;
inline constexpr uint8_t maxNumUsart_ = USART_COUNT;

usart::USART& UsartSerial::get_usart_instance(usart::USART_Base Base) {
    static usart::USART* usart_instances[maxNumUsart_] = {nullptr};
    size_t index = static_cast<size_t>(Base);

    if (usart_instances[index] == nullptr) {
        auto result = usart::USART::get_instance(Base);
        if (result.error() != usart::USART_Error_Type::OK) {
#ifdef CORE_DEBUG
            core_debug("ERROR: Invalid usart instance!");
#endif
            __builtin_trap();
        }
        usart_instances[index] = &result.value();
    }
    return *usart_instances[index];
}

UsartSerial& UsartSerial::get_instance(usart::USART_Base Base, pin_size_t rxPin, pin_size_t txPin) {
    switch (Base) {
        case usart::USART_Base::USART0_BASE: {
            static UsartSerial USerial0(Base, rxPin, txPin);
            return USerial0;
        }
        case usart::USART_Base::USART1_BASE: {
            static UsartSerial USerial1(Base, rxPin, txPin);
            return USerial1;
        }
        case usart::USART_Base::USART2_BASE: {
            static UsartSerial USerial2(Base, rxPin, txPin);
            return USerial2;
        }
        case usart::USART_Base::UART3_BASE: {
            static UsartSerial USerial3(Base, rxPin, txPin);
            return USerial3;
        }
        case usart::USART_Base::UART4_BASE: {
            static UsartSerial USerial4(Base, rxPin, txPin);
            return USerial4;
        }
        case usart::USART_Base::INVALID:
        default:
#ifdef CORE_DEBUG
            core_debug("Invalid USART instance!");
            __builtin_trap();
#endif
            static UsartSerial dummy(Base, rxPin, txPin);
            return dummy;
    }
}

std::array<bool, static_cast<size_t>(usart::USART_Base::INVALID)> UsartSerial::dataTransmitted_ = {false};

std::array<usart_to_irq, maxNumUsart_> UsartSerial::usart_irq {{
    {0U, USART0_IRQn}, {1U, USART1_IRQn}, {2U, USART2_IRQn}, {3U, UART3_IRQn}, {4U, UART4_IRQn}
}};

UsartSerial::UsartSerial(usart::USART_Base Base, pin_size_t rxPin, pin_size_t txPin) :
    base_(Base),
    usart_(get_usart_instance(Base)),
    customRxPin_(rxPin),
    customTxPin_(txPin),
    config_(usart::default_config),
    dmaBase_(get_dma_base()),
    dmaChannel_(get_dma_channel()),
    dma_(dma::DMA::get_instance(dmaBase_, dmaChannel_).value()),
    dmaConfig_(dma::default_config) {}

/**
 * Initializes the USART instance with the specified baudrate and configuration.
 * Also sets RX pin as input and TX pin as output.
 *
 * @param baudrate The baudrate to use for this instance.
 * @param config The configuration to use for this instance.
 */
void UsartSerial::begin(unsigned long baudrate, uint16_t config) {
    if (!(dataTransmitted_[static_cast<size_t>(base_)])) {
        end();
    }
    // Configure pins
    configurePins();

    usart::USART_DMA_Config dmaMode = usart::USART_DMA_Config::DMA_NONE;
#ifdef SERIAL_USE_DMA_RX
    dmaMode = usart::USART_DMA_Config::DMA_RX;
#endif

    usart_.init({
        static_cast<uint32_t>(baudrate),
        dmaMode,
        get_parity_mode(config),
        get_word_length(config),
        get_stop_bits(config),
        usart::Direction_Mode::RXTX_MODE,
        usart::USART_State::IDLE,
        interruptPreemptPriority,
        interruptSubPriority,
        0U
    });

    // HWFC not yet_supported
    usart_.set_hwfc_rts_enable(false);
    usart_.set_hwfc_cts_enable(false);

    // Check DMA config is valid
    checkDmaConfig();

    if (dmaMode == usart::USART_DMA_Config::DMA_NONE) {
        // Dont prepare receive interrupts if DMA is used
        usart_.prepare_receive_interrupts();
    }

    IRQn_Type irq = usartToIrq(static_cast<size_t>(base_));
    NVIC_DisableIRQ(irq);
    NVIC_SetPriority(irq, interruptPreemptPriority);
    NVIC_EnableIRQ(irq);
}

/**
 * @brief Disable the USART and flush the transmit buffer.
 *
 * This function is used to disable the USART and flush the transmit buffer.
 * It is usually called before the end of the program to ensure that the USART
 * is disabled and that the interrupts are cleared.
 */
void UsartSerial::end() {
    while (usart_.get_config().state != usart::USART_State::IDLE) {
        // Wait for the USART to become idle
    }
    flush();
    dataTransmitted_[static_cast<size_t>(base_)] = false;
    if (config_.dma_ops == usart::USART_DMA_Config::DMA_NONE) {
        usart_.set_interrupt_enable(usart::Interrupt_Type::INTR_PERRIE, false);
        usart_.set_interrupt_enable(usart::Interrupt_Type::INTR_ERRIE, false);
        usart_.set_interrupt_enable(usart::Interrupt_Type::INTR_RBNEIE, false);
    }
    usart_.clear_interrupt_flag(usart::Interrupt_Flags::INTR_FLAG_CTL0_TC);
    usart_.set_interrupt_enable(usart::Interrupt_Type::INTR_TBEIE, false);
    usart_.flush_buffer(false);
    usart_.release();
    usart_.reset();
}

/**
 * @brief Checks how many bytes are available in the USART serial buffer.
 *
 * Waits for the USART to become idle, updates the RX DMA buffer, and checks how
 * many bytes are available to read in the serial buffer.
 *
 * @return The number of bytes available in the serial buffer.
 */
int UsartSerial::available() {
    if (usart_.get_config().dma_ops == usart::USART_DMA_Config::DMA_NONE) {
        if (usart_.get_config().state == usart::USART_State::ERROR) {
            return 0;
        }
    }
    while (usart_.get_config().state != usart::USART_State::IDLE) {
        // Wait for the USART to become idle
    }
    updateRxDmaBuffer();
    return usart_.available_for_read(true);
}

/**
 * @brief Peeks at the next byte in the USART serial buffer without removing it.
 *
 * Waits for the USART to become idle, updates the RX DMA buffer, and retrieves
 * the next byte from the buffer if available.
 *
 * @return The next byte in the buffer, or -1 if no byte is available.
 */
int UsartSerial::peek() {
    if (usart_.get_config().dma_ops == usart::USART_DMA_Config::DMA_NONE) {
        if (usart_.get_config().state == usart::USART_State::ERROR) {
            return -1;
        }
        while (usart_.get_config().state != usart::USART_State::IDLE) {
        // Wait for the USART to become idle
        }
    }
    updateRxDmaBuffer();

    uint8_t data = 0;
    if (!usart_.peek_buffer(true, data)) {
        return -1;
    }
    return data;
}

/**
 * @brief Reads a byte from the USART serial buffer.
 *
 * Waits for the USART to become idle, updates the RX DMA buffer, and retrieves a byte.
 *
 * @return The read byte, or -1 if the read operation fails.
 */
int UsartSerial::read() {
    if (usart_.get_config().dma_ops == usart::USART_DMA_Config::DMA_NONE) {
        if (usart_.get_config().state == usart::USART_State::ERROR) {
            return -1;
        }
        while (usart_.get_config().state != usart::USART_State::IDLE) {
            // Wait for the USART to become idle
        }
    }
    updateRxDmaBuffer();

    uint8_t data;
    if (!usart_.read_rx_buffer(data)) {
        return -1;
    }
    return data;
}

/**
 * @brief Flushes the USART TX buffer.
 *
 * Waits until the USART is idle and then waits until all bytes in the TX buffer
 * have been sent.
 *
 * @details This function is used to ensure that any previously written data is
 * sent before the serial connection is closed.
 */
void UsartSerial::flush() {
    while (usart_.get_config().state != usart::USART_State::IDLE) {
        // Wait for the USART to become idle
    }
    // Wait for any outstanding data to be sent
    while (!usart_.buffer_is_empty(false));
}

/**
 * @brief Writes a single byte to the USART transmitter.
 *
 * Waits for the USART to become idle, marks that data has been transmitted, and
 * writes the byte to the TX buffer. If the buffer is full, waits until space is
 * available and then triggers transmission.
 *
 * @param[in] byte The byte to transmit.
 *
 * @return The number of bytes written (always 1).
 */
size_t UsartSerial::write(uint8_t byte) {
    dataTransmitted_[static_cast<size_t>(base_)] = true;
    while (usart_.get_config().state != usart::USART_State::IDLE) {
        // Wait for the USART to become idle
    }
    while (!usart_.usart_transmit_interrupt(byte)) {
        // Wait for room in buffer
    }
    return 1;
}

/**
 * @brief Updates the RX DMA buffer.
 *
 * Waits until the USART is idle and updates the DMA RX buffer head by subtracting
 * the current transfer count from the total RX buffer size. This is used to
 * synchronize the DMA transfer count with the actual RX buffer size.
 * 
 * NOTE:
 *  This function does only adjustes to buffer in DMA mode and
 *  is skipped in IRQ context.
 */
void UsartSerial::updateRxDmaBuffer() {
    if (usart_.get_config().dma_ops == usart::USART_DMA_Config::DMA_NONE) {
        return;
    }
    usart_.modify_rx_buffer_head(USART_RX_BUFFER_SIZE - dma_.get_transfer_count());
}

/**
 * @brief Sets the DMA mode of the USART.
 *
 * If the DMA mode is being changed, updates the DMA configuration of the USART.
 * Otherwise, does nothing.
 *
 * @param[in] mode The DMA mode to set.
 */
void UsartSerial::setDmaMode(usart::USART_DMA_Config mode) {
    if (usart_.get_config().dma_ops == mode) return;
    usart_.get_config().dma_ops = mode;
    checkDmaConfig();
}

/**
 * @brief Handles the USART interrupt.
 *
 * Delegates the interrupt handling to the underlying USART
 * instance's interrupt handler.
 */
void UsartSerial::handleInterrupt() {
    usart_.handle_interrupt();
}

/**
 * @brief Configures the RX and TX pins for USART serial communication.
 *
 * Initializes the RX and TX pins using the fast path, sets their modes and speeds,
 * applies custom pin assignments or overrides if specified, and handles pin remapping.
 */
void UsartSerial::configurePins() {
    // RX pin
    auto rxPinOps = getPinOpsByPeripheral(UART_RX_PinOps, base_);
#ifdef SERIAL_PIN_RX
        // RX pin override defined in Variant.h
        rxPinOps = getPinOpsByPin(UART_RX_PinOps, SERIAL_PIN_RX);
#endif
    if (customRxPin_ != NO_PIN) {
        rxPinOps = getPinOpsByPin(UART_RX_PinOps, customRxPin_);
    }
    if (rxPinOps == invalidPinOps) {
        return;
    }
    gpio::Pin_Mode mode = getPackedPinMode(rxPinOps.packedPinOps);
    gpio::Output_Speed speed = getPackedPinSpeed(rxPinOps.packedPinOps);
    // Initialize RX pin using fast path
    auto rxResult = gpio::GPIO::get_instance(rxPinOps.port);
    auto& gpioRxPort = rxResult.value();
    gpioRxPort.set_pin_mode(rxPinOps.pin, mode, speed);
    // Check remap
    gpio::Pin_Remap_Select rxRemap = getPackedPinRemap(rxPinOps.packedPinOps);
    if (rxRemap != gpio::Pin_Remap_Select::NO_REMAP) {
        AFIO_I.set_remap(rxRemap);
    }

    // TX pin
    auto txPinOps = getPinOpsByPeripheral(UART_TX_PinOps, base_);
#ifdef SERIAL_PIN_TX
    // TX pin override defined in Variant.h
    txPinOps = getPinOpsByPin(UART_TX_PinOps, SERIAL_PIN_TX);
#endif
    if (customTxPin_ != NO_PIN) {
        txPinOps = getPinOpsByPin(UART_TX_PinOps, customTxPin_);
    }
    if (txPinOps == invalidPinOps) {
        return;
    }
    mode = getPackedPinMode(txPinOps.packedPinOps);
    speed = getPackedPinSpeed(txPinOps.packedPinOps);
    // Initialize RX pin using fast path
    auto txResult = gpio::GPIO::get_instance(txPinOps.port);
    auto& gpioTxPort = txResult.value();
    gpioTxPort.set_pin_mode(txPinOps.pin, mode, speed);
    // Check remap
    gpio::Pin_Remap_Select txRemap = getPackedPinRemap(txPinOps.packedPinOps);
    if (txRemap != gpio::Pin_Remap_Select::NO_REMAP) {
        AFIO_I.set_remap(txRemap);
    }
}

/**
 * @brief Validates and configures the DMA settings for the USART.
 *
 * Checks the current DMA mode set in the configuration, and if DMA is enabled 
 * for RX, it calls the function to enable DMA reception. Logs a debug message 
 * if unsupported DMA modes are detected. If DMA is not enabled, the function 
 * exits without making changes.
 */
void UsartSerial::checkDmaConfig() {
    if (usart_.get_config().dma_ops == usart::USART_DMA_Config::DMA_NONE) {
        return;
    }

    if (usart_.get_config().dma_ops == usart::USART_DMA_Config::DMA_TX ||
        usart_.get_config().dma_ops == usart::USART_DMA_Config::DMA_DUAL) {
        core_debug("DMA_TX and DMA_DUAL modes are not supported");
        return;
    }

    if (usart_.get_config().dma_ops == usart::USART_DMA_Config::DMA_RX) {
        setDmaRxEnable();
    }
}

/**
 * @brief Enables DMA reception for the USART.
 *
 * If DMA_RX is enabled in the configuration, this function initializes the DMA
 * channel and enables DMA reception for the USART. The DMA configuration is
 * stored in the class member variable `dmaConfig_`.
 *
 * @note This function is called by `checkDmaConfig()` and should not be called
 * directly by the user.
 */
void UsartSerial::setDmaRxEnable() {
    if (usart_.get_config().dma_ops != usart::USART_DMA_Config::DMA_RX) {
        return; // Unsupported
    }

    dma_.clear_channel();

    dma_.init({
        usart::RxBufferSize,
        reinterpret_cast<uint32_t>(usart_.get_buffer_data(true)),
        reinterpret_cast<uint32_t>(usart_.reg_address(usart::USART_Regs::DATA)),
        dma::Bit_Width::WIDTH_8BIT,
        dma::Bit_Width::WIDTH_8BIT,
        dma::Increase_Mode::INCREASE_DISABLE,
        dma::Increase_Mode::INCREASE_ENABLE,
        dma::Channel_Priority::MEDIUM_PRIORITY,
        dma::Transfer_Direction::P2M,
    });

    dma_.set_memory_to_memory_enable(false);
    dma_.set_circulation_mode_enable(true);
    usart_.receive_data_dma_enable(true);
    dma_.set_channel_enable(true);
}

#ifdef USE_HARDWARE_SERIAL0
    arduino::UsartSerial& Serial0 = arduino::UsartSerial::get_instance(usart::USART_Base::USART0_BASE);
    void serialEvent0() __attribute__((weak));
#endif

#ifdef USE_HARDWARE_SERIAL1
    arduino::UsartSerial& Serial1 = arduino::UsartSerial::get_instance(usart::USART_Base::USART1_BASE);
    void serialEvent1() __attribute__((weak));
#endif

#ifdef USE_HARDWARE_SERIAL2
    arduino::UsartSerial& Serial2 = arduino::UsartSerial::get_instance(usart::USART_Base::USART2_BASE);
    void serialEvent2() __attribute__((weak));
#endif

#ifdef USE_HARDWARE_SERIAL3
    arduino::UsartSerial& Serial3 = arduino::UsartSerial::get_instance(usart::USART_Base::UART3_BASE);
    void serialEvent3() __attribute__((weak));
#endif

#ifdef USE_HARDWARE_SERIAL4
    arduino::UsartSerial& Serial4 = arduino::UsartSerial::get_instance(usart::USART_Base::U4ART4_BASE);
    void serialEvent4() __attribute__((weak));
#endif

void serialEventRun(void) {
#if defined(USE_HARDWARE_SERIAL0)
    if (serialEvent0 && Serial0.available()) {
        serialEvent0();
    }
#endif
#if defined(USE_HARDWARE_SERIAL1)
    if (serialEvent1 && Serial1.available()) {
        serialEvent1();
    }
#endif
#if defined(USE_HARDWARE_SERIAL2)
    if (serialEvent2 && Serial2.available()) {
        serialEvent2();
    }
#endif
#if defined(USE_HARDWARE_SERIAL3)
    if (serialEvent3 && Serial3.available()) {
        serialEvent3();
    }
#endif
#if defined(USE_HARDWARE_SERIAL4)
    if (serialEvent4 && Serial4.available()) {
        serialEvent4();
    }
#endif
}

} // namespace arduino


extern "C" {

#ifdef USE_HARDWARE_SERIAL0
    void USART0_IRQHandler() {
        NVIC_ClearPendingIRQ(USART0_IRQn);
        auto& instance = arduino::UsartSerial::get_instance(usart::USART_Base::USART0_BASE);
        instance.handleInterrupt();
    }
#endif
#ifdef USE_HARDWARE_SERIAL1
    void USART1_IRQHandler() {
        NVIC_ClearPendingIRQ(USART1_IRQn);
        auto& instance = arduino::UsartSerial::get_instance(usart::USART_Base::USART1_BASE);
        instance.handleInterrupt();
    }
#endif
#ifdef USE_HARDWARE_SERIAL2
    void USART2_IRQHandler() {
        NVIC_ClearPendingIRQ(USART2_IRQn);
        auto& instance = arduino::UsartSerial::get_instance(usart::USART_Base::USART2_BASE);
        instance.handleInterrupt();
    }
#endif
#ifdef USE_HARDWARE_SERIAL3
    void UART3_IRQHandler() {
        NVIC_ClearPendingIRQ(UART3_IRQn);
        auto& instance = arduino::UsartSerial::get_instance(usart::USART_Base::UART3_BASE);
        instance.handleInterrupt();
    }
#endif
#ifdef USE_HARDWARE_SERIAL4
    void UART4_IRQHandler() {
        NVIC_ClearPendingIRQ(UART4_IRQn);
        auto& instance = arduino::UsartSerial::get_instance(usart::USART_Base::UART4_BASE);
        instance.handleInterrupt();
    }
#endif

} // extern "C"
