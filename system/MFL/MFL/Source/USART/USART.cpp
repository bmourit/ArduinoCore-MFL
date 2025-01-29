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

#include "USART.hpp"
#include "RCU.hpp"
#include "GPIO.hpp"
#include "CORTEX.hpp"

namespace usart {

template <USART_Base Base>
USART& get_instance_for_base() {
    static USART instance(Base);
    return instance;
}

Result<USART, USART_Error_Type> USART::get_instance(USART_Base Base) {
    switch (Base) {
    case USART_Base::USART0_BASE:
        return get_enum_instance<USART_Base, USART, USART_Error_Type>(
                   Base, USART_Base::USART0_BASE, get_instance_for_base<USART_Base::USART0_BASE>()
               );
     case USART_Base::USART1_BASE:
        return get_enum_instance<USART_Base, USART, USART_Error_Type>(
                   Base, USART_Base::USART1_BASE, get_instance_for_base<USART_Base::USART1_BASE>()
               );
    case USART_Base::USART2_BASE:
        return get_enum_instance<USART_Base, USART, USART_Error_Type>(
                   Base, USART_Base::USART2_BASE, get_instance_for_base<USART_Base::USART2_BASE>()
               );
    case USART_Base::UART3_BASE:
        return get_enum_instance<USART_Base, USART, USART_Error_Type>(
                   Base, USART_Base::UART3_BASE, get_instance_for_base<USART_Base::UART3_BASE>()
               );
    case USART_Base::UART4_BASE:
        return get_enum_instance<USART_Base, USART, USART_Error_Type>(
                   Base, USART_Base::UART4_BASE, get_instance_for_base<USART_Base::UART4_BASE>()
               );
    case USART_Base::INVALID:
    default:
        return RETURN_RESULT(USART, USART_Error_Type::INVALID_USART);
    }
}

std::array<bool, static_cast<size_t>(USART_Base::INVALID)> USART::clock_enabled_ = {false};

USART::USART(USART_Base Base) :
    base_(Base),
    USART_pclk_info_(USART_pclk_index[static_cast<size_t>(Base)]),
    base_address_(USART_baseAddress[static_cast<size_t>(Base)]),
    config_(default_config),
    rx_buffer_(),
    tx_buffer_(),
    interrupt_callbacks_{nullptr}
{
    if (!clock_enabled_[static_cast<size_t>(Base)]) {
        RCU_I.set_pclk_enable(USART_pclk_info_.clock_reg, true);
        RCU_I.set_pclk_reset_enable(USART_pclk_info_.reset_reg, true);
        RCU_I.set_pclk_reset_enable(USART_pclk_info_.reset_reg, false);
        clock_enabled_[static_cast<size_t>(Base)] = true;
    }
    // Initialize default values
    init();
}

/**
 * @brief Resets the USART peripheral by toggling its peripheral clock reset.
 *
 * This function enables the peripheral clock reset for the USART by setting
 * the reset register, then disables the reset to complete the reset operation.
 * This effectively resets all registers of the USART peripheral to their
 * default values.
 */
void USART::reset() {
    RCU_I.set_pclk_reset_enable(USART_pclk_info_.reset_reg, true);
    RCU_I.set_pclk_reset_enable(USART_pclk_info_.reset_reg, false);
}

// USART init() is called with the default configuration after a USART instance is created,
// and ince again when the required configuration is passed by the user.
// Calling pin_config_init() will initialize the required GPIO ports and pins for the instance
// based on the configured paramaters. This includes the pin number, mode, and speed.
// The USART_Config paramaters are then use to finish the USART setup of
// baudrate, parity mode, word length, and stop bits(with HWFC, half-duplex to come).
// The USART instance configuration will also store parameters for the DMA mode.
// The DMA options are NONE, RX, TX, or DUAL, and they are used at operation time.
//
// Passing the configuration structure using the configure() function is the preferred and fastest way
// to set up a U(S)ART.

/**
 * @brief Initialize the USART peripheral with the given configuration.
 *
 * This function will set the USART in the disabled state, and then
 * set the configuration parameters for the USART such as word length,
 * parity, stop bits, and baudrate. The USART will then be enabled.
 *
 * This function should be called immediately after creating a USART
 * instance, or when the configuration needs to be changed.
 * 
 * @param config The configuration structure for the USART.
 */
void USART::init(USART_Config config) {
    // Some bits cannot be written unless USART is disabled
    write_bit(*this, USART_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::UEN), false);

    // Set USART configuration parameters
    write_bit(*this, USART_Regs::CTL0,
               static_cast<uint32_t>(CTL0_Bits::WL), config.word_length == Word_Length::WL_9BITS);
    write_bit_range(*this, USART_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::PMEN), static_cast<uint32_t>(config.parity));
    write_bit_range(*this, USART_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::STB), static_cast<uint32_t>(config.stop_bits));
    set_baudrate(config.baudrate);
    set_direction(config.direction);

    // Enable
    write_bit(*this, USART_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::UEN), true);

    // Store the new config
    config_ = config;
}

/**
 * @brief Releases and disables the USART peripheral
 * 
 * Releases the USART peripheral by disabling the USART and resetting its
 * configuration to the default state. This function will also clear all
 * status flags and disable all interrupts. This function should be
 * called when the USART is no longer needed and its resources should be
 * released. After calling this function, the USART instance should be
 * considered invalid and should not be used again until it is reinitialized
 * with a valid configuration.
 */
void USART::release() {
    constexpr uint32_t all_stat0_flags = ((1U << static_cast<uint32_t>(STAT0_Bits::PERR)) |
                                      (1U << static_cast<uint32_t>(STAT0_Bits::FERR)) |
                                      (1U << static_cast<uint32_t>(STAT0_Bits::NERR)) |
                                      (1U << static_cast<uint32_t>(STAT0_Bits::ORERR)) |
                                      (1U << static_cast<uint32_t>(STAT0_Bits::IDLEF)) |
                                      (1U << static_cast<uint32_t>(STAT0_Bits::RBNE)) |
                                      (1U << static_cast<uint32_t>(STAT0_Bits::TC)) |
                                      (1U << static_cast<uint32_t>(STAT0_Bits::TBE)) |
                                      (1U << static_cast<uint32_t>(STAT0_Bits::LBDF)) |
                                      (1U << static_cast<uint32_t>(STAT0_Bits::CTSF)));

    constexpr uint32_t all_stat1_flags = ((1U << static_cast<uint32_t>(STAT1_Bits::RTF)) |
                                      (1U << static_cast<uint32_t>(STAT1_Bits::EBF)) |
                                      (1U << static_cast<uint32_t>(STAT1_Bits::BSY)));

    constexpr uint32_t all_ctl0_intr = ((1U << static_cast<uint32_t>(CTL0_Bits::PERRIE)) |
                                    (1U << static_cast<uint32_t>(CTL0_Bits::TBEIE)) |
                                    (1U << static_cast<uint32_t>(CTL0_Bits::TCIE)) |
                                    (1U << static_cast<uint32_t>(CTL0_Bits::RBNEIE)) |
                                    (1U << static_cast<uint32_t>(CTL0_Bits::IDLEIE)));

    constexpr uint32_t all_ctl3_intr = ((1U << static_cast<uint32_t>(CTL3_Bits::EBIE)) |
                                    (1U << static_cast<uint32_t>(CTL3_Bits::RTIE)));

    // Clear flags
    write_bits(*this, USART_Regs::STAT0, all_stat0_flags, false);
    write_bits(*this, USART_Regs::STAT1, all_stat1_flags, false);
    
    // Disable interrupts
    write_bits(*this, USART_Regs::CTL0, all_ctl0_intr, false);
    write_bit(*this, USART_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::LBDIE), false);
    write_bit(*this, USART_Regs::CTL2, static_cast<uint32_t>(CTL2_Bits::ERRIE), false);
    write_bits(*this, USART_Regs::CTL3, all_ctl3_intr, true);

    // Reset to default
    config_ = default_config;

    // Disable usart
    set_enable(false);

    // Reset the pclk
    reset();
}

/**
 * @brief Sets the baudrate for the USART peripheral
 *
 * This function calculates the USARTDIV value from the given baudrate and
 * writes it to the USART_BAUD register. The USARTDIV value is calculated as
 * (clock_frequency + baudrate/2) / baudrate.
 *
 * @param baudrate The baudrate for the USART peripheral
 */
void USART::set_baudrate(uint32_t baudrate) {
    // Must not be zero
    if (baudrate == 0U) {
        return;
    }

    uint32_t freq = 0U;
    // Get clock frequency based on the USART instance
    if (base_ == USART_Base::USART0_BASE) {
        freq = RCU_I.get_clock_frequency(rcu::Clock_Frequency::CK_APB2);
    } else {
        freq = RCU_I.get_clock_frequency(rcu::Clock_Frequency::CK_APB1);
    }

    // Calculate the USARTDIV value
    uint32_t usart_div = (freq + baudrate / 2U) / baudrate;

    // Write to the baud rate register
    write_register(*this, USART_Regs::BAUD, usart_div);
}

/**
 * @brief Sets the parity mode for the USART peripheral
 *
 * This function sets the PMEN bit in the USART_CTL0 register to enable or disable
 * parity mode. The parity mode is then set to either PM_EVEN or PM_ODD as specified
 * by the Parity_Mode parameter.
 *
 * @param parity The parity mode for the USART peripheral
 */
void USART::set_parity(Parity_Mode parity) {
    write_bit_ranges(*this, USART_Regs::CTL0,
               static_cast<uint32_t>(CTL0_Bits::PMEN), Clear,
               static_cast<uint32_t>(CTL0_Bits::PMEN), static_cast<uint32_t>(parity));
}

/**
 * @brief Sets the word length for the USART peripheral
 *
 * This function sets the word length to 8 or 9 bits depending on the Word_Length parameter.
 * The word length is set by writing to the WL bit in the USART_CTL0 register.
 *
 * @param word_length The word length for the USART peripheral
 */
void USART::set_word_length(Word_Length word_length) {
    write_bits_sequence(*this, USART_Regs::CTL0,
               static_cast<uint32_t>(CTL0_Bits::WL), false,
               static_cast<uint32_t>(CTL0_Bits::WL), word_length == Word_Length::WL_9BITS);
}

/**
 * @brief Sets the stop bits for the USART peripheral
 *
 * This function sets the stop bits to one and a half, two, or one stop bit
 * depending on the Stop_Bits parameter. The stop bits are set by writing to
 * the STB bits in the USART_CTL1 register.
 *
 * @param stop_bits The number of stop bits for the USART peripheral
 */
void USART::set_stop_bits(Stop_Bits stop_bits) {
    write_bit_ranges(*this, USART_Regs::CTL1,
               static_cast<uint32_t>(CTL1_Bits::STB), Clear,
               static_cast<uint32_t>(CTL1_Bits::STB), static_cast<uint32_t>(stop_bits));
}

/**
 * @brief Enables or disables the USART peripheral
 *
 * This function sets or clears the USART enable (UEN) bit in the USART control
 * register 0. If the USART is enabled, the USART peripheral is enabled and
 * becomes operational. If the USART is disabled, the USART peripheral is
 * disabled and its resources are released.
 *
 * @param enable Set to true to enable the USART peripheral, false to disable it.
 */
void USART::set_enable(bool enable) {
    write_bit(*this, USART_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::UEN), enable);
}

/**
 * @brief Enables the USART peripheral
 *
 * This function sets the USART enable (UEN) bit in the USART control
 * register 0 to true. This enables the USART peripheral and allows it to
 * start operating.
 */
void USART::enable() {
    write_bit(*this, USART_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::UEN), true);
}

/**
 * @brief Disables the USART peripheral
 *
 * This function sets the USART enable (UEN) bit in the USART control
 * register 0 to false. This disables the USART peripheral and releases
 * its resources.
 */
void USART::disable() {
    write_bit(*this, USART_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::UEN), false);
}

/**
 * @brief Sets the USART direction to RX, TX, or RX/TX mode.
 *
 * This function sets the USART direction to receive only, transmit only, or receive and transmit
 * by setting the REN and TEN bits in the USART_CTL0 register. The USART direction is set based on
 * the Direction_Mode parameter.
 *
 * @param direction The direction for the USART peripheral
 */
void USART::set_direction(Direction_Mode direction) {
    switch (direction) {
    case Direction_Mode::RX_MODE:
        write_bits_sequence(*this, USART_Regs::CTL0,
                   static_cast<uint32_t>(CTL0_Bits::REN), false,
                   static_cast<uint32_t>(CTL0_Bits::TEN), false,
                   static_cast<uint32_t>(CTL0_Bits::REN), true);
        break;
    case Direction_Mode::TX_MODE:
        write_bits_sequence(*this, USART_Regs::CTL0,
                   static_cast<uint32_t>(CTL0_Bits::REN), false,
                   static_cast<uint32_t>(CTL0_Bits::TEN), false,
                   static_cast<uint32_t>(CTL0_Bits::TEN), true);
        break;
    case Direction_Mode::RXTX_MODE:
        write_bits_sequence(*this, USART_Regs::CTL0,
                   static_cast<uint32_t>(CTL0_Bits::REN), false,
                   static_cast<uint32_t>(CTL0_Bits::TEN), false,
                   static_cast<uint32_t>(CTL0_Bits::REN), true,
                   static_cast<uint32_t>(CTL0_Bits::TEN), true);
        break;
    case Direction_Mode::RXTX_OFF:
    default:
        write_bits_sequence(*this, USART_Regs::CTL0,
                   static_cast<uint32_t>(CTL0_Bits::REN), false,
                   static_cast<uint32_t>(CTL0_Bits::TEN), false);
        break;
    }
}

/**
 * @brief Sets the MSB (Most Significant Bit) first mode for the USART peripheral.
 *
 * This function sets the MSB first mode for the USART peripheral by setting the MSBF bit in the
 * USART_CTL3 register. The MSB first mode is set based on the MSBF_Mode parameter.
 *
 * @param msbf The MSB first mode to set, which can be either MSBF_MSB or MSBF_LSB
 */
void USART::set_msb(MSBF_Mode msbf) {
    write_bits_sequence(*this, USART_Regs::CTL3,
               static_cast<uint32_t>(CTL3_Bits::MSBF), false,
               static_cast<uint32_t>(CTL3_Bits::MSBF), msbf == MSBF_Mode::MSBF_MSB);
}

/**
 * @brief Sets the USART inversion method enable for the given method.
 *
 * This function sets the USART inversion method enable for the given
 * Inversion_Method parameter. The inversion method is set by setting the
 * corresponding bit in the USART_CTL3 register. The inversion method can
 * be either DATA, TRANSMISSION, or RECEPTION.
 *
 * @param method The Inversion_Method to set the enable for.
 * @param enable Set to true to enable the inversion method, false to disable it.
 */
void USART::set_inversion_method_enable(Inversion_Method method, bool enable) {
    switch (method) {
    case Inversion_Method::DATA:
        write_bit(*this, USART_Regs::CTL3, static_cast<uint32_t>(CTL3_Bits::DINV), enable);
        break;
    case Inversion_Method::TRANSMISSION:
        write_bit(*this, USART_Regs::CTL3, static_cast<uint32_t>(CTL3_Bits::TINV), enable);
        break;
    case Inversion_Method::RECEPTION:
        write_bit(*this, USART_Regs::CTL3, static_cast<uint32_t>(CTL3_Bits::RINV), enable);
        break;
    default:
        break;
    }
}

/**
 * @brief Enables or disables the receiver timeout feature for the USART peripheral.
 *
 * This function enables or disables the receiver timeout feature for the USART peripheral.
 * If the receiver timeout feature is enabled, the USART peripheral will generate an interrupt
 * when the receiver timeout timer expires. If the receiver timeout feature is disabled, the
 * USART peripheral will not generate an interrupt when the receiver timeout timer expires.
 *
 * @param enable Set to true to enable the receiver timeout feature, false to disable it.
 */
void USART::set_rx_timeout_enable(bool enable) {
    write_bit(*this, USART_Regs::CTL3, static_cast<uint32_t>(CTL3_Bits::RTEN), enable);
}

/**
 * @brief Sets the receiver timeout threshold for the USART peripheral.
 *
 * This function sets the receiver timeout threshold for the USART peripheral.
 * The receiver timeout threshold is the number of clock cycles that must elapse
 * before the receiver timeout timer expires. If the receiver timeout feature is
 * enabled, the USART peripheral will generate an interrupt when the receiver
 * timeout timer expires.
 *
 * @param timeout The receiver timeout threshold in clock cycles.
 */
void USART::set_rx_timeout_threshold(uint32_t timeout) {
    write_bit_ranges(*this, USART_Regs::RT,
               static_cast<uint32_t>(RT_Bits::RT), Clear,
               static_cast<uint32_t>(RT_Bits::RT), timeout);
}

/**
 * @brief Transmits a 16-bit data value over the USART bus.
 *
 * This function writes the provided 16-bit data value to the DATA register, which
 * initiates the transmission of the data value over the USART bus.
 *
 * @param data The 16-bit data value to be transmitted.
 */
void USART::send_data(uint16_t data) {
    write_bit_range(*this, USART_Regs::DATA, static_cast<uint32_t>(DATA_Bits::DATA), static_cast<uint32_t>(data));
}

/**
 * @brief Receives a 16-bit data value from the USART peripheral.
 *
 * This function reads the 16-bit data value from the DATA register, retrieving
 * the received data value from the USART bus.
 *
 * @return The 16-bit data value received from the USART peripheral.
 */
uint16_t USART::receive_data16() {
    return static_cast<uint16_t>(read_bit_range(*this, USART_Regs::DATA, static_cast<uint32_t>(DATA_Bits::DATA)) & 0xFFFFU);
}

/**
 * @brief Transmits an 8-bit data value over the USART bus.
 *
 * This function writes the provided 8-bit data value to the DATA register, which
 * initiates the transmission of the data value over the USART bus.
 *
 * @param data The 8-bit data value to be transmitted.
 */
void USART::send_data(uint8_t data) {
    write_bit_range(*this, USART_Regs::DATA, static_cast<uint32_t>(DATA_Bits::DATA), static_cast<uint32_t>(data));
}

/**
 * @brief Receives an 8-bit data value from the USART peripheral.
 *
 * This function reads an 8-bit data value from the DATA register, retrieving
 * the received data value from the USART bus.
 *
 * @return The 8-bit data value received from the USART peripheral.
 */
uint8_t USART::receive_data8() {
    return static_cast<uint8_t>(read_bit_range(*this, USART_Regs::DATA, static_cast<uint32_t>(DATA_Bits::DATA)) & 0xFFU);
}

/**
 * @brief Sets the USART's wake-up address.
 *
 * This function sets the USART's wake-up address to the specified address value.
 * The USART peripheral can be configured to wake up the system from sleep mode
 * when it receives a character matching the wake-up address.
 *
 * @param address The wake-up address to be set for the USART peripheral.
 */
void USART::set_wakeup_address(uint8_t address) {
    write_bit_ranges(*this, USART_Regs::CTL1,
               static_cast<uint32_t>(CTL1_Bits::ADDR), Clear,
               static_cast<uint32_t>(CTL1_Bits::ADDR), static_cast<uint32_t>(address));
}

/**
 * @brief Enables or disables the USART peripheral's mute mode.
 *
 * This function enables or disables the USART peripheral's mute mode.
 * If mute mode is enabled, the USART peripheral will not send any data out
 * onto the bus. If mute mode is disabled, the USART peripheral will send data
 * out onto the bus as normal.
 *
 * @param enable Set to true to enable mute mode, false to disable it.
 */
void USART::mute_mode_enable(bool enable) {
    write_bit(*this, USART_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::RWU), enable);
}

/**
 * @brief Configures the USART peripheral to wake up the system on a specific
 *        event while in mute mode.
 *
 * This function configures the USART peripheral to wake up the system when it
 * receives a character matching the wake-up address (WM_ADDR) or when it
 * receives any character (WM_IDLE), while in mute mode.
 *
 * @param wakeup_mode The wake-up mode to be set for the USART peripheral while
 *                    in mute mode.
 */
void USART::set_mute_mode_wakeup(Wakeup_Mode wakeup_mode) {
    write_bits_sequence(*this, USART_Regs::CTL0,
               static_cast<uint32_t>(CTL0_Bits::WM), false,
               static_cast<uint32_t>(CTL0_Bits::WM), wakeup_mode == Wakeup_Mode::WM_ADDR);
}

/**
 * @brief Enables or disables the half-duplex mode for the USART peripheral.
 *
 * This function sets the HDEN bit in the CTL2 register to enable or disable
 * the half-duplex mode for the USART peripheral. In half-duplex mode, the USART
 * peripheral acts as a single-wire half-duplex asynchronous communication
 * interface.
 *
 * @param enable Set to true to enable the half-duplex mode, false to disable it.
 */
void USART::set_half_duplex_enable(bool enable) {
    write_bit(*this, USART_Regs::CTL2, static_cast<uint32_t>(CTL2_Bits::HDEN), enable);
}

/**
 * @brief Enables or disables the synchronous clock for the USART peripheral.
 *
 * This function sets the CKEN bit in the CTL1 register to enable or disable
 * the synchronous clock for the USART peripheral. When enabled, the USART
 * peripheral will use the synchronous clock in synchronous mode.
 *
 * @param enable Set to true to enable the synchronous clock, false to disable it.
 */
void USART::set_synchronous_clock_enable(bool enable) {
    write_bit(*this, USART_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::CKEN), enable);
}

/**
 * @brief Configures the synchronous clock for the USART peripheral.
 *
 * This function configures the synchronous clock for the USART peripheral.
 * It sets the CLEN, CPH, and CPL bits in the CTL1 register to configure the
 * length, phase, and polarity of the synchronous clock.
 *
 * @param length The length of the pulse in the synchronous clock.
 * @param phase The phase of the synchronous clock.
 * @param polarity The polarity of the synchronous clock.
 */
void USART::synchronous_clock_configure(Pulse_Length length, Clock_Phase phase, Clock_Polarity polarity) {
    write_bits_sequence(*this, USART_Regs::CTL1,
               static_cast<uint32_t>(CTL1_Bits::CPH), phase == Clock_Phase::SECOND_CLOCK,
               static_cast<uint32_t>(CTL1_Bits::CPL), polarity == Clock_Polarity::POLARITY_HIGH,
               static_cast<uint32_t>(CTL1_Bits::CLEN), length == Pulse_Length::EXT_PULSE_ENABLE);
}

/**
 * @brief Enables or disables DMA for receiving data for the USART peripheral.
 *
 * This function sets the DENR bit in the CTL2 register to enable or disable
 * DMA for receiving data for the USART peripheral. When enabled, the USART
 * peripheral will use DMA to receive data.
 *
 * @param enable Set to true to enable DMA, false to disable it.
 */
void USART::receive_data_dma_enable(bool enable) {
    write_bit(*this, USART_Regs::CTL2,static_cast<uint32_t>(CTL2_Bits::DENR), enable);
}

/**
 * @brief Enables or disables DMA for sending data for the USART peripheral.
 *
 * This function sets the DENT bit in the CTL2 register to enable or disable
 * DMA for sending data for the USART peripheral. When enabled, the USART
 * peripheral will use DMA to send data.
 *
 * @param enable Set to true to enable DMA, false to disable it.
 */
void USART::send_data_dma_enable(bool enable) {
    write_bit(*this, USART_Regs::CTL2, static_cast<uint32_t>(CTL2_Bits::DENT), enable);
}

/**
 * @brief Enables or disables the LIN mode for the USART peripheral.
 *
 * This function sets the LMEN bit in the CTL1 register to enable or disable
 * the LIN mode for the USART peripheral. When enabled, the USART peripheral
 * will operate in LIN mode and will support LIN protocol communication.
 *
 * @param enable Set to true to enable LIN mode, false to disable it.
 */
void USART::set_lin_mode_enable(bool enable) {
    write_bit(*this, USART_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::LMEN), enable);
}

/**
 * @brief Configures the length of the LIN break frame for the USART peripheral.
 *
 * This function sets the LBLEN bit in the CTL1 register to configure the
 * length of the LIN break frame for the USART peripheral. When set to true,
 * the break frame length is set to 11 bits, otherwise it is set to 10 bits.
 *
 * @param length The length of the LIN break frame to be set. Can be either
 *               Break_Length::LENGTH_10B or Break_Length::LENGTH_11B.
 */
void USART::set_lin_frame_break_length(Break_Length length) {
    write_bit(*this, USART_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::LBLEN), length == Break_Length::LENGTH_11B);
}

/**
 * @brief Sends a LIN break frame.
 *
 * This function sets the SBKCMD bit in the CTL0 register to true, which sends
 * a LIN break frame. The break frame is used to wake up a LIN slave device
 * from a sleep mode.
 */
void USART::send_lin_frame_break() {
    write_bit(*this, USART_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SBKCMD), true);
}

/**
 * @brief Configures the guard time for the USART peripheral.
 *
 * This function sets the GUAT bits in the GP register to configure the guard
 * time for the USART peripheral. The guard time is the time between the end of
 * the last stop bit and the start of the next start bit. The guard time must be
 * longer than the maximum propagation delay of the bus.
 *
 * @param guard_time The guard time to be set, in the range of 0 to 255 (inclusive).
 */
void USART::set_guard_time(uint8_t guard_time) {
    write_bit_range(*this, USART_Regs::GP, static_cast<uint32_t>(GP_Bits::GUAT), static_cast<uint32_t>(guard_time));
}

/**
 * @brief Enables or disables the smartcard mode for the USART peripheral.
 *
 * This function sets the SCEN bit in the CTL2 register to enable or disable
 * the smartcard mode for the USART peripheral. When enabled, the USART
 * peripheral will operate in smartcard mode and will support smartcard
 * protocol communication.
 *
 * @param enable Set to true to enable smartcard mode, false to disable it.
 */
void USART::set_smardcard_mode_enable(bool enable) {
    write_bit(*this, USART_Regs::CTL2, static_cast<uint32_t>(CTL2_Bits::SCEN), enable);
}

/**
 * @brief Enables or disables the NACK mode for the USART peripheral in smartcard
 *        mode.
 *
 * This function sets the NKEN bit in the CTL2 register to enable or disable
 * the NACK mode for the USART peripheral in smartcard mode. When enabled, the
 * USART peripheral will send a NACK (negative acknowledge) after receiving a
 * block from the smartcard. When disabled, the USART peripheral will not send
 * a NACK.
 *
 * @param enable Set to true to enable NACK mode, false to disable it.
 */
void USART::set_smartcard_nack_mode_enable(bool enable) {
    write_bit(*this, USART_Regs::CTL2, static_cast<uint32_t>(CTL2_Bits::NKEN), enable);
}

/**
 * @brief Configures the auto-retry count for the USART peripheral in smartcard
 *        mode.
 *
 * This function sets the SCRTNUM bits in the CTL3 register to configure the
 * auto-retry count for the USART peripheral in smartcard mode. The auto-retry
 * count is the number of times the USART peripheral will retransmit a block to
 * the smartcard if it receives a NACK (negative acknowledge) after receiving
 * a block from the smartcard.
 *
 * @param retry_count The auto-retry count to be set, in the range of 0 to 15
 *                    (inclusive).
 */
void USART::set_smartcard_auto_retry(uint8_t retry_count) {
    write_bit_range(*this, USART_Regs::CTL3, static_cast<uint32_t>(CTL3_Bits::SCRTNUM), static_cast<uint32_t>(retry_count));
}

/**
 * @brief Sets the block size for the USART peripheral in smartcard mode.
 *
 * This function configures the block size for the USART peripheral when
 * operating in smartcard mode by setting the BL bits in the RT register.
 * The block size determines the number of bytes to be sent in a single
 * transmission block. This is important for communication with smartcards
 * that require a specific block size for data exchange.
 *
 * @param size The block size to be set, in the range of 0 to 255 (inclusive).
 */
void USART::set_smartcard_block_size(uint8_t size) {
    write_bit_range(*this, USART_Regs::RT, static_cast<uint32_t>(RT_Bits::BL), static_cast<uint32_t>(size));
}

/**
 * @brief Enables or disables the IrDA mode for the USART peripheral.
 *
 * This function sets the IREN bit in the CTL2 register to enable or disable
 * the IrDA mode for the USART peripheral. When enabled, the USART peripheral
 * will operate in IrDA mode and will support IrDA protocol communication.
 *
 * @param enable Set to true to enable IrDA mode, false to disable it.
 */
void USART::set_irda_mode_enable(bool enable) {
    write_bit(*this, USART_Regs::CTL2, static_cast<uint32_t>(CTL2_Bits::IREN), enable);
}

/**
 * @brief Configures the prescaler for the IrDA low power mode.
 *
 * This function sets the PSC bits in the GP register to configure the
 * prescaler for the IrDA low power mode. The prescaler determines the
 * division factor applied to the clock signal provided to the IrDA
 * peripheral.
 *
 * @param prescaler The prescaler to be set, in the range of 0 to 255
 *                  (inclusive).
 */
void USART::set_irda_low_power_prescaler(uint8_t prescaler) {
    write_bit_range(*this, USART_Regs::GP, static_cast<uint32_t>(GP_Bits::PSC), static_cast<uint32_t>(prescaler));
}

/**
 * @brief Configures the power mode for the IrDA peripheral.
 *
 * This function sets the IRLP bit in the CTL2 register to configure the
 * power mode for the IrDA peripheral. When set to true, the IrDA peripheral
 * enters low power mode, and when set to false, the IrDA peripheral operates
 * in normal power mode.
 *
 * @param power The power mode to be set, either LOW or NORMAL.
 */
void USART::set_irda_power_mode(IrDA_Power power) {
    write_bit(*this, USART_Regs::CTL2, static_cast<uint32_t>(CTL2_Bits::IRLP), power == IrDA_Power::LOW);
}

/**
 * @brief Enables or disables the RTS (Request to Send) pin hardware flow control
 *        for the USART peripheral.
 *
 * This function sets the RTSEN bit in the CTL2 register to enable or disable
 * the RTS pin hardware flow control for the USART peripheral. When enabled, the
 * RTS pin is used to signal to the transmitter that it should pause transmission
 * when the receive buffer is full. When disabled, the RTS pin is not used for
 * hardware flow control.
 *
 * @param enable Set to true to enable hardware flow control using the RTS pin,
 *               false to disable it.
 */
void USART::set_hwfc_rts_enable(bool enable) {
    write_bit(*this, USART_Regs::CTL2, static_cast<uint32_t>(CTL2_Bits::RTSEN), enable);
}

/**
 * @brief Enables or disables the CTS (Clear to Send) pin hardware flow control
 *        for the USART peripheral.
 *
 * This function sets the CTSEN bit in the CTL2 register to enable or disable
 * the CTS pin hardware flow control for the USART peripheral. When enabled, the
 * CTS pin is used to signal to the transmitter that it should pause transmission
 * when the receive buffer is full. When disabled, the CTS pin is not used for
 * hardware flow control.
 *
 * @param enable Set to true to enable hardware flow control using the CTS pin,
 *               false to disable it.
 */
void USART::set_hwfc_cts_enable(bool enable) {
    write_bit(*this, USART_Regs::CTL2, static_cast<uint32_t>(CTL2_Bits::CTSEN), enable);
}

/**
 * @brief Retrieves the status of the specified flag.
 *
 * This function reads the specified flag from the appropriate register and
 * returns its status. If the flag is not applicable to the USART peripheral,
 * this function returns false.
 *
 * @param flag The status flag to retrieve. Must be a value from the
 *             Status_Flags enumeration.
 * @return true if the flag is set, false otherwise.
 */
bool USART::get_flag(Status_Flags flag) {
    const auto& config = status_config[static_cast<size_t>(flag)];
    return read_bit_range(*this, config.reg, config.bit) != Clear;
}

/**
 * @brief Clears a specified status flag in the USART peripheral.
 *
 * This function clears a given status flag specified by the Status_Flags
 * enumeration. The flag is cleared by writing a 0 to the corresponding bit
 * in the appropriate register. If the flag is not applicable to the USART
 * peripheral, this function does nothing.
 *
 * @param flag The status flag to clear. Must be a value from the
 *             Status_Flags enumeration.
 */
void USART::clear_flag(Status_Flags flag) {
    const auto& config = status_config[static_cast<size_t>(flag)];
    write_bit_range(*this, config.reg, config.bit, Clear);
}

/**
 * @brief Retrieves the status of the specified interrupt flag.
 *
 * This function checks if the specified interrupt flag is set and enabled.
 * The flag is checked by reading the corresponding bit in the appropriate
 * register and comparing it to the value Clear. The flag is enabled if the
 * corresponding enable bit in the appropriate register is set to the value
 * Set. If the flag is not applicable to the USART peripheral, this function
 * returns false.
 *
 * @param flag The interrupt flag to retrieve. Must be a value from the
 *             Interrupt_Flags enumeration.
 * @return true if the flag is set and enabled, false otherwise.
 */
bool USART::get_interrupt_flag(Interrupt_Flags flag) {
    const auto& config = interrupt_flags_config[static_cast<size_t>(flag)];
    bool flag_set = read_bit_range(*this, config.reg, config.bit) != Clear;
    auto bit_value = config.invert ? Set : Clear;
    bool is_enabled = read_bit_range(*this, config.enable_reg, config.enable_bit) != bit_value;
    return (flag_set && is_enabled);
}

/**
 * @brief Clears a specified interrupt flag in the USART peripheral.
 *
 * This function clears a given interrupt flag specified by the Interrupt_Flags
 * enumeration. The flag is cleared by writing a 0 to the corresponding bit
 * in the appropriate register. If the flag is not applicable to the USART
 * peripheral, this function does nothing.
 *
 * @param flag The interrupt flag to clear. Must be a value from the
 *             Interrupt_Flags enumeration.
 */
void USART::clear_interrupt_flag(Interrupt_Flags flag) {
    const auto& config = interrupt_flags_config[static_cast<size_t>(flag)];
    write_bit_range(*this, config.reg, config.bit, Clear);
}

/**
 * @brief Sets the interrupt priority of the USART peripheral.
 *
 * This function sets the interrupt priority of the USART peripheral.
 * The priority is specified by two parameters: prepriority and subpriority.
 * The prepriority specifies the priority of the interrupt in the NVIC,
 * and the subpriority specifies the priority of the interrupt within the
 * USART peripheral. The subpriority is used to prioritize interrupts within
 * the peripheral. If two interrupts have the same prepriority, the one with
 * the higher subpriority takes precedence.
 *
 * @param prepriority The prepriority of the interrupt.
 * @param subpriority The subpriority of the interrupt.
 */
void USART::set_interrupt_priority(uint8_t prepriority, uint8_t subpriority) {
    config_.interrupt_prepriority = prepriority;
    config_.interrupt_subpriority = subpriority;
}

/**
 * @brief Enables or disables a specific interrupt type in the USART peripheral.
 *
 * This function configures the interrupt settings for the USART peripheral by
 * enabling or disabling the specified interrupt type. It modifies the interrupt
 * enable register to reflect the desired configuration.
 *
 * @param type The interrupt type to enable or disable. Must be a value from the
 *             Interrupt_Type enumeration.
 * @param enable Set to true to enable the interrupt, false to disable it.
 */
void USART::set_interrupt_enable(Interrupt_Type type, bool enable) {
    if (type == Interrupt_Type::INTR_COUNT ||
        static_cast<size_t>(type) >= static_cast<size_t>(Interrupt_Type::INTR_COUNT)) {
        return;
    }
    const auto& config = interrupt_config[static_cast<size_t>(type)];
    bool bit_value = config.invert ? !enable : enable;
    write_bit_range(*this, config.reg, config.bit_info, bit_value ? Set : Clear);
}

/**
 * @brief Registers an interrupt callback for the specified interrupt type in
 *        the USART peripheral.
 * 
 * Registers an interrupt callback for the specified interrupt type in the USART
 * peripheral. The interrupt callback is called when the specified interrupt
 * occurs. The callback is called with the USART instance as its only parameter.
 *
 * @param type The interrupt type for which to register the callback. Must be a
 *             value from the Interrupt_Type enumeration.
 * @param callback The callback function to register. The callback takes a single
 *                 parameter of type USART&.
 */
void USART::register_interrupt_callback(Interrupt_Type type, IRQCallback callback) {
    if (type == Interrupt_Type::INTR_COUNT || 
        static_cast<size_t>(type) >= static_cast<size_t>(Interrupt_Type::INTR_COUNT)) {
        return;
    }
    interrupt_callbacks_[static_cast<size_t>(type)] = callback;
}

/**
 * @brief Unregisters an interrupt callback for the specified interrupt type in
 *        the USART peripheral.
 *
 * This function sets the callback for the specified interrupt type to nullptr.
 * Any previously registered callback will be lost.
 *
 * @param type The interrupt type for which to unregister the callback. Must be a
 *             value from the Interrupt_Type enumeration.
 */
void USART::unregister_interrupt_callback(Interrupt_Type type) {
    interrupt_callbacks_[static_cast<size_t>(type)] = nullptr;
}

/**
 * @brief Handles errors in the USART peripheral.
 *
 * This function checks the USART status flags for error conditions and
 * clears the corresponding flags if an error is detected.
 *
 * The errors handled by this function are:
 * - Overrun error (FLAG_ORERR)
 * - Noise error (FLAG_NERR)
 * - Framing error (FLAG_FERR)
 */
void USART::handle_errors() {
    // Handle overrun error
    if (get_flag(usart::Status_Flags::FLAG_ORERR)) {
        clear_flag(usart::Status_Flags::FLAG_ORERR);
    }

    // Handle noise error
    if (get_flag(usart::Status_Flags::FLAG_NERR)) {
        clear_flag(usart::Status_Flags::FLAG_NERR);
    }

    // Handle framing error
    if (get_flag(usart::Status_Flags::FLAG_FERR)) {
        clear_flag(usart::Status_Flags::FLAG_FERR);
    }
}

/**
 * @brief Checks the USART status flags for error conditions and handles them
 *        accordingly.
 *
 * This function checks the USART status flags for overrun, noise, and framing
 * errors. If any of these errors are detected, the function sets the USART state
 * to ERROR and calls the registered error callback if one is available. If no
 * callback is registered, the function calls handle_errors() to clear the
 * error flags. If no error is detected, the function sets the USART state back
 * to BUSY_RECEIVE.
 */
bool USART::check_error_flags() {
    if (get_interrupt_flag(Interrupt_Flags::INTR_FLAG_CTL2_ORERR) ||
            get_interrupt_flag(Interrupt_Flags::INTR_FLAG_CTL2_NERR) ||
            get_interrupt_flag(Interrupt_Flags::INTR_FLAG_CTL2_FERR)) {
        config_.state = USART_State::ERROR;
        if (interrupt_callbacks_[static_cast<size_t>(Interrupt_Type::INTR_ERRIE)]) {
            interrupt_callbacks_[static_cast<size_t>(Interrupt_Type::INTR_ERRIE)]();
        } else {
            handle_errors();
        }
        return true;
    }
    // No error - back to reception state
    config_.state = USART_State::BUSY_RECEIVE;
    return false;
}

/**
 * @brief Handles the receive interrupt.
 *
 * This function is called when a receive interrupt occurs. It clears the
 * receive interrupt flag, checks for errors, and handles the received data.
 * If an error is detected, it sets the USART state to ERROR and calls the
 * registered error callback if one is available. Otherwise, it sets the USART
 * state to BUSY_RECEIVE and calls the registered receive callback if one is
 * available. If no callback is registered, it sets the USART state back to
 * IDLE.
 */
void USART::handle_rx_interrupt() {
    if (check_error_flags()) {
        return;
    }

    uint8_t data = receive_data8();
    rx_buffer_.write(data);
    config_.last_data = data;

    config_.state = USART_State::IDLE;

    const auto rx_callback = interrupt_callbacks_[static_cast<size_t>(Interrupt_Type::INTR_RBNEIE)];
    if (rx_callback) {
        rx_callback();
    }
}

/**
 * @brief Handles the transmit interrupt.
 *
 * This function is called when a transmit interrupt occurs. It reads the next
 * byte from the transmit buffer and sends it over the USART. If the buffer is
 * empty, it switches to the TC interrupt for completion. It then calls the
 * registered transmit callback if one is available, and sets the USART state
 * back to IDLE.
 */
void USART::handle_tx_interrupt() {
    set_interrupt_enable(Interrupt_Type::INTR_TBEIE, false);

    uint8_t data;
    if (tx_buffer_.read(data)) {
        send_data(data);
        config_.state = USART_State::IDLE;
    	const auto tx_callback = interrupt_callbacks_[static_cast<size_t>(Interrupt_Type::INTR_TBEIE)];
    	if (tx_callback) {
            tx_callback();
    	}
        set_interrupt_enable(Interrupt_Type::INTR_TBEIE, true);
    } else {
        config_.state = USART_State::IDLE;
        set_interrupt_enable(Interrupt_Type::INTR_TCIE, true);
    }
}

/**
 * @brief Handles the transfer complete interrupt.
 *
 * This function is called when a transfer complete interrupt occurs. It clears
 * the transfer complete interrupt flag, disables the transfer complete interrupt
 * enable, and calls the registered transfer complete callback if one is
 * available. It then sets the USART state back to IDLE.
 */
void USART::handle_transfer_complete_interrupt() {
    clear_interrupt_flag(Interrupt_Flags::INTR_FLAG_CTL0_TC);
    set_interrupt_enable(Interrupt_Type::INTR_TCIE, false);

    config_.state = USART_State::IDLE;

    const auto tc_callback = interrupt_callbacks_[static_cast<size_t>(Interrupt_Type::INTR_TCIE)];
    if (tc_callback) {
        tc_callback();
    }
}

/**
 * @brief Handles USART interrupts.
 *
 * This function is called when a USART interrupt occurs. It determines which
 * interrupt occurred and calls the appropriate handler function. The handlers
 * are responsible for clearing the interrupt flags and performing the necessary
 * actions. They also call any registered callbacks if available.
 */
void USART::handle_interrupt() {
    if (get_interrupt_flag(Interrupt_Flags::INTR_FLAG_CTL0_RBNE)) {
        config_.state = USART_State::BUSY_RECEIVE;
        handle_rx_interrupt();
    }

    if (get_interrupt_flag(Interrupt_Flags::INTR_FLAG_CTL0_TBE)) {
        config_.state = USART_State::BUSY_TRANSMIT;
        handle_tx_interrupt();
    }

    if (get_interrupt_flag(Interrupt_Flags::INTR_FLAG_CTL0_TC)) {
        config_.state = USART_State::BUSY_COMPLETION;
        handle_transfer_complete_interrupt();
    }
    // TODO: other interrupts...
}

/**
 * @brief Prepares and enables the USART receive interrupts.
 *
 * This function sets up the USART receive interrupts. It enables the
 * parity error interrupt if the USART is configured to use a parity bit, the
 * error interrupt, and the receive buffer not empty interrupt. It is called
 * internally by the USART class to prepare the USART for receive interrupts.
 * It does not need to be called by the user.
 */
void USART::prepare_receive_interrupts() {
    if (config_.parity != Parity_Mode::PM_NONE) {
        set_interrupt_enable(Interrupt_Type::INTR_PERRIE, true);
    }
    set_interrupt_enable(Interrupt_Type::INTR_ERRIE, true);
    set_interrupt_enable(Interrupt_Type::INTR_RBNEIE, true);
}

// Non-blocking

/**
 * @brief Reads data from the receive buffer using interrupts.
 *
 * This function reads data from the USART receive buffer using interrupts.
 * It enables the receive buffer not empty interrupt and the error interrupt.
 * If data is available in the receive buffer, it is read and returned in the
 * given parameter. If no data is available, the function returns false.
 * @param data Reference to a uint8_t to store the received data in
 * @return true if data was read, false otherwise
 */
bool USART::usart_receive_interrupt(uint8_t& data) {
    while (config_.state == USART_State::BUSY_COMPLETION ||
           config_.state == USART_State::BUSY_TRANSMIT) {
    }
    set_interrupt_enable(Interrupt_Type::INTR_ERRIE, true);
    set_interrupt_enable(Interrupt_Type::INTR_RBNEIE, true);

    return rx_buffer_.read(data);
}

/**
 * @brief Sends data from the transmit buffer using interrupts.
 *
 * This function sends data from the USART transmit buffer using interrupts.
 * It enables the transmit buffer empty interrupt and the error interrupt.
 * If data is available in the transmit buffer, it is sent and the function returns true.
 * If no data is available, the function returns false.
 * @param data the data to send
 * @return true if data was sent, false otherwise
 */
bool USART::usart_transmit_interrupt(uint8_t data) {
    // If transfer complete or reception IRQ is in progress
    // write to the txbuffer if there is space, otherwise
    // just wait for state transition.
    while (config_.state == USART_State::BUSY_COMPLETION ||
           config_.state == USART_State::BUSY_RECEIVE) {
        if (!tx_buffer_.isFull()) {
            tx_buffer_.write(data);
        }
    }

    if (tx_buffer_.write(data)) {
        set_interrupt_enable(Interrupt_Type::INTR_TBEIE, true);
        return true;
    }

    // If the txbuffer is full, enable the interrupt
    // to transmit and make space
    if (tx_buffer_.isFull()) {
        set_interrupt_enable(Interrupt_Type::INTR_TBEIE, true);
    }

    return false;   // No room in buffer
}

// Blocking

/**
 * @brief Receives data from the USART using polling.
 *
 * This function waits for data to be available in the USART receive buffer
 * by continuously checking the receive buffer not empty flag.
 * Once data is available, it reads the data into the provided reference.
 * If data is successfully read, the function returns true.
 * If no data is available or an error occurs, the function returns false.
 *
 * @param data Reference to a uint8_t to store the received data in.
 * @return true if data was successfully read, false otherwise.
 */

bool USART::usart_receive_polling(uint8_t& data) {
    // TODO
    return false;
}

/**
 * @brief Sends data from the transmit buffer using polling.
 *
 * This function waits for the USART transmit buffer to be empty by
 * continuously checking the transmit buffer empty flag.
 * Once the transmit buffer is empty, it sends the given data and returns true.
 * If no data is available or an error occurs, the function returns false.
 *
 * @param data the data to send
 * @return true if data was successfully sent, false otherwise
 */
bool USART::usart_transmit_polling(uint8_t data) {
    // TODO
    return false;
}

// DMA

/**
 * @brief Receives data from the USART using DMA.
 *
 * This function enables the receive DMA and waits for the DMA to complete.
 * Once the DMA is complete, it reads the data into the provided reference.
 * If data is successfully read, the function returns true.
 * If no data is available or an error occurs, the function returns false.
 *
 * @param data Reference to a uint8_t to store the received data in.
 * @return true if data was successfully read, false otherwise.
 */
bool USART::usart_receive_dma(uint8_t& data) {
    // TODO
    return false;
}

/**
 * @brief Sends data from the transmit buffer using DMA.
 *
 * This function enables the transmit DMA and waits for the DMA to complete.
 * Once the DMA is complete, it returns true.
 * If no data is available or an error occurs, the function returns false.
 *
 * @param data the data to send
 * If no data is available or an error occurs, the function returns false.
 *
 * @param data the data to send
 * @return true if data was successfully sent, false otherwise
 */
bool USART::usart_transmit_dma(uint8_t data) {
    // TODO
    return false;
}

// DMA with interrupt

/**
 * @brief Receives data from the USART using DMA with interrupts.
 *
 * This function enables the receive DMA and interrupts and waits for
 * the DMA to complete. Once the DMA is complete, it reads the data into
 * the provided reference. If data is successfully read, the function
 * returns true. If no data is available or an error occurs, the
 * function returns false.
 *
 * @param data Reference to a uint8_t to store the received data in.
 * @return true if data was successfully read, false otherwise
 */
bool USART::usart_receive_dma_interrupt(uint8_t& data) {
    // TODO
    return false;
}

/**
 * @brief Sends data from the transmit buffer using DMA with interrupts.
 *
 * This function enables the transmit DMA and interrupts and waits for
 * the DMA to complete. Once the DMA is complete, it returns true.
 * If no data is available or an error occurs, the function returns false.
 *
 * @param data the data to send
 * @return true if data was successfully sent, false otherwise
 */
bool USART::usart_transmit_dma_interrupt(uint8_t data) {
    // TODO
    return false;
}
} // namespace usart
