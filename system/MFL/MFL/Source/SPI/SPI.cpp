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

#include "SPI.hpp"
#include "RCU.hpp"

namespace spi {

template <SPI_Base Base>
SPI& get_instance_for_base() {
    static SPI instance(Base);
    return instance;
}

Result<SPI, SPI_Error_Type> SPI::get_instance(SPI_Base Base) {
    switch (Base) {
    case SPI_Base::SPI0_BASE:
        return get_enum_instance<SPI_Base, SPI, SPI_Error_Type>(
                   Base, SPI_Base::SPI0_BASE, get_instance_for_base<SPI_Base::SPI0_BASE>()
               );
    case SPI_Base::SPI1_BASE:
        return get_enum_instance<SPI_Base, SPI, SPI_Error_Type>(
                   Base, SPI_Base::SPI1_BASE, get_instance_for_base<SPI_Base::SPI1_BASE>()
               );
    case SPI_Base::SPI2_BASE:
        return get_enum_instance<SPI_Base, SPI, SPI_Error_Type>(
                   Base, SPI_Base::SPI2_BASE, get_instance_for_base<SPI_Base::SPI2_BASE>()
               );
    case SPI_Base::INVALID:
    default:
        return RETURN_RESULT(SPI, SPI_Error_Type::INVALID_SPI);
    }
}

std::array<bool, static_cast<size_t>(SPI_Base::INVALID)> SPI::clock_enabled_ = {false};

SPI::SPI(SPI_Base Base) :
    base_(Base),
    SPI_pclk_info_(SPI_pclk_index[static_cast<uint8_t>(Base)]),
    base_address_(SPI_baseAddress[static_cast<uint8_t>(Base)]),
    config_(default_config)
{
    if (!clock_enabled_[static_cast<uint8_t>(Base)]) {
        RCU_I.set_pclk_enable(SPI_pclk_info_.clock_reg, true);
        RCU_I.set_pclk_reset_enable(SPI_pclk_info_.reset_reg, true);
        RCU_I.set_pclk_reset_enable(SPI_pclk_info_.reset_reg, false);
        clock_enabled_[static_cast<uint8_t>(Base)] = true;
    }
    // Initialize default values
    init();
}

/**
 * Resets the SPI peripheral by toggling its peripheral clock reset.
 * This function enables the reset of the SPI peripheral by setting the
 * reset register, then disables the reset to complete the operation.
 */
void SPI::reset() {
    RCU_I.set_pclk_reset_enable(SPI_pclk_info_.reset_reg, true);
    RCU_I.set_pclk_reset_enable(SPI_pclk_info_.reset_reg, false);
}

/**
 * @brief Initializes the SPI peripheral.
 *
 * This function configures the SPI peripheral's operational mode, frame format, polarity,
 * clock phase, endianness, and NSS type, as well as the divider for the peripheral clock.
 *
 * @param config The SPI configuration structure containing the desired settings.
 */
void SPI::init(SPI_Config config) {
    // Frame format
    if (config.frame_format == Frame_Format::FF_16BIT) {
        write_bit(*this, SPI_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::FF16), true);
    }
    // Polarity
    if (config.polarity_pull == Clock_Polarity::PULL_HIGH) {
        write_bit(*this, SPI_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::CKPL), true);
    }
    // Clock phase
    if (config.clock_phase == Clock_Phase::PHASE_SECOND_EDGE) {
        write_bit(*this, SPI_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::CKPH), true);
    }
    // MSBF
    if (config.msbf == Endian_Type::LSBF) {
        write_bit(*this, SPI_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::LF), true);
    }
    // NSS
    if (config.nss_type == NSS_Type::SOFTWARE_NSS) {
        write_bit(*this, SPI_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SWNSSEN), true);
    }
    // TODO:
    // 	Check this needed for initialization.
    // 	Datasheet says it isn't, but equivalent
    // 	STM32 drivers do this. Keeping for now
    write_bit_range(*this, SPI_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::PSC), static_cast<uint32_t>(config.pclk_divider));

    // Set SPI operational mode
    // Operational_Mode in config file
    switch (config.operational_mode) {
    case Operational_Mode::MFD_MODE:
    case Operational_Mode::MTU_MODE:
        write_bit(*this, SPI_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::MSTMOD), true);
        break;
    case Operational_Mode::MRU_MODE:
        write_bits_sequence(*this, SPI_Regs::CTL0,
                   static_cast<uint32_t>(CTL0_Bits::MSTMOD), true,
                   static_cast<uint32_t>(CTL0_Bits::RO), true);
        break;
    case Operational_Mode::MTB_MODE:
        write_bits_sequence(*this, SPI_Regs::CTL0,
                   static_cast<uint32_t>(CTL0_Bits::MSTMOD), true,
                   static_cast<uint32_t>(CTL0_Bits::BDEN), true,
                   static_cast<uint32_t>(CTL0_Bits::BDOEN), true);
        break;
    case Operational_Mode::MRB_MODE:
        write_bits_sequence(*this, SPI_Regs::CTL0,
                   static_cast<uint32_t>(CTL0_Bits::MSTMOD), true,
                   static_cast<uint32_t>(CTL0_Bits::BDEN), true);
        break;
    case Operational_Mode::SRU_MODE:
        write_bit(*this, SPI_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::RO), true);
        break;
    case Operational_Mode::STB_MODE:
        write_bits_sequence(*this, SPI_Regs::CTL0,
                   static_cast<uint32_t>(CTL0_Bits::BDEN), true,
                   static_cast<uint32_t>(CTL0_Bits::BDOEN), true);
        break;
    case Operational_Mode::SRB_MODE:
        write_bit(*this, SPI_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::BDEN), true);
        break;
    case Operational_Mode::SFD_MODE:
    case Operational_Mode::STU_MODE:
    default:
        break;
    }

    // Store the config
    config_ = config;
}

/**
 * @brief Enables or disables the SPI peripheral.
 *
 * This function sets the SPI enable bit in the control register to enable or disable
 * the SPI peripheral based on the provided boolean value.
 *
 * @param enable A boolean value where true enables the SPI peripheral and false disables it.
 */
void SPI::set_enable(bool enable) {
    write_bit(*this, SPI_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SPIEN), enable);
}

/**
 * @brief Enables or disables the NSS output.
 *
 * This function sets the NSSDRV bit in the CTL1 register to enable or
 * disable the NSS output. When enabled, the NSS signal is driven by
 * the SPI hardware, otherwise it is controlled by software.
 *
 * @param enabled Set to true to enable the NSS output, false to disable it.
 */
void SPI::set_nss_output_enable(bool enabled) {
    write_bit(*this, SPI_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::NSSDRV), enabled);
}

/**
 * @brief Sets the NSS signal internally high.
 *
 * This function sets the NSS signal internally high by setting the
 * SWNSS bit in the CTL0 register. When the NSS signal is internally
 * high, the SPI peripheral is enabled and the NSS signal is driven
 * by the SPI hardware. When the NSS signal is internally low, the
 * SPI peripheral is disabled and the NSS signal is controlled by
 * software.
 */
void SPI::nss_internal_high() {
    write_bit(*this, SPI_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SWNSS), true);
}

/**
 * @brief Sets the NSS signal internally low.
 *
 * This function sets the NSS signal internally low by clearing the
 * SWNSS bit in the CTL0 register. When the NSS signal is internally
 * low, the SPI peripheral is disabled and the NSS signal is controlled
 * by software.
 */
void SPI::nss_internal_low() {
    write_bit(*this, SPI_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SWNSS), false);
}

/**
 * @brief Enables or disables DMA for the SPI peripheral.
 *
 * This function sets the DMATEN or DMAREN bit in the CTL1 register to enable or
 * disable DMA for the SPI peripheral. When enabled, the SPI peripheral will use
 * DMA for data transfers.
 *
 * @param dma The DMA direction to enable or disable. Can be either
 *            DMA_Direction::DMA_TX for transmit or DMA_Direction::DMA_RX for
 *            receive.
 * @param enabled Set to true to enable DMA, false to disable it.
 */
void SPI::set_dma_enable(DMA_Direction dma, bool enabled) {
    if (dma == DMA_Direction::DMA_TX) {
        write_bit(*this, SPI_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::DMATEN), enabled);
    } else {
        write_bit(*this, SPI_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::DMAREN), enabled);
    }
}

/**
 * @brief Configures the data frame format for the SPI peripheral.
 *
 * This function sets the data frame format to either 8-bit or 16-bit based on
 * the provided Frame_Format parameter. The setting is applied by writing to
 * the FF16 bit in the CTL0 register. A 16-bit frame format is selected if
 * Frame_Format::FF_16BIT is specified; otherwise, an 8-bit frame format is used.
 *
 * @param frame_format The desired data frame format, either Frame_Format::FF_8BIT
 * or Frame_Format::FF_16BIT.
 */
void SPI::data_frame_format_config(Frame_Format frame_format) {
    write_bit(*this, SPI_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::FF16), frame_format == Frame_Format::FF_16BIT ? true : false);
}

/**
 * @brief Transmits a 16-bit data value over the SPI bus.
 *
 * This function writes the provided 16-bit data value to the DATA register, which
 * initiates the transmission of the data value over the SPI bus.
 *
 * @param data The 16-bit data value to be transmitted.
 */
void SPI::data_transmit(uint16_t data) {
    write_register(*this, SPI_Regs::DATA, static_cast<uint32_t>(data));
}

/**
 * @brief Receives a 16-bit data value from the SPI peripheral.
 *
 * This function reads a 16-bit data value from the DATA register, retrieving the
 * received data value from the SPI bus.
 *
 * @return The 16-bit data value received from the SPI peripheral.
 */
uint16_t SPI::data_receive() {
    return static_cast<uint16_t>(read_register<uint32_t>(*this, SPI_Regs::DATA));
}

/**
 * @brief Configures the SPI peripheral for bidirectional data transfer in either
 *        transmit or receive direction.
 *
 * This function sets the bidirectional transfer mode for the SPI peripheral
 * based on the provided Direction_Mode parameter. When set to
 * Direction_Mode::BIDIRECTIONAL_TRANSMIT, the SPI peripheral is configured for
 * bidirectional transmission. Otherwise, the SPI peripheral is configured for
 * bidirectional reception.
 *
 * @param transfer_direction The direction of the bidirectional transfer, either
 *                            Direction_Mode::BIDIRECTIONAL_TRANSMIT or
 *                            Direction_Mode::BIDIRECTIONAL_RECEIVE.
 */
void SPI::bidirectional_transfer_config(Direction_Mode transfer_direction) {
    if (transfer_direction == Direction_Mode::BIDIRECTIONAL_TRANSMIT) {
        // Set bidirectional tx mode
        write_bits_sequence(*this, SPI_Regs::CTL0,
                   static_cast<uint32_t>(CTL0_Bits::BDOEN), true,
                   static_cast<uint32_t>(CTL0_Bits::BDEN), true);
    } else {
        // Set bidirectional rx mode
        write_bits_sequence(*this, SPI_Regs::CTL0,
                   static_cast<uint32_t>(CTL0_Bits::BDOEN), false,
                   static_cast<uint32_t>(CTL0_Bits::BDEN), true);
    }
}

/**
 * @brief Sets the CRC polynomial for the SPI peripheral.
 *
 * This function sets the polynomial for the SPI peripheral's cyclic redundancy
 * check (CRC) calculation. The CRC polynomial is a 16-bit value, and the
 * provided value is written to the CRCPOLY register. The CRC polynomial is used
 * in combination with the CRC initialization value to calculate the CRC
 * checksum for data transmitted or received over the SPI bus.
 *
 * @param crc_poly The 16-bit CRC polynomial value to be set.
 */
void SPI::set_crc_polynomial(uint16_t crc_poly) {
    write_register(*this, SPI_Regs::CRCPOLY, static_cast<uint32_t>(crc_poly));
}

/**
 * @brief Gets the current CRC polynomial value from the SPI peripheral.
 *
 * This function reads the 16-bit CRC polynomial value from the CRCPOLY register
 * and returns it as a uint16_t.
 *
 * @return The current 16-bit CRC polynomial value.
 */
uint16_t SPI::get_crc_polynomial() {
    return static_cast<uint16_t>(read_register<uint32_t>(*this, SPI_Regs::CRCPOLY));
}

/**
 * @brief Enables or disables the cyclic redundancy check (CRC) calculation for
 *        the SPI peripheral.
 *
 * This function sets the CRCEN bit in the CTL0 register to enable or disable
 * the CRC calculation for data transmitted or received over the SPI bus.
 *
 * @param enabled Set to true to enable the CRC calculation, false to disable it.
 */
void SPI::set_crc_enable(bool enabled) {
    write_bit(*this, SPI_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::CRCEN), enabled);
}

/**
 * @brief Sets the CRCNT bit in the CTL0 register to true, which clears the 
 *        current CRC calculation result and enables the next CRC calculation.
 */
void SPI::set_crc_next() {
    write_bit(*this, SPI_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::CRCNT), true);
}

/**
 * @brief Retrieves the current CRC value from the SPI peripheral.
 *
 * This function returns the CRC value based on the specified CRC direction.
 * If the direction is CRC_Direction::CRC_TX, it retrieves the CRC value for
 * the transmitted data. Otherwise, if the direction is CRC_Direction::CRC_RX,
 * it retrieves the CRC value for the received data.
 *
 * @param crc The direction for which to retrieve the CRC value, either
 *            CRC_Direction::CRC_TX or CRC_Direction::CRC_RX.
 * @return The 16-bit CRC value corresponding to the specified direction.
 */
uint16_t SPI::get_crc(CRC_Direction crc) {
    if (crc == CRC_Direction::CRC_TX) {
        return static_cast<uint16_t>(read_register<uint32_t>(*this, SPI_Regs::TCRC));
    } else {
        return static_cast<uint16_t>(read_register<uint32_t>(*this, SPI_Regs::RCRC));
    }
}

/**
 * @brief Clears the CRC error flag in the SPI peripheral.
 *
 * This function clears the CRCERR bit in the STAT register, which indicates
 * that a CRC error has occurred. The CRC error flag is set when the SPI
 * peripheral detects a mismatch between the calculated CRC value and the
 * received CRC value. This function must be called before attempting to
 * transmit or receive data again.
 */
void SPI::clear_crc_error() {
    write_bit(*this, SPI_Regs::STAT, static_cast<uint32_t>(STAT_Bits::CRCERR), false);
}

/**
 * @brief Enables or disables the NSSP mode for the SPI peripheral.
 *
 * This function sets the NSSP bit in the CTL1 register to enable or disable
 * the NSSP mode for the SPI peripheral. When enabled, the SPI peripheral
 * will use the NSSP pin as the slave select pin for the SPI bus. When disabled,
 * the NSS pin is used as the slave select pin.
 *
 * @param enabled Set to true to enable the NSSP mode, false to disable it.
 */
void SPI::set_nssp_mode_enable(bool enabled) {
    write_bit(*this, SPI_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::NSSP), enabled);
}

/**
 * @brief Enables or disables the quad mode for the SPI peripheral.
 *
 * This function sets the QMOD bit in the QCTL register to enable or disable
 * the quad mode for the SPI peripheral. When enabled, the SPI peripheral will
 * use the quad mode for data transfers. When disabled, the SPI peripheral will
 * use the standard SPI mode for data transfers.
 *
 * @param enabled Set to true to enable the quad mode, false to disable it.
 */
void SPI::set_quad_mode_enable(bool enabled) {
    write_bit(*this, SPI_Regs::QCTL, static_cast<uint32_t>(QCTL_Bits::QMOD), enabled);
}

/**
 * @brief Enables the quad mode for writing data from the SPI peripheral.
 *
 * This function sets the QRD bit in the QCTL register to false, which enables
 * the quad mode for writing data from the SPI peripheral. The quad mode allows
 * for faster data transfers by using four data lines instead of one.
 */
void SPI::quad_write_enable() {
    write_bit(*this, SPI_Regs::QCTL, static_cast<uint32_t>(QCTL_Bits::QRD), false);
}

/**
 * @brief Enables the quad mode for reading data from the SPI peripheral.
 *
 * This function sets the QRD bit in the QCTL register to true, which enables
 * the quad mode for reading data from the SPI peripheral. The quad mode allows
 * for faster data transfers by using four data lines instead of one.
 */
void SPI::quad_read_enable() {
    write_bit(*this, SPI_Regs::QCTL, static_cast<uint32_t>(QCTL_Bits::QRD), true);
}

/**
 * @brief Enables or disables the IO23 output driver for the quad mode.
 *
 * This function sets the IO23_DRV bit in the QCTL register to enable or disable
 * the IO23 output driver for the quad mode. When enabled, the IO23 output driver
 * will drive the IO23 pin for data transfers. When disabled, the IO23 output
 * driver will be in a high-impedance state.
 *
 * @param enabled Set to true to enable the IO23 output driver, false to disable it.
 */
void SPI::set_quad_io23_output_enable(bool enabled) {
    write_bit(*this, SPI_Regs::QCTL, static_cast<uint32_t>(QCTL_Bits::IO23_DRV), enabled);
}

/**
 * @brief Returns the status of a specified flag in the SPI peripheral.
 *
 * This function reads the specified flag from the STAT register and returns
 * its status. If the flag is not applicable to the SPI peripheral, this
 * function returns false.
 *
 * @param[in] flag The status flag to retrieve. Must be a value from the
 *                 Status_Flags enumeration.
 * @return true if the flag is set, false otherwise.
 */
bool SPI::get_flag(Status_Flags flag) {
    return read_bit(*this, SPI_Regs::STAT, static_cast<uint32_t>(flag));
}

/**
 * @brief Retrieves the status of a specified interrupt flag in the SPI peripheral.
 *
 * This function checks both the status and control registers to determine if
 * a specified interrupt flag is set and enabled. It evaluates different interrupt
 * flags such as transmit buffer empty, receive buffer not empty, and various error
 * conditions. If both the status and control bits for the specified interrupt flag
 * are set, the function returns true, indicating that the interrupt condition is
 * active and enabled.
 *
 * @param flag The interrupt flag to check, specified as an Interrupt_Flags enumeration value.
 * @return true if the specified interrupt flag is set and enabled, false otherwise.
 */
bool SPI::get_interrupt_flag(Interrupt_Flags flag) {
    const auto& config = interrupt_flags_config[static_cast<size_t>(flag)];
    bool stat_bit = read_bit_range(*this, config.stat_reg, config.stat_bit) != Clear;
    bool ctl_bit = read_bit_range(*this, config.ctl_reg, config.ctl_bit) != Clear;
    return (stat_bit && ctl_bit);
}

/**
 * @brief Enables or disables a specified interrupt type in the SPI peripheral.
 *
 * This function sets the interrupt enable bit for the specified interrupt type
 * in the SPI peripheral. The function takes an Interrupt_Type enumeration value
 * and a boolean value indicating whether to enable (true) or disable (false) the
 * interrupt type. The function sets the appropriate bit in the control register
 * to enable or disable the interrupt type.
 *
 * @param[in] type The interrupt type to enable or disable, specified as an
 *                 Interrupt_Type enumeration value.
 * @param[in] enabled A boolean value indicating whether to enable (true) or
 *                    disable (false) the interrupt type.
 */
void SPI::set_interrupt_enable(Interrupt_Type type, bool enabled) {
    write_bit(*this, SPI_Regs::CTL1, static_cast<uint32_t>(type), enabled);
}


} // namespace spi
