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

#include "I2C.hpp"
#include "RCU.hpp"
#include "GPIO.hpp"

namespace i2c {

template <I2C_Base Base>
I2C& get_instance_for_base() {
    static I2C instance(Base);
    return instance;
}

Result<I2C, I2C_Error_Type> I2C::get_instance(I2C_Base Base) {
    switch (Base) {
        case I2C_Base::I2C0_BASE:
            return get_enum_instance<I2C_Base, I2C, I2C_Error_Type>(
                       Base, I2C_Base::I2C0_BASE, get_instance_for_base<I2C_Base::I2C0_BASE>()
                   );
        case I2C_Base::I2C1_BASE:
            return get_enum_instance<I2C_Base, I2C, I2C_Error_Type>(
                       Base, I2C_Base::I2C1_BASE, get_instance_for_base<I2C_Base::I2C1_BASE>()
                   );
        case I2C_Base::INVALID:
        default:
            return RETURN_RESULT(I2C, I2C_Error_Type::INVALID_I2C);
    }
}

std::array<bool, static_cast<size_t>(I2C_Base::INVALID)> I2C::clock_enabled_ = {false};

I2C::I2C(I2C_Base Base) :
    base_(Base),
    I2C_pclk_info_(I2C_pclk_index[static_cast<size_t>(Base)]),
    base_address_(I2C_baseAddress[static_cast<size_t>(Base)])
{
    if (!clock_enabled_[static_cast<size_t>(Base)]) {
        RCU_I.set_pclk_enable(I2C_pclk_info_.clock_reg, true);
        RCU_I.set_pclk_reset_enable(I2C_pclk_info_.reset_reg, true);
        RCU_I.set_pclk_reset_enable(I2C_pclk_info_.reset_reg, false);
        clock_enabled_[static_cast<size_t>(Base)] = true;
    }
}

/**
 * @brief Resets the I2C peripheral by toggling its peripheral clock reset.
 *
 * This function enables the reset of the I2C peripheral by setting the
 * reset register, then disables the reset to complete the operation.
 */
void I2C::reset() {
    RCU_I.set_pclk_reset_enable(I2C_pclk_info_.reset_reg, true);
    RCU_I.set_pclk_reset_enable(I2C_pclk_info_.reset_reg, false);
}

// Standard mode max: 100 kHz
// Fast mode max: 400 kHz
// Fast mode plus max: 1MHz

/**
 * @brief Sets the clock speed for the I2C peripheral and configures the prescaler duty cycle.
 *
 * @param speed The desired clock speed in Hz. The maximum clock speed is currently 1MHz.
 * @param duty The desired duty cycle for the prescaler to use. The duty cycle is either 2 or 16/9.
 * @return An error code representing the result of the operation.
 *
 * @note The maximum clock speed is currently 1MHz. Attempting to set a higher clock speed will
 *       result in an error. Additionally, the I2C peripheral will be disabled if the clock speed
 *       is set to 0.
 */
I2C_Error_Type I2C::set_clock_speed_duty(uint32_t speed, Duty_Cycle duty) {
    if (!speed) return I2C_Error_Type::INVALID_CLOCK_FREQUENCY;

    const uint32_t apb1_clock = RCU_I.get_clock_frequency(rcu::Clock_Frequency::CK_APB1);
    const uint32_t frequency = custom_min(apb1_clock >> 20, MaximumClockSpeed);

    write_bit_range(*this, I2C_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::I2CCLK), frequency);

    uint32_t clkc = (duty == Duty_Cycle::DTCY_2) ?
                    apb1_clock / (speed * 3U) :
                    apb1_clock / ((speed << 4) + (speed << 3) + speed); // speed * 25 = (speed << 4) + (speed << 3) + speed

    // Standard mode
    if (speed <= 100'000U) {
        const uint32_t rise_time = custom_clamp((apb1_clock >> 20) + 1U, MinimumClockSpeed, MaximumClockSpeed);
        write_register(*this, I2C_Regs::RT, rise_time);
        clkc = custom_max(apb1_clock / (speed << 1), 4);
        write_bit_range(*this, I2C_Regs::CKCFG, static_cast<uint32_t>(CKCFG_Bits::CLKC), clkc);
        return I2C_Error_Type::OK;
    }

    // Fast modes
    const bool is_fast_plus = speed > 400'000U;

    const uint32_t rt_multiplier = is_fast_plus ? 120U : 300U;
    write_register(*this, I2C_Regs::RT, (frequency * rt_multiplier) / 1000U + 1U);

    write_bits_sequence(*this, I2C_Regs::CKCFG,
                        static_cast<uint32_t>(CKCFG_Bits::DTCY), duty == Duty_Cycle::DTCY_16_9 ? true : false,
                        static_cast<uint32_t>(CKCFG_Bits::FAST), true);

    if (!is_fast_plus && !read_bit_range(*this, I2C_Regs::CKCFG, static_cast<uint32_t>(CKCFG_Bits::CLKC))) {
        clkc |= 1U;
    }
    write_bit_range(*this, I2C_Regs::CKCFG, static_cast<uint32_t>(CKCFG_Bits::CLKC), clkc);

    if (is_fast_plus) {
        write_bit(*this, I2C_Regs::FMPCFG, static_cast<uint32_t>(FMPCFG_Bits::FMPEN), true);
    }

    return I2C_Error_Type::OK;
}

/**
 * @brief Configures the I2C address format and operational mode.
 *
 * This function sets the address format (either 7-bit or 10-bit) and the mode
 * (I2C or SMBus) for the I2C peripheral. It updates the control and address
 * registers to reflect these configurations.
 *
 * @param address The device address to be set. This address is masked according
 * to the selected address format.
 * @param format The address format to be used, which can be either 7-bit or 10-bit.
 * @param mode The bus mode to be set, which can be either I2C or SMBus.
 */
void I2C::set_address_format(uint32_t address, Address_Format format, Bus_Mode mode) {
    address &= AddressMask;
    write_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SMBEN), mode == Bus_Mode::SMBUS ? true : false);
    write_bit(*this, I2C_Regs::SADDR0, static_cast<uint32_t>(SADDR0_Bits::ADDFORMAT), format == Address_Format::FORMAT_10BITS ? true : false);
    write_bit_ranges(*this, I2C_Regs::SADDR0,
                     static_cast<uint32_t>((format == Address_Format::FORMAT_10BITS) ? SADDR0_Bits::ADDRESS_10BIT : SADDR0_Bits::ADDRESS_7BIT), address);
}

/**
 * @brief Sets the SMBus mode for the I2C peripheral.
 *
 * This function configures the I2C peripheral to operate in either host or
 * device mode for SMBus transactions. It updates the control register to
 * reflect this configuration.
 *
 * @param type The SMBus mode to be set, which can be either Bus_Type::HOST or
 * Bus_Type::DEVICE.
 */
void I2C::set_smbus_type(Bus_Type type) {
    write_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SMBSEL), type == Bus_Type::HOST ? true : false);
}

/**
 * @brief Enables or disables the acknowledgment feature for the I2C peripheral.
 *
 * This function configures the I2C peripheral to either send or not send an
 * acknowledgment after receiving a byte of data. Enabling acknowledgment is
 * essential for proper communication in I2C protocols, as it ensures that
 * the master acknowledges receipts from the slave devices.
 *
 * @param enable Set to true to enable acknowledgment, false to disable it.
 */
void I2C::set_ack_enable(bool enable) {
    write_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::ACKEN), enable);
}

/**
 * @brief Sets the acknowledgment position for the I2C peripheral.
 *
 * This function configures the position of the acknowledgment in the data
 * stream. The acknowledgment can be either after the current byte or after
 * the next byte. The I2C peripheral sends an acknowledgment after receiving
 * a byte of data. Enabling acknowledgment is essential for proper communication
 * in I2C protocols, as it ensures that the master acknowledges receipts from
 * the slave devices.
 *
 * @param select The acknowledgment position, which can be either ACK_Select::CURRENT
 *               or ACK_Select::NEXT.
 */
void I2C::set_ack_position(ACK_Select select) {
    write_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::POAP), select == ACK_Select::NEXT ? true : false);
}

/**
 * @brief Sets the direction and address for the I2C peripheral.
 *
 * This function configures the transfer direction (transmit or receive)
 * and the target address for the I2C peripheral. For transmit operations,
 * the least significant bit (LSB) of the address is cleared. For receive
 * operations, the LSB is set. The modified address is then written to the
 * data register.
 *
 * @param direction The transfer direction, which can be either
 *                  Transfer_Direction::TRANSMIT or Transfer_Direction::RECEIVE.
 * @param address The 7-bit or 10-bit address of the I2C device. The LSB is
 *                adjusted based on the transfer direction.
 */
void I2C::set_direction_address(Transfer_Direction direction, uint32_t address) {
    address = (address & ~1U) | (direction == Transfer_Direction::RECEIVE);
    write_register(*this, I2C_Regs::DATA, address);
}

/**
 * @brief Configures the dual addressing mode for the I2C peripheral.
 *
 * This function enables or disables the dual addressing mode, allowing the I2C
 * peripheral to respond to two different addresses. When enabled, the second
 * address is configured and stored in the appropriate register.
 *
 * @param address The second device address to be set. This address is masked
 *                according to the Address2Mask.
 * @param enable Set to true to enable dual addressing mode, false to disable it.
 */
void I2C::set_dual_address_enable(uint32_t address, bool enable) {
    if (enable) {
        address &= Address2Mask;
        write_bit_range(*this, I2C_Regs::SADDR1, static_cast<uint32_t>(SADDR1_Bits::ADDRESS2), address);
    }

    write_bit(*this, I2C_Regs::SADDR1, static_cast<uint32_t>(SADDR1_Bits::DUADEN), enable);
}

/**
 * @brief Enables or disables the I2C peripheral.
 *
 * This function sets or clears the I2CEN bit in the control register (CTL0)
 * to enable or disable the I2C peripheral.
 *
 * @param enable Set to true to enable the I2C peripheral, false to disable it.
 */
void I2C::set_enable(bool enable) {
    write_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::I2CEN), enable);
}

/**
 * @brief Generates a start condition for the I2C peripheral.
 *
 * This function sets the START bit in the control register (CTL0) to generate
 * a start condition for the I2C peripheral. This bit is cleared by hardware
 * after the start condition is sent.
 */
void I2C::generate_start_condition() {
    write_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::START), true);
}

/**
 * @brief Checks the state of the START bit in the control register (CTL0)
 *
 * This function returns the current state of the START bit, which is set by
 * the user to generate a start condition for the I2C peripheral. The start
 * condition is cleared by hardware after the start condition is sent.
 *
 * @return The current state of the START bit, either Set (true) or Clear (false)
 */
uint32_t I2C::get_start_condition() {
    return (read_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::START)) == true ? Set : Clear);
}

/**
 * @brief Generates a stop condition for the I2C peripheral.
 *
 * This function sets the STOP bit in the control register (CTL0) to generate
 * a stop condition for the I2C peripheral. This bit is cleared by hardware
 * after the stop condition is sent.
 */
void I2C::generate_stop_condition() {
    write_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::STOP), true);
}

/**
 * @brief Checks the state of the STOP bit in the control register (CTL0).
 *
 * This function returns the current state of the STOP bit, which is set by
 * the user to generate a stop condition for the I2C peripheral. The stop
 * condition is cleared by hardware after the stop condition is sent.
 *
 * @return The current state of the STOP bit, either Set (true) or Clear (false).
 */
uint32_t I2C::get_stop_condition() {
    return (read_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::STOP)) == true ? Set : Clear);
}

/**
 * @brief Transmits a byte of data through the I2C peripheral.
 *
 * This function writes an 8-bit data value to the I2C data register,
 * initiating the transmission of the data byte over the I2C bus.
 *
 * @param data The 8-bit data to be transmitted.
 */
void I2C::transmit_data(uint8_t data) {
    write_bit_range(*this, I2C_Regs::DATA, static_cast<uint32_t>(DATA_Bits::TRB), static_cast<uint32_t>(data));
}

/**
 * @brief Receives a byte of data from the I2C peripheral.
 *
 * This function reads an 8-bit data value from the I2C data register,
 * retrieving the received data byte from the I2C bus.
 *
 * @return The 8-bit data received from the I2C peripheral.
 */
uint8_t I2C::receive_data() {
    return read_bit8_range(*this, I2C_Regs::DATA, static_cast<uint32_t>(DATA_Bits::TRB));
}

/**
 * @brief Enables or disables DMA for the I2C peripheral.
 *
 * This function sets the DMAON bit in the control register (CTL1) to enable
 * or disable DMA for the I2C peripheral. When enabled, the I2C peripheral
 * will use DMA for data transfers.
 *
 * @param enable Set to true to enable DMA, false to disable it.
 */
void I2C::set_dma_enable(bool enable) {
    write_bit(*this, I2C_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::DMAON), enable);
}

/**
 * @brief Sets the DMA transfer end flag for the I2C peripheral.
 *
 * This function writes to the DMALST bit in the control register (CTL1) to
 * indicate whether the current DMA transfer is the last transfer in a sequence.
 * Setting this flag to true marks the transfer as the last one, while setting
 * it to false indicates that more transfers will follow.
 *
 * @param is_end Set to true if the current DMA transfer is the last one in the sequence, false otherwise.
 */
void I2C::set_dma_transfer_end(bool is_end) {
    write_bit(*this, I2C_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::DMALST), is_end);
}

/**
 * @brief Configures the clock stretching behavior for the I2C peripheral.
 *
 * This function enables or disables clock stretching by setting the SS bit
 * in the control register (CTL0). When clock stretching is enabled, the I2C
 * peripheral can hold the clock line (SCL) low to pause the communication
 * until it is ready to proceed.
 *
 * @param stretch Set to Stretch_Low::SCLSTRETCH_ENABLE to enable clock
 *                stretching, or Stretch_Low::SCLSTRETCH_DISABLE to disable it.
 */
void I2C::set_stretch_low(Stretch_Low stretch) {
    write_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SS),
              (stretch == Stretch_Low::SCLSTRETCH_ENABLE) ? false : true);
}

/**
 * @brief Enables or disables the response of the I2C peripheral to a general call
 *        address.
 *
 * This function sets the GCEN bit in the control register (CTL0) to enable
 * or disable the response of the I2C peripheral to a general call address.
 * When enabled, the I2C peripheral will respond to a general call address
 * by sending an acknowledge bit.
 *
 * @param respond Set to true to enable the response to a general call address,
 *                false to disable it.
 */
void I2C::set_general_call_respond(bool respond) {
    write_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::GCEN), respond);
}

/**
 * @brief Enables or disables the software reset for the I2C peripheral.
 *
 * This function sets the SRESET bit in the control register (CTL0) to enable
 * or disable the software reset for the I2C peripheral. When enabled, the
 * I2C peripheral can be reset by software by setting the SRESET bit.
 *
 * @param reset Set to true to enable the software reset, false to disable it.
 */
void I2C::set_software_reset_enable(bool reset) {
    write_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SRESET), reset);
}

/**
 * @brief Enables or disables the automatic calculation of the PEC (Packet Error Check)
 *        for the I2C peripheral.
 *
 * This function sets the PECEN bit in the control register (CTL0) to enable
 * or disable the automatic calculation of the PEC for data transfers.
 *
 * @param enable Set to true to enable the automatic PEC calculation, false to disable it.
 */
void I2C::set_pec_calculate(bool enable) {
    write_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::PECEN), enable);
}

/**
 * @brief Enables or disables the transmission of the PEC (Packet Error Check)
 *        during data transfers for the I2C peripheral.
 *
 * This function sets the PECTRANS bit in the control register (CTL0) to enable
 * or disable the transmission of the PEC during data transfers.
 *
 * @param enable Set to true to enable the transmission of the PEC during data
 *               transfers, false to disable it.
 */
void I2C::set_pec_transfer_enable(bool enable) {
    write_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::PECTRANS), enable);
}

/**
 * @brief Retrieves the current value of the PEC (Packet Error Check) for the last data
 *        transfer.
 *
 * This function reads the PECV bits in the status register (STAT1) to retrieve the
 * current value of the PEC for the last data transfer.
 *
 * @return The current value of the PEC for the last data transfer.
 */
uint8_t I2C::get_pec() {
    return read_bit8_range(*this, I2C_Regs::STAT1, static_cast<uint32_t>(STAT1_Bits::PECV));
}

/**
 * @brief Enables or disables the SMBus alert mode for the I2C peripheral.
 *
 * This function sets the SALT bit in the control register (CTL0) to enable
 * or disable the SMBus alert mode. When the SMBus alert mode is enabled and
 * the slave address is matched, an interrupt is generated. The interrupt flag
 * can be cleared by writing 1 to the SMBALERT bit.
 *
 * @param enable Set to true to enable the SMBus alert mode, false to disable it.
 */
void I2C::set_smbus_alert_enable(bool enable) {
    write_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SALT), enable);
}

/**
 * @brief Enables or disables the ARP (Address Resolution Protocol) for the I2C peripheral
 *        in SMBus mode.
 *
 * This function sets the ARPEN bit in the control register (CTL0) to enable or disable
 * the ARP for the I2C peripheral in SMBus mode. When the ARP is enabled, the slave
 * address is automatically generated and transferred to the bus. The ARP is only
 * valid for SMBus mode and is ignored for standard I2C mode.
 *
 * @param enable Set to true to enable the ARP for the I2C peripheral in SMBus mode,
 *               false to disable it.
 */
void I2C::set_smbus_arp_enable(bool enable) {
    write_bit(*this, I2C_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::ARPEN), enable);
}

/**
 * @brief Retrieves the status of the specified flag.
 *
 * This function retrieves the status of the specified flag. The status flags are
 * defined in the Status_Flags enumeration.
 *
 * @param flag The status flag to retrieve. Must be a value from the
 *             Status_Flags enumeration.
 * @return true if the flag is set, false otherwise.
 */
bool I2C::get_flag(Status_Flags flag) {
    return get_value(flag);
}

/**
 * @brief Clears the specified flag in the I2C peripheral.
 *
 * This function clears a given flag specified by the Clear_Flags enumeration.
 * If the flag is FLAG_ADDSEND, it clears the flag by reading the STAT0 and STAT1
 * registers. For other flags, it clears the flag by writing to the corresponding
 * bit range in the appropriate register.
 *
 * @param flag The flag to clear, specified as a Clear_Flags enumeration value.
 */
void I2C::clear_flag(Clear_Flags flag) {
    if (flag == Clear_Flags::FLAG_ADDSEND) {
        // Read STAT0 and STAT1 to clear
        read_register<uint32_t>(*this, I2C_Regs::STAT0);
        read_register<uint32_t>(*this, I2C_Regs::STAT1);
    } else {
        const auto& info = clear_flag_index[static_cast<size_t>(flag)];
        write_bit_range(*this, info.register_offset, info.bit_info, Clear);
    }
}

/**
 * @brief Retrieves the status of a specified interrupt flag in the I2C peripheral.
 *
 * This function retrieves the status of a given interrupt flag within the I2C
 * peripheral. The interrupt flags are defined in the Interrupt_Flags enumeration.
 *
 * @param flag The interrupt flag to retrieve. Must be a value from the
 *             Interrupt_Flags enumeration.
 * @return true if the flag is set, false otherwise.
 */
bool I2C::get_interrupt_flag(Interrupt_Flags flag) {
    return get_value(flag);
}

/**
 * @brief Clears the specified interrupt flag in the I2C peripheral.
 *
 * This function clears the specified interrupt flag within the I2C peripheral.
 * For the FLAG_SBSEND and FLAG_ADDSEND flags, it clears the flag by reading the
 * specified register(s0. For other flags, it clears the flag by writing to the
 * corresponding bit range in the appropriate register.
 *
 * @param flag The interrupt flag to clear. Must be a value from the
 *             Clear_Flags enumeration.
 */
void I2C::clear_interrupt_flag(Clear_Flags flag) {
    if ((flag == Clear_Flags::FLAG_SBSEND) || (flag == Clear_Flags::FLAG_ADDSEND)) {
        read_register<uint32_t>(*this, I2C_Regs::STAT0);
        if (flag == Clear_Flags::FLAG_ADDSEND) {
            // ADDSEND must also read STAT1 to clear
            read_register<uint32_t>(*this, I2C_Regs::STAT1);
        }
    } else {
        const auto& info = clear_flag_index[static_cast<size_t>(flag)];
        write_bit_range(*this, info.register_offset, info.bit_info, Clear);
    }
}

/**
 * @brief Enables or disables the specified interrupt type for the I2C peripheral.
 *
 * This function enables or disables the specified interrupt type for the I2C
 * peripheral. The interrupt types are defined in the Interrupt_Type enumeration.
 *
 * @param type The interrupt type to enable or disable. Must be a value from the
 *             Interrupt_Type enumeration.
 * @param enable Set to true to enable the interrupt, false to disable it.
 */
void I2C::set_interrupt_enable(Interrupt_Type type, bool enable) {
    const auto& info = interrupt_type_index[static_cast<size_t>(type)];
    write_bit_range(*this, info.register_offset, info.bit_info, enable ? Set : Clear);
}


} // namespace i2c
