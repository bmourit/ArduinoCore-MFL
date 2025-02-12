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

#include "SDIO.hpp"
#include "RCU.hpp"
#include "GPIO.hpp"
#include "Utility.hpp"

namespace sdio {

SDIO& SDIO::get_instance() {
    static SDIO instance;
    return instance;
}

SDIO::SDIO() :
    config_(default_config),
    is_clock_enabled_(false)
{
    if (!is_clock_enabled_) {
        RCU_I.set_pclk_enable(rcu::RCU_PCLK::PCLK_SDIO, true);
        is_clock_enabled_ = true;
    }
}

/**
 * @brief Sets the SDIO peripheral to its reset values and resets the configuration
 * to the default.
 *
 * This function resets the following registers to their reset values:
 *
 * - SDIO_PWRCTL
 * - SDIO_CMDAGMT
 * - SDIO_CMDCTL
 * - SDIO_CLKCTL
 * - SDIO_DATATO
 * - SDIO_DATALEN
 * - SDIO_DATACTL
 * - SDIO_INTC
 * - SDIO_INTEN
 *
 * The configuration is also reset to the default values.
 */
void SDIO::reset() {
    // Set to reset values
    const uint32_t reset_value = 0x00000000U;
    write_register(*this, SDIO_Regs::PWRCTL, reset_value);
    write_register(*this, SDIO_Regs::CMDAGMT, reset_value);
    write_register(*this, SDIO_Regs::CMDCTL, reset_value);
    write_register(*this, SDIO_Regs::CLKCTL, reset_value);
    write_register(*this, SDIO_Regs::DATATO, reset_value);
    write_register(*this, SDIO_Regs::DATALEN, reset_value);
    write_register(*this, SDIO_Regs::DATACTL, reset_value);
    write_register(*this, SDIO_Regs::INTC, reset_value);
    write_register(*this, SDIO_Regs::INTEN, reset_value);

    // Reset to default
    config_ = default_config;
};

/**
 * @brief Initializes the SDIO peripheral with the given configuration.
 *
 * This function sets the clock divider and mode, bus width, bypass mode, power save mode, and hardware clock
 * enable based on the given configuration. It also sets the configuration to the given values.
 *
 * @param config The configuration to use for the SDIO peripheral.
 */
void SDIO::init(SDIO_Config config) {
    config_ = config;
    const uint16_t divider = calculate_clock_divider(config_.desired_clock);
    const bool use_div8 = (divider >= 256U) ? true : false;
    const uint32_t div = (divider >= 256U) ? (divider - 256U) : divider;

    constexpr uint32_t all_bits_mask = (
                                           1U << static_cast<uint32_t>(CLKCTL_Bits::DIV8)) |
                                       (DIV_Mask << DIV_Pos) |
                                       (1U << static_cast<uint32_t>(CLKCTL_Bits::CLKEDGE)) |
                                       (3U << BUSMODE_Pos) |
                                       (1U << static_cast<uint32_t>(CLKCTL_Bits::CLKBYP)) |
                                       (1U << static_cast<uint32_t>(CLKCTL_Bits::CLKPWRSAV)) |
                                       (1U << static_cast<uint32_t>(CLKCTL_Bits::HWCLKEN)
                                       );

    uint32_t clkctl_val = read_register<uint32_t>(*this, SDIO_Regs::CLKCTL) & ~all_bits_mask;

    clkctl_val |= (
                      (use_div8 ? Set : Clear) << static_cast<uint32_t>(CLKCTL_Bits::DIV8)) |
                  (static_cast<uint16_t>(div) << DIV_Pos) |
                  (static_cast<uint32_t>(config_.clock_edge) << static_cast<uint32_t>(CLKCTL_Bits::CLKEDGE)) |
                  (static_cast<uint32_t>(config_.width) << BUSMODE_Pos) |
                  ((config_.enable_bypass ? Set : Clear) << static_cast<uint32_t>(CLKCTL_Bits::CLKBYP)) |
                  ((config_.enable_powersave ? Set : Clear) << static_cast<uint32_t>(CLKCTL_Bits::CLKPWRSAV)) |
                  ((config_.enable_hwclock ? Set : Clear) << static_cast<uint32_t>(CLKCTL_Bits::HWCLKEN)
                  );

    write_register(*this, SDIO_Regs::CLKCTL, clkctl_val);
}

/**
 * @brief Configures the SDIO peripheral clock settings.
 *
 * @param edge The clock edge to use.
 * @param bypass Whether to bypass the clock divider.
 * @param low_power Whether to enable low power mode.
 * @param divider The clock divider value.
 */
void SDIO::interface_clock_configure(Clock_Edge edge, bool bypass, bool low_power, uint16_t divider) {
    const bool use_div8 = (divider >= 256) ? true : false;
    const uint32_t div = (divider >= 256) ? (divider - 256) : divider;

    constexpr uint32_t all_bits_mask = (
                                           (1U << static_cast<uint32_t>(CLKCTL_Bits::DIV8)) |
                                           (DIV_Mask << DIV_Pos) |
                                           (1U << static_cast<uint32_t>(CLKCTL_Bits::CLKEDGE)) |
                                           (1U << static_cast<uint32_t>(CLKCTL_Bits::CLKBYP)) |
                                           (1U << static_cast<uint32_t>(CLKCTL_Bits::CLKPWRSAV))
                                       );

    uint32_t clkctl_val = read_register<uint32_t>(*this, SDIO_Regs::CLKCTL) & ~all_bits_mask;

    clkctl_val |= ((use_div8 ? Set : Clear) << static_cast<uint32_t>(CLKCTL_Bits::DIV8)) |
                  (static_cast<uint16_t>(div) << DIV_Pos) |
                  (static_cast<uint32_t>(edge) << static_cast<uint32_t>(CLKCTL_Bits::CLKEDGE)) |
                  ((bypass ? Set : Clear) << static_cast<uint32_t>(CLKCTL_Bits::CLKBYP)) |
                  ((low_power ? Set : Clear) << static_cast<uint32_t>(CLKCTL_Bits::CLKPWRSAV));

    // store updated values to config_
    config_.clock_edge = edge;
    config_.enable_bypass = bypass;
    config_.enable_powersave = low_power;

    write_register(*this, SDIO_Regs::CLKCTL, clkctl_val);
}

/**
 * @brief Retrieves the current clock divider value for the SDIO peripheral.
 *
 * The clock divider is determined by reading the CLKCTL register, specifically
 * the DIV and DIV8 bits. The final divider value is calculated by adding 2 to
 * the sum of the DIV bit value and an optional 256 if the DIV8 bit is set.
 *
 * @return The clock divider value.
 */
uint32_t SDIO::get_clock_divider() {
    uint32_t divider = read_bit_range(*this, SDIO_Regs::CLKCTL, static_cast<uint32_t>(CLKCTL_Bits::DIV));
    divider += (read_bit(*this, SDIO_Regs::CLKCTL, static_cast<uint32_t>(CLKCTL_Bits::DIV8)) ? 1U * 256U : 0U);
    divider += 2U;
    return divider;
}

/**
 * @brief Enables or disables the hardware clock for the SDIO peripheral.
 *
 * This function sets the HWCLKEN bit in the CLKCTL register to enable or
 * disable the hardware clock for the SDIO peripheral.
 *
 * @param enable Set to true to enable the hardware clock, or false to disable it.
 */
void SDIO::set_hardware_flow_control_enable(bool enable) {
    write_bit(*this, SDIO_Regs::CLKCTL, static_cast<uint32_t>(CLKCTL_Bits::HWCLKEN), enable);
    config_.enable_hwclock = enable;
}

/**
 * @brief Sets the bus width for the SDIO peripheral.
 *
 * This function sets the BUSMODE bits in the CLKCTL register to one of the
 * values from the Bus_Width enumeration to set the bus width for the SDIO
 * peripheral.
 *
 * @param width The bus width to set. Must be a value from the Bus_Width
 *              enumeration.
 */
void SDIO::set_bus_width(Bus_Width width) {
    write_bit_range(*this, SDIO_Regs::CLKCTL, static_cast<uint32_t>(CLKCTL_Bits::BUSMODE), static_cast<uint32_t>(width));
    config_.width = width;
}

/**
 * @brief Sets the power mode for the SDIO peripheral.
 *
 * This function sets the power mode to one of the values from the Power_Control
 * enumeration. The power mode determines the voltage level of the SDIO interface.
 *
 * @param power The power mode to set, must be a value from the Power_Control
 *              enumeration.
 */
void SDIO::set_power_mode(Power_Control power) {
    write_bit_range(*this, SDIO_Regs::PWRCTL, static_cast<uint32_t>(PWRCTL_Bits::PWRCTL), static_cast<uint32_t>(power));
}

/**
 * @brief Retrieves the current power mode of the SDIO peripheral.
 *
 * This function reads the PWRCTL register and returns the current power mode
 * as specified by the Power_Control enumeration. The power mode determines
 * the voltage level of the SDIO interface.
 *
 * @return The current power mode, which is a value from the Power_Control
 *         enumeration.
 */
uint32_t SDIO::get_power_mode() {
    return read_bit_range(*this, SDIO_Regs::PWRCTL, static_cast<uint32_t>(PWRCTL_Bits::PWRCTL));
}

/**
 * @brief Enables or disables the clock for the SDIO peripheral.
 *
 * This function sets the CLKEN bit in the CLKCTL register to enable or
 * disable the clock for the SDIO peripheral.
 *
 * @param enable Set to true to enable the clock, or false to disable it.
 */
void SDIO::set_clock_enable(bool enable) {
    write_bit(*this, SDIO_Regs::CLKCTL, static_cast<uint32_t>(CLKCTL_Bits::CLKEN), enable);
}

/**
 * @brief Sets the block size for the SDIO peripheral.
 *
 * This function sets the BLKSZ bits in the DATACTL register to specify the
 * block size for the SDIO peripheral.
 *
 * @param size The block size to set, must be a value from the Block_Size
 *              enumeration.
 */
void SDIO::set_block_size(Block_Size size) {
    write_bit_range(*this, SDIO_Regs::DATACTL, static_cast<uint32_t>(DATACTL_Bits::BLKSZ), static_cast<uint32_t>(size));
}

/**
 * @brief Configures and sends a command to the SDIO peripheral.
 *
 * This function sets up the command configuration by writing the command
 * argument to the CMDAGMT register and configuring the CMDCTL register
 * with the specified command index, response type, wait type, and an
 * enable flag for the command state machine.
 *
 * @param index The index of the command to be sent.
 * @param argument The argument associated with the command.
 * @param response The expected response type from the command.
 * @param type The wait type to be used while waiting for the response.
 * @param enable A boolean flag to enable or disable the command state machine.
 */
void SDIO::set_command_state_machine_and_send(Command_Index index, uint32_t argument, Command_Response response, Wait_Type type, bool enable) {
    write_register(*this, SDIO_Regs::CMDAGMT, argument);

    constexpr uint32_t all_bits_mask = ((static_cast<uint32_t>(CMDIDX_Mask) << CMDIDX_Pos) |
                                        (static_cast<uint32_t>(CMDRESP_Mask) << CMDRESP_Pos) |
                                        (static_cast<uint32_t>(WAITTYPE_Mask) << WAITTYPE_Pos));

    uint32_t cmdctl_val = read_register<uint32_t>(*this, SDIO_Regs::CMDCTL) & ~all_bits_mask;

    cmdctl_val |= (static_cast<uint8_t>(index) << CMDIDX_Pos) |
                  (static_cast<uint8_t>(response) << CMDRESP_Pos) |
                  (static_cast<uint8_t>(type) << WAITTYPE_Pos) |
                  ((enable ? Set : Clear) << static_cast<uint32_t>(CMDCTL_Bits::CSMEN));

    write_register(*this, SDIO_Regs::CMDCTL, cmdctl_val);
}

/**
 * @brief Configures the command and start state machine for the SDIO peripheral.
 *
 * This function sets the command index, argument, response type, wait type, and enable
 * bit in the CMDCTL register. The command index is set to the index of the command to
 * send, the argument is set to the argument of the command, the response type is set to
 * the type of response expected from the card, the wait type is set to the type of wait
 * to use while waiting for the response, and the enable bit is set to disable the
 * command state machine.
 *
 * @param index The index of the command to send.
 * @param argument The argument of the command.
 * @param response The type of response expected from the card.
 * @param type The type of wait to use while waiting for the response.
 */
void SDIO::set_command_state_machine(Command_Index index, uint32_t argument, Command_Response response, Wait_Type type) {
    write_register(*this, SDIO_Regs::CMDAGMT, argument);

    constexpr uint32_t all_bits_mask = (static_cast<uint32_t>(CMDIDX_Mask) << CMDIDX_Pos) |
                                       (static_cast<uint32_t>(CMDRESP_Mask) << CMDRESP_Pos) |
                                       (static_cast<uint32_t>(WAITTYPE_Mask) << WAITTYPE_Pos);

    uint32_t cmdctl_val = read_register<uint32_t>(*this, SDIO_Regs::CMDCTL) & ~all_bits_mask;

    cmdctl_val |= (static_cast<uint8_t>(index) << CMDIDX_Pos) |
                  (static_cast<uint8_t>(response) << CMDRESP_Pos) |
                  (static_cast<uint8_t>(type) << WAITTYPE_Pos);

    write_register(*this, SDIO_Regs::CMDCTL, cmdctl_val);
}

/**
 * @brief Enables or disables the command state machine for the SDIO peripheral.
 *
 * This function sets the CSMEN bit in the CMDCTL register to enable or
 * disable the command state machine, which is responsible for sending
 * commands to the SDIO card.
 *
 * @param enable Set to true to enable the command state machine, or false
 *               to disable it.
 */
void SDIO::set_command_state_machine_enable(bool enable) {
    write_bit(*this, SDIO_Regs::CMDCTL, static_cast<uint32_t>(CMDCTL_Bits::CSMEN), enable);
}

/**
 * @brief Returns the index of the command that was most recently sent to the SDIO card.
 *
 * This function returns the value stored in the RSPCMDIDX register, which is the
 * index of the most recently sent command.
 *
 * @return The index of the most recently sent command.
 */
uint8_t SDIO::get_command_index() {
    return static_cast<uint8_t>(read_bit_range(*this, SDIO_Regs::RSPCMDIDX, static_cast<uint32_t>(RSPCMDIDX_Bits::RSPCMDIDX)) & 0xFF);
}

/**
 * @brief Retrieves the response from the SDIO card based on the specified response type.
 *
 * This function reads the appropriate response register (RESP0, RESP1, RESP2, or RESP3)
 * corresponding to the provided Response_Type and returns the response value.
 *
 * @param type The response type to retrieve, must be one of the enumerated values
 *             from Response_Type indicating which response register to read.
 * @return The 32-bit response value from the selected response register.
 */
uint32_t SDIO::get_response(Response_Type type) {
    return read_register<uint32_t>(*this,
                                   (type == Response_Type::RESPONSE0) ? SDIO_Regs::RESP0 :
                                   (type == Response_Type::RESPONSE1) ? SDIO_Regs::RESP1 :
                                   (type == Response_Type::RESPONSE2) ? SDIO_Regs::RESP2 : SDIO_Regs::RESP3);
}

/**
 * @brief Configures the data transfer and enables or disables the data state machine.
 *
 * This function sets the timeout and data length for the data transfer and
 * configures the data transfer mode and direction. Additionally, it enables
 * or disables the data state machine, which is responsible for sending or
 * receiving data to or from the SDIO card.
 *
 * @param timeout The timeout value to be used for the data transfer.
 * @param length The length of the data transfer.
 * @param size The block size of the data transfer.
 * @param mode The transfer mode, either block or byte.
 * @param direction The direction of the data transfer, either write or read.
 * @param enable Set to true to enable the data state machine, or false to disable it.
 */
void SDIO::set_data_state_machine_and_send(uint32_t timeout, uint32_t length, Block_Size size,
        Transfer_Mode mode, Transfer_Direction direction, bool enable)
{
    // Write the timeout and data length
    write_register(*this, SDIO_Regs::DATATO, timeout);
    write_bit_range(*this, SDIO_Regs::DATALEN, static_cast<uint32_t>(DATALEN_Bits::DATALEN), length);

    constexpr uint32_t all_bits_mask = (
                                           (BLKSZ_Mask << BLKSZ_Pos) |
                                           (1U << static_cast<uint32_t>(DATACTL_Bits::TRANSMOD)) |
                                           (1U << static_cast<uint32_t>(DATACTL_Bits::DATADIR))
                                       );

    uint32_t datactl_val = read_register<uint32_t>(*this, SDIO_Regs::DATACTL) & ~all_bits_mask;

    datactl_val |= (
                       (static_cast<uint32_t>(size) << BLKSZ_Pos) |
                       (static_cast<uint32_t>(mode) << static_cast<uint32_t>(DATACTL_Bits::TRANSMOD)) |
                       (static_cast<uint32_t>(direction) << static_cast<uint32_t>(DATACTL_Bits::DATADIR)) |
                       ((enable ? Set : Clear) << static_cast<uint32_t>(DATACTL_Bits::DATAEN))
                   );

    write_register(*this, SDIO_Regs::DATACTL, datactl_val);
}

/**
 * @brief Configures the data transfer length and timeout.
 *
 * This function sets the timeout and data length for the data transfer and
 * configures the block size of the data transfer.
 *
 * @param timeout The timeout value to be used for the data transfer.
 * @param length The length of the data transfer.
 * @param size The block size of the data transfer.
 */
void SDIO::set_data_state_machine(uint32_t timeout, uint32_t length, Block_Size size) {
    write_register(*this, SDIO_Regs::DATATO, timeout);
    write_bit_range(*this, SDIO_Regs::DATALEN, static_cast<uint32_t>(DATALEN_Bits::DATALEN), length);
    write_bit_range(*this, SDIO_Regs::DATACTL, static_cast<uint32_t>(DATACTL_Bits::BLKSZ), static_cast<uint32_t>(size));
}

/**
 * @brief Configures the data transfer mode and direction.
 *
 * This function sets the transfer mode and direction bits in the data control
 * register (DATACTL). The transfer mode determines whether the data transfer
 * is a block or byte transfer, and the transfer direction determines whether
 * the data is sent to or from the SDIO card.
 *
 * @param mode The transfer mode, either block or byte.
 * @param direction The direction of the data transfer, either write or read.
 */
void SDIO::set_data_transfer(Transfer_Mode mode, Transfer_Direction direction) {
    constexpr uint32_t all_bits_mask = (1U << static_cast<uint32_t>(DATACTL_Bits::TRANSMOD)) |
                                       (1U << static_cast<uint32_t>(DATACTL_Bits::DATADIR));

    uint32_t datactl_val = read_register<uint32_t>(*this, SDIO_Regs::DATACTL) & ~all_bits_mask;

    datactl_val |= (static_cast<uint32_t>(mode) << static_cast<uint32_t>(DATACTL_Bits::TRANSMOD)) |
                   (static_cast<uint32_t>(direction) << static_cast<uint32_t>(DATACTL_Bits::DATADIR));

    write_register(*this, SDIO_Regs::DATACTL, datactl_val);
}

/**
 * @brief Enables or disables the data state machine.
 *
 * This function enables or disables the data state machine, which controls the
 * data transfer between the SDIO card and the CPU. When enabled, the data
 * state machine will transfer data based on the settings in the data control
 * register (DATACTL). When disabled, the data state machine will not transfer
 * any data.
 *
 * @param enable Set to true to enable the data state machine, false to disable it.
 */
void SDIO::set_data_state_machine_enable(bool enable) {
    write_bit(*this, SDIO_Regs::DATACTL, static_cast<uint32_t>(DATACTL_Bits::DATAEN), enable);
}

/**
 * @brief Reads a single data word from the SDIO FIFO register.
 *
 * This function retrieves a 32-bit data word from the FIFO register
 * of the SDIO peripheral. It is used to read data during an SDIO
 * data transfer operation.
 *
 * @return The 32-bit data word read from the FIFO register.
 */
uint32_t SDIO::read_fifo_word() {
    return read_register<uint32_t>(*this, SDIO_Regs::FIFO);
}

/**
 * @brief Writes a single data word to the SDIO FIFO register.
 *
 * This function writes a 32-bit data word to the FIFO register
 * of the SDIO peripheral. It is used to send data during an SDIO
 * data transfer operation.
 *
 * @param data The 32-bit data word to be written to the FIFO register.
 */
void SDIO::write_fifo_word(uint32_t data) {
    write_register(*this, SDIO_Regs::FIFO, data);
}

/**
 * @brief Retrieves the current data word count in the data transfer.
 *
 * This function reads the DATACNT register to determine the number of
 * data words remaining to be transferred in the current data operation.
 *
 * @return The number of data words left to be transferred.
 */
uint32_t SDIO::get_data_count() {
    return read_bit_range(*this, SDIO_Regs::DATACNT, static_cast<uint32_t>(DATACNT_Bits::DATACNT));
}

/**
 * @brief Retrieves the current count of data words in the FIFO register.
 *
 * This function reads the FIFOCNT register to determine the number of data words
 * currently stored in the FIFO register of the SDIO peripheral.
 *
 * @return The number of data words currently stored in the FIFO register.
 */
uint32_t SDIO::get_fifo_count() {
    return read_bit_range(*this, SDIO_Regs::FIFOCNT, static_cast<uint32_t>(FIFOCNT_Bits::FIFOCNT));
}

/**
 * @brief Enables or disables DMA for the SDIO peripheral.
 *
 * This function sets the DMAEN bit in the data control register (DATACTL) to
 * enable or disable DMA for the SDIO peripheral. When enabled, the SDIO
 * peripheral will use DMA for data transfers during data transfer operations.
 *
 * @param enable Set to true to enable DMA, false to disable it.
 */
void SDIO::set_dma_enable(bool enable) {
    write_bit(*this, SDIO_Regs::DATACTL, static_cast<uint32_t>(DATACTL_Bits::DMAEN), enable);
}

/**
 * @brief Enables or disables the read wait function.
 *
 * This function enables or disables the read wait function, which allows the
 * SDIO peripheral to wait for the SDIO card to be ready before attempting
 * to read data from the card. When enabled, the SDIO peripheral will wait
 * for the SDIO card to be ready before attempting to read data from the
 * card. When disabled, the SDIO peripheral will not wait and will attempt
 * to read data from the card immediately.
 *
 * @param enable Set to true to enable the read wait function, false to disable it.
 */
void SDIO::set_read_wait_enable(bool enable) {
    write_bit(*this, SDIO_Regs::DATACTL, static_cast<uint32_t>(DATACTL_Bits::RWEN), enable);
}

/**
 * @brief Enables or disables the stop read wait function.
 *
 * This function sets the RWSTOP bit in the DATACTL register to enable or
 * disable the stop read wait function for the SDIO peripheral. When enabled,
 * the SDIO peripheral will stop the read wait operation. When disabled,
 * the stop read wait operation will not be activated.
 *
 * @param enable Set to true to enable the stop read wait function, false to disable it.
 */
void SDIO::set_stop_read_wait_enable(bool enable) {
    write_bit(*this, SDIO_Regs::DATACTL, static_cast<uint32_t>(DATACTL_Bits::RWSTOP), enable);
}

/**
 * @brief Sets the read wait type for the SDIO peripheral.
 *
 * This function sets the RWTYPE bit in the DATACTL register to specify the
 * read wait type used by the SDIO peripheral. The read wait type determines
 * the behavior of the SDIO peripheral when waiting for the card to be ready
 * during a read operation.
 *
 * @param type The read wait type to set, specified as a value from the
 *             Read_Wait enumeration.
 */
void SDIO::set_read_wait_type(Read_Wait type) {
    write_bit(*this, SDIO_Regs::DATACTL, static_cast<uint32_t>(DATACTL_Bits::RWTYPE), (type == Read_Wait::CLK));
}

/**
 * @brief Enables or disables the IO operation for the SDIO peripheral.
 *
 * This function sets the IOEN bit in the DATACTL register to enable or
 * disable the IO operation for the SDIO peripheral. When enabled, the SDIO
 * peripheral will perform IO operations as configured. When disabled, the
 * IO operations will be halted.
 *
 * @param enable Set to true to enable IO operation, false to disable it.
 */
void SDIO::set_IO_operation_enable(bool enable) {
    write_bit(*this, SDIO_Regs::DATACTL, static_cast<uint32_t>(DATACTL_Bits::IOEN), enable);
}

/**
 * @brief Enables or disables the suspend operation for the SDIO peripheral during
 *        an IO operation.
 *
 * This function sets the SUSPEND bit in the CMDCTL register to enable or
 * disable the suspend operation during an IO operation. When enabled, the
 * SDIO peripheral will suspend the IO operation if the card is not ready.
 * When disabled, the IO operation will not be suspended.
 *
 * @param enable Set to true to enable the suspend operation, false to disable it.
 */
void SDIO::set_IO_suspend_enable(bool enable) {
    write_bit(*this, SDIO_Regs::CMDCTL, static_cast<uint32_t>(CMDCTL_Bits::SUSPEND), enable);
}

/**
 * @brief Enables or disables the command enable (CE) and the ATA command
 *        enable for the SDIO peripheral.
 *
 * This function sets the CE and ATAEN bits in the CMDCTL register to
 * enable or disable the command enable (CE) and the ATA command enable
 * for the SDIO peripheral. When enabled, the SDIO peripheral will
 * respond to the command enable (CE) and the ATA commands. When
 * disabled, the SDIO peripheral will not respond to the command enable
 * (CE) and the ATA commands.
 *
 * @param enable Set to true to enable the command enable (CE) and the
 *               ATA command enable, false to disable them.
 */
void SDIO::set_CE_ATA_command_enable(bool enable) {
    write_bit(*this, SDIO_Regs::CMDCTL, static_cast<uint32_t>(CMDCTL_Bits::ATAEN), enable);
}

/**
 * @brief Enables or disables the command enable (CE) and the ATA command
 *        interrupt for the SDIO peripheral.
 *
 * This function sets the NINTEN bit in the CMDCTL register to enable or
 * disable the command enable (CE) and the ATA command interrupt for the SDIO
 * peripheral. When enabled, the SDIO peripheral will generate an interrupt
 * when the card is ready to receive a command. When disabled, the SDIO
 * peripheral will not generate an interrupt when the card is ready to receive
 * a command.
 *
 * @param enable Set to true to enable the command enable (CE) and the ATA
 *               command interrupt, false to disable them.
 */
void SDIO::set_CE_ATA_interrupt_enable(bool enable) {
    write_bit(*this, SDIO_Regs::CMDCTL, static_cast<uint32_t>(CMDCTL_Bits::NINTEN), enable);
}

/**
 * @brief Enables or disables the command enable (CE) and the ATA command
 *        completion signal for the SDIO peripheral.
 *
 * This function sets the ENCMDC bit in the CMDCTL register to enable or
 * disable the command enable (CE) and the ATA command completion signal for
 * the SDIO peripheral. When enabled, the SDIO peripheral will assert the
 * command enable (CE) and the ATA command completion signal when the command
 * is completed. When disabled, the SDIO peripheral will not assert the
 * command enable (CE) and the ATA command completion signal when the command
 * is completed.
 *
 * @param enable Set to true to enable the command enable (CE) and the ATA
 *               command completion signal, false to disable them.
 */
void SDIO::set_CE_ATA_command_completion_enable(bool enable) {
    write_bit(*this, SDIO_Regs::CMDCTL, static_cast<uint32_t>(CMDCTL_Bits::ENCMDC), enable);
}

/**
 * @brief Clears the data state machine (DSM) for the SDIO peripheral.
 *
 * This function resets the data state machine (DSM) for the SDIO peripheral
 * by clearing the data length and timeout registers, and setting the data
 * control register (DATACTL) to its default values. The data direction is set
 * to the specified value.
 *
 * @param direction The direction of the data transfer, either
 *                  Transfer_Direction::WRITE or Transfer_Direction::READ.
 */
void SDIO::clear_data_state_machine(Transfer_Direction direction) {
    write_register(*this, SDIO_Regs::DATATO, Clear);
    write_bit_range(*this, SDIO_Regs::DATALEN, static_cast<uint32_t>(DATALEN_Bits::DATALEN), Clear);

    constexpr uint32_t all_bits_mask = (BLKSZ_Mask << BLKSZ_Pos) |
                                       (1U << static_cast<uint32_t>(DATACTL_Bits::TRANSMOD)) |
                                       (1U << static_cast<uint32_t>(DATACTL_Bits::DATADIR));

    uint32_t datactl_val = read_register<uint32_t>(*this, SDIO_Regs::DATACTL) & ~all_bits_mask;

    datactl_val |= (static_cast<uint32_t>(Block_Size::BYTES_1) << BLKSZ_Pos) |
                   (static_cast<uint32_t>(Transfer_Mode::BLOCK) << static_cast<uint32_t>(DATACTL_Bits::TRANSMOD)) |
                   (Clear << static_cast<uint32_t>(DATACTL_Bits::DMAEN)) |
                   (Clear << static_cast<uint32_t>(DATACTL_Bits::DATAEN)) |
                   (static_cast<uint32_t>(direction) << static_cast<uint32_t>(DATACTL_Bits::DATADIR));

    write_register(*this, SDIO_Regs::DATACTL, datactl_val);
}

/**
 * @brief Waits until the SDIO command flags are set.
 *
 * This function waits until the command flags for the SDIO peripheral are set.
 * It returns true if the flags are set, false if the timeout is reached.
 *
 * @return true if the command flags are set, false if the timeout is reached.
 */
bool SDIO::wait_cmd_flags() {
    // Calculate timeout based on system clock
    static const uint32_t timeout_multiplier = 100000U;
    static const uint32_t timeout_divider = 8000U;
    static const uint32_t timeout = (RCU_I.get_system_clock() / timeout_divider) * timeout_multiplier;

    volatile uint32_t count = timeout;
    uint32_t flags = 0U;

    // Flags to wait for
    constexpr uint32_t command_complete_flags =
        (1U << static_cast<uint32_t>(Status_Flags::FLAG_CCRCERR)) |
        (1U << static_cast<uint32_t>(Status_Flags::FLAG_CMDTMOUT)) |
        (1U << static_cast<uint32_t>(Status_Flags::FLAG_CMDRECV));

    constexpr uint32_t command_running_flag = (1U << static_cast<uint32_t>(Status_Flags::FLAG_CMDRUN));

    do {
        if (count == 0U) {
            return false; // Timeout
        } else {
            count = count - 1U;
        }
        flags = read_register<uint32_t>(*this, SDIO_Regs::STAT);
    }
    while (((flags & command_complete_flags) == 0U) || ((flags & command_running_flag) != 0U));

    return true; // Command completed successfully
}

/*bool SDIO::wait_cmd_flags() {
    volatile uint32_t count = 100000U * (RCU_I.get_system_clock() / 8U / 1000U);
    uint32_t flags = 0U;

    do {
        if (count == 0U) {
            return false;
        } else {
            count = count - 1U;
        }
        flags = read_register<uint32_t>(*this, SDIO_Regs::STAT);
    }
    while (((flags & ((1U << static_cast<uint32_t>(Status_Flags::FLAG_CCRCERR)) |
                      (1U << static_cast<uint32_t>(Status_Flags::FLAG_CMDTMOUT)) |
                      (1U << static_cast<uint32_t>(Status_Flags::FLAG_CMDRECV)))) == 0U) ||
          ((flags & (1U << static_cast<uint32_t>(Status_Flags::FLAG_CMDRUN))) != 0U));

    return true;
}*/

/**
 * @brief Checks the status of specific SCR flags.
 *
 * This function reads the status register of the SDIO peripheral and checks
 * for the presence of specific status flags: FLAG_DTCRCERR, FLAG_DTTMOUT,
 * FLAG_RXORE, FLAG_DTBLKEND, and FLAG_STBITE. It returns true if any of
 * these flags are set, indicating an error or event condition within the
 * SDIO module.
 *
 * @return true if any of the specified SCR flags are set, false otherwise.
 */
bool SDIO::check_scr_flags() {
    uint32_t reg_value = read_register<uint32_t>(*this, SDIO_Regs::STAT);

    uint32_t mask = ((1U << static_cast<uint32_t>(Status_Flags::FLAG_RXORE)) |
                     (1U << static_cast<uint32_t>(Status_Flags::FLAG_DTCRCERR)) |
                     (1U << static_cast<uint32_t>(Status_Flags::FLAG_DTTMOUT)) |
                     (1U << static_cast<uint32_t>(Status_Flags::FLAG_DTBLKEND)));

    return (reg_value & mask) != Clear;
}

/**
 * @brief Retrieves the status of the specified flag.
 *
 * This function reads the status register of the SDIO peripheral and returns
 * the value of the specified flag, which can be any of the values in the
 * Status_Flags enumeration.
 *
 * @param flag The status flag to retrieve. Must be a value from the
 *             Status_Flags enumeration.
 * @return true if the flag is set, false otherwise.
 */
bool SDIO::get_flag(Status_Flags flag) {
    return read_bit(*this, SDIO_Regs::STAT, static_cast<uint32_t>(flag));
}

/**
 * @brief Clears the specified flag in the SDIO peripheral.
 *
 * This function clears the specified flag by writing a 1 to the
 * corresponding bit in the INTC register. The flag is cleared
 * regardless of whether or not the interrupt is enabled.
 *
 * @param flag The flag to clear, specified as a Clear_Flags enumeration
 *             value.
 */
void SDIO::clear_flag(Clear_Flags flag) {
    write_register(*this, SDIO_Regs::INTC, (1U << static_cast<uint32_t>(flag)));
}

/**
 * @brief Checks if any of the specified flags are set.
 *
 * This function takes an array of Status_Flags values and the number of elements
 * in the array. It then checks each flag in the array to see if it is set.
 * If any of the flags are set, it returns true. Otherwise, it returns false.
 *
 * @param flags The array of Status_Flags values to check.
 * @param count The number of elements in the flags array.
 * @return true if any of the specified flags are set, false otherwise.
 */
bool SDIO::any_flag_set(const Status_Flags* flags, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        if (get_flag(flags[i])) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Clears all command-related flags in the SDIO peripheral.
 *
 * This function clears all flags related to command transfer, such as
 * CCRCERR, CMDTMOUT, CMDREC, and CMDSEND, regardless of whether or not
 * the interrupts are enabled.
 */
void SDIO::clear_all_cmd_flags() {
    write_register(*this, SDIO_Regs::INTC, clear_command_flags);
};

/**
 * @brief Clears all data-related flags in the SDIO peripheral.
 *
 * This function clears all flags related to data transfer, such as
 * DTCRCERR, DTTMOUT, TXURE, RXORE, DTEND, and DTBLKEND, regardless of
 * whether or not the interrupts are enabled.
 */
void SDIO::clear_all_data_flags() {
    write_register(*this, SDIO_Regs::INTC, clear_data_flags);
};

/**
 * @brief Checks the status of a specified interrupt flag in the SDIO peripheral.
 *
 * This function evaluates whether a given interrupt flag is active by reading
 * the SDIO status register. It returns true if the specified flag is set,
 * indicating that the particular interrupt condition has occurred.
 *
 * @param flag The interrupt flag to check, specified as an Interrupt_Flags
 *             enumeration value.
 * @return true if the specified interrupt flag is set, false otherwise.
 */
bool SDIO::get_interrupt_flag(Interrupt_Flags flag) {
    return read_bit(*this, SDIO_Regs::STAT, static_cast<uint32_t>(flag));
}

/**
 * @brief Clears the specified interrupt flag in the SDIO peripheral.
 *
 * This function clears the specified interrupt flag by writing a 1 to the
 * corresponding bit in the INTC register. The flag is cleared regardless of
 * whether or not the interrupt is enabled.
 *
 * @param flag The flag to clear, specified as a Clear_Flags enumeration
 *             value.
 */
void SDIO::clear_interrupt_flag(Clear_Flags flag) {
    write_register(*this, SDIO_Regs::INTC, (1U << static_cast<uint32_t>(flag)));
}

/**
 * @brief Clears multiple interrupt flags in the SDIO peripheral.
 *
 * This function clears multiple interrupt flags by writing a given bit mask
 * to the INTC register. The bit mask should contain the bits that correspond
 * to the interrupt flags to be cleared.
 *
 * @param bitmask The bit mask to write to the INTC register, indicating which
 *                interrupt flags to clear.
 */
void SDIO::clear_multiple_interrupt_flags(uint32_t bitmask) {
    write_register(*this, SDIO_Regs::INTC, bitmask);
}

/**
 * @brief Enables or disables a specified interrupt type in the SDIO peripheral.
 *
 * This function configures the interrupt settings for the SDIO peripheral by
 * enabling or disabling the specified interrupt type. It modifies the interrupt
 * enable register to reflect the desired configuration.
 *
 * @param type The interrupt type to enable or disable. Must be a value from the
 *             Interrupt_Type enumeration.
 * @param enable Set to true to enable the interrupt, false to disable it.
 */
void SDIO::set_interrupt_enable(Interrupt_Type type, bool enable) {
    write_bit(*this, SDIO_Regs::INTEN, static_cast<uint32_t>(type), enable);
}


} // namespace sdio
