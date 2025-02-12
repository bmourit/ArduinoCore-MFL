//
// MFL gd32f30x ADC peripheral register access in C++
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

#include "ADC.hpp"
#include "RCU.hpp"

namespace adc {

template <ADC_Base Base>
ADC& get_instance_for_base() {
    static ADC instance(Base);
    return instance;
}

Result<ADC, ADC_Error_Type> ADC::get_instance(ADC_Base Base) {
    switch (Base) {
        case ADC_Base::ADC0_BASE:
            return get_enum_instance<ADC_Base, ADC, ADC_Error_Type>(
                       Base, ADC_Base::ADC0_BASE, get_instance_for_base<ADC_Base::ADC0_BASE>()
                   );
        case ADC_Base::ADC1_BASE:
            return get_enum_instance<ADC_Base, ADC, ADC_Error_Type>(
                       Base, ADC_Base::ADC1_BASE, get_instance_for_base<ADC_Base::ADC1_BASE>()
                   );
        case ADC_Base::ADC2_BASE:
            return get_enum_instance<ADC_Base, ADC, ADC_Error_Type>(
                       Base, ADC_Base::ADC2_BASE, get_instance_for_base<ADC_Base::ADC2_BASE>()
                   );
        case ADC_Base::INVALID:
        default:
            return RETURN_RESULT(ADC, ADC_Error_Type::INVALID_ADC);
    }
}

std::array<bool, static_cast<size_t>(ADC_Base::INVALID)> ADC::clock_enabled_ = {false};

ADC::ADC(ADC_Base Base) :
    base_(Base),
    ADC_pclk_info_(ADC_pclk_index[static_cast<size_t>(Base)]),
    base_address_(ADC_baseAddress[static_cast<size_t>(Base)]),
    prescaler_(get_prescaler_value())
{
    if (!clock_enabled_[static_cast<size_t>(Base)]) {
        RCU_I.set_pclk_enable(ADC_pclk_info_.clock_reg, true);
        RCU_I.set_pclk_reset_enable(ADC_pclk_info_.reset_reg, true);
        RCU_I.set_pclk_reset_enable(ADC_pclk_info_.reset_reg, false);
        clock_enabled_[static_cast<size_t>(Base)] = true;
    }
}

/**
 * @brief Reset the ADC peripheral to its default state.
 *
 * This function will first enable the peripheral clock reset, then disable it.
 * This will cause the ADC peripheral to be reset to its default state.
 */
void ADC::reset() {
    RCU_I.set_pclk_reset_enable(ADC_pclk_info_.reset_reg, true);
    RCU_I.set_pclk_reset_enable(ADC_pclk_info_.reset_reg, false);
}

/**
 * @brief Enable the ADC by setting the ADCON bit in the CTL1 register.
 *
 * This function checks if the ADC is not already enabled by reading the ADCON bit.
 * If it is not enabled, it sets the ADCON bit to true to enable the ADC.
 */
void ADC::enable() {
    if (!read_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ADCON))) {
        write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ADCON), true);
    }
}

/**
 * @brief Disable the ADC by clearing the ADCON bit in the CTL1 register.
 *
 * This function will clear the ADCON bit in the CTL1 register, disabling the ADC.
 */
void ADC::disable() {
    write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ADCON), false);
}

/**
 * @brief Enable or disable the ADC peripheral.
 *
 * This function sets the ADCON bit in the CTL1 register to enable or disable
 * the ADC. If enabling, it first checks if the ADC is not already enabled.
 *
 * @param enable Set to true to enable the ADC, false to disable it.
 */
void ADC::set_enable(bool enable) {
    if (enable) {
        if (!read_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ADCON))) {
            write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ADCON), true);
        }
    } else {
        write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ADCON), false);
    }
}

/**
 * @brief Get the current enabled state of the ADC peripheral.
 *
 * This function will read the ADCON bit in the CTL1 register and return its value.
 *
 * @return true if the ADC is enabled, false if it is not.
 */
bool ADC::is_enabled() {
    return read_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ADCON));
}

/**
 * @brief Start the ADC calibration.
 *
 * This function will first reset the ADC's calibration block, then start the
 * calibration. It will block until the calibration is complete.
 */
void ADC::calibration_enable() {
    write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::RSTCLB), true);
    while (read_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::RSTCLB))) {
    }
    // Start the calibration
    write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::CLB), true);
    while (read_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::CLB))) {
    }
}

/**
 * @brief Enable or disable DMA for the ADC.
 *
 * This function sets the DMA bit in the CTL1 register to enable or disable
 * DMA for the ADC. When enabled, the ADC will use DMA for data transfers.
 *
 * @param enable Set to true to enable DMA, false to disable it.
 */
void ADC::set_dma_enable(bool enable) {
    write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::DMA), enable);
}

/**
 * @brief Enable or disable the temperature and voltage reference.
 *
 * This function sets the TSVREN bit in the CTL1 register to enable or disable
 * the temperature and voltage reference. When enabled, the ADC will use the
 * internal temperature and voltage references.
 *
 * @param enable Set to true to enable the temperature and voltage reference,
 *               false to disable it.
 */
void ADC::set_temperature_voltage_reference_enable(bool enable) {
    write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::TSVREN), enable);
}

/**
 * @brief Sets the resolution of the ADC.
 *
 * This function sets the DRES bits in the OVSAMPCTL register to the given
 * ADC_Resolution value. The available resolutions are:
 * ADC_Resolution::RESOLUTION_6BIT
 * ADC_Resolution::RESOLUTION_8BIT
 * ADC_Resolution::RESOLUTION_10BIT
 * ADC_Resolution::RESOLUTION_12BIT
 *
 * @param resolution The desired ADC resolution.
 */
void ADC::set_resolution(ADC_Resolution resolution) {
    write_bit_range(*this, ADC_Regs::OVSAMPCTL,
                    static_cast<uint32_t>(OVSAMPCTL_Bits::DRES), static_cast<uint32_t>(resolution));
}

/**
 * @brief Configure the discontinuous mode for the ADC channel groups.
 *
 * This function sets the discontinuous mode for either regular or inserted
 * channel groups, or disables it entirely. It configures the ADC control
 * register based on the specified channel group type and length.
 *
 * @param channel_group The type of channel group (regular, inserted, or disable).
 * @param length The number of conversions in the discontinuous group minus one.
 *               Only applicable for the regular channel group.
 */
void ADC::set_group_channel_discontinuous_mode(Channel_Group_Type channel_group, uint8_t length) {
    write_bits_sequence(*this, ADC_Regs::CTL0,
                        static_cast<uint32_t>(CTL0_Bits::DISIC), false,
                        static_cast<uint32_t>(CTL0_Bits::DISRC), false);

    switch (channel_group) {
        case Channel_Group_Type::REGULAR_CHANNEL:
            write_bit_range(*this, ADC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::DISNUM), static_cast<uint32_t>(length - 1U));
            write_bit(*this, ADC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::DISRC), true);
            break;
        case Channel_Group_Type::INSERTED_CHANNEL:
            write_bit(*this, ADC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::DISIC), true);
            break;
        case Channel_Group_Type::CHANNEL_DISCON_DISABLE:
            break;
        case Channel_Group_Type::REGULAR_INSERTED_CHANNEL:
        default:
            break;
    }
}

/**
 * @brief Configure the ADC's sync mode.
 *
 * This function configures the ADC's sync mode for regular and
 * inserted channel groups. The sync mode determines how the ADC
 * starts conversions and when the ADC interrupt is generated for each group.
 *
 * @param mode The desired Sync_Mode enum class value.
 */
void ADC::set_mode(Sync_Mode mode) {
    write_bit_range(*this, ADC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SYNCM), static_cast<uint32_t>(mode));
}

/**
 * @brief Enable or disable specific functional modes for the ADC.
 *
 * This function configures the ADC to enable or disable various operating
 * modes based on the specified `function` parameter. The available
 * functions include scan mode, inserted channel mode, continuous mode,
 * and combinations thereof.
 *
 * @param function The functional mode to configure (e.g., SCAN_MODE,
 *                 INSERTED_CH_MODE, CONTINUOUS_MODE, SCAN_INSERTED,
 *                 SCAN_CONTINUOUS, SCAN_INSERTED_CONTINUOUS or
 *                 INSERTED_CONTINUOUS).
 * @param enable Set to true to enable the specified functional mode,
 *               false to disable it.
 */
void ADC::set_functional_mode(Functional_Mode function, bool enable) {
    switch (function) {
        case Functional_Mode::SCAN_MODE:
            write_bit(*this, ADC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SM), enable);
            break;
        case Functional_Mode::INSERTED_CH_MODE:
            write_bit(*this, ADC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::ICA), enable);
            break;
        case Functional_Mode::CONTINUOUS_MODE:
            write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::CTN), enable);
            break;
        case Functional_Mode::SCAN_INSERTED:
            write_bits_sequence(*this, ADC_Regs::CTL0,
                                static_cast<uint32_t>(CTL0_Bits::SM), enable,
                                static_cast<uint32_t>(CTL0_Bits::ICA), enable);
            break;
        case Functional_Mode::SCAN_CONTINUOUS:
            write_bit(*this, ADC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SM), enable);
            write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::CTN), enable);
            break;
        case Functional_Mode::SCAN_INSERTED_CONTINUOUS:
            write_bits_sequence(*this, ADC_Regs::CTL0,
                                static_cast<uint32_t>(CTL0_Bits::SM), enable,
                                static_cast<uint32_t>(CTL0_Bits::ICA), enable);
            write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::CTN), enable);
            break;
        case Functional_Mode::INSERTED_CONTINUOUS:
            write_bit(*this, ADC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::ICA), enable);
            write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::CTN), enable);
            break;
        default:
            break;
    }
}

/**
 * Sets the alignment of the data output.
 *
 * @param align Aligns data to the left or right of the output word.
 */
void ADC::set_data_alignment(Data_Alignment align) {
    write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::DAL), align == Data_Alignment::DATA_ALIGN_LEFT);
}

/**
 * @brief Sets the length of the conversion sequence for the specified channel group.
 *
 * This function configures the number of conversions in the sequence for either
 * regular or inserted channel groups. It updates the corresponding ADC register
 * to reflect the specified length.
 *
 * @param type The channel group type (either regular or inserted).
 * @param length The number of conversions in the channel sequence.
 *               This should be the number of conversions minus one.
 */
void ADC::set_channel_length(Channel_Group_Type type, uint32_t length) {
    switch (type) {
        case Channel_Group_Type::REGULAR_CHANNEL:
            write_bit_range(*this, ADC_Regs::RSQ0, static_cast<uint32_t>(RSQX_Bits::RL), length - 1U);
            break;
        case Channel_Group_Type::INSERTED_CHANNEL:
            write_bit_range(*this, ADC_Regs::ISQ, static_cast<uint32_t>(ISQ_Bits::IL), length - 1U);
            break;
        default:
            break;
    }
}

/**
 * @brief Sets the regular channel sequence for the given rank, channel, and sampling time.
 *
 * This function configures the specified rank in the regular channel sequence
 * with the given channel and sample time. It updates the ADC's register state
 * to reflect the new configuration.
 *
 * @param rank The rank of the channel to be configured in the regular sequence.
 * @param channel The channel to be configured in the regular sequence.
 * @param sample_time The sample time for the channel in the regular sequence.
 */
void ADC::set_regular_channel_sequence(uint8_t rank, ADC_Channel channel, ADC_Sample_Time sample_time) {
    if (channel == ADC_Channel::INVALID) {
        return;
    }

    const ADC_Regs reg = (rank < 6U) ? ADC_Regs::RSQ2 :
                         (rank < 12U) ? ADC_Regs::RSQ1 :
                         ADC_Regs::RSQ0;

    const uint8_t adjusted_rank = (rank < 6U) ? rank :
                                  (rank < 12U) ? (rank - 6U) :
                                  (rank - 12U);

    uint32_t reg_value = read_register<uint32_t>(*this, reg);
    reg_value &= ~(static_cast<uint32_t>(0x1FU) << (adjusted_rank * 5U));
    reg_value |= (static_cast<uint32_t>(channel) << (adjusted_rank * 5U));
    write_register(*this, reg, reg_value);

    set_sampling_time(channel, sample_time);
}

/**
 * @brief Sets the inserted channel sequence for the given rank, channel, and sampling time.
 *
 * This function configures the specified rank in the inserted channel sequence
 * with the given channel and sample time. It updates the ADC's register state
 * to reflect the new configuration. The inserted sequence is a sequence of
 * channels that are inserted between the regular sequence channels.
 *
 * @param rank The rank of the channel to be configured in the inserted sequence.
 * @param channel The channel to be configured in the inserted sequence.
 * @param sample_time The sample time for the channel in the inserted sequence.
 */
void ADC::set_inserted_channel_sequence(uint8_t rank, ADC_Channel channel, ADC_Sample_Time sample_time) {
    if (channel == ADC_Channel::INVALID) {
        return;
    }

    uint32_t inserted_length = read_bit_range(*this, ADC_Regs::ISQ, static_cast<uint32_t>(ISQ_Bits::IL));
    uint32_t reg = read_register<uint32_t>(*this, ADC_Regs::ISQ);
    reg &= ~(static_cast<uint32_t>(0x1FU) << (((rank + 3U) - inserted_length) * 5U));
    reg |= static_cast<uint8_t>(channel) << (((rank + 3U) - inserted_length) * 5U);
    write_register(*this, ADC_Regs::ISQ, reg);

    set_sampling_time(channel, sample_time);
}

/**
 * @brief Sets the offset of the inserted channel data in the data register.
 *
 * This function sets the offset of the inserted channel data in the data register
 * to the specified value. The offset is the number of bytes to skip before
 * reading the inserted channel data.
 *
 * @param inserted_channel The inserted channel to set the offset for.
 * @param offset The offset of the inserted channel data in the data register.
 */
void ADC::set_inserted_channel_offset(Inserted_Channel inserted_channel, uint16_t offset) {
    uint32_t inserted_length = read_bit_range(*this, ADC_Regs::ISQ, static_cast<uint32_t>(ISQ_Bits::IL));
    uint32_t reg_offset = 3U - (inserted_length - static_cast<uint32_t>(inserted_channel));

    ADC_Regs reg = (reg_offset == 0U) ? ADC_Regs::IOFF0 :
                   (reg_offset == 1U) ? ADC_Regs::IOFF1 :
                   (reg_offset == 2U) ? ADC_Regs::IOFF1 :
                   ADC_Regs::IOFF3;

    write_bit_range(*this, reg, static_cast<uint32_t>(IOFFX_Bits::IOFF), static_cast<uint32_t>(offset));
}

/**
 * @brief Enables or disables the external trigger for the specified channel group.
 *
 * This function sets the external trigger enable bit for the specified channel
 * group. The external trigger is an external event that triggers the ADC
 * conversion. The channel group can be either the regular channel group, the
 * inserted channel group, or both.
 *
 * @param channel_group The channel group to enable or disable the external
 *                      trigger for.
 * @param enable True to enable the external trigger, false to disable it.
 */
void ADC::set_external_trigger_enable(Channel_Group_Type channel_group, bool enable) {
    if (channel_group == Channel_Group_Type::REGULAR_CHANNEL) {
        write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ETERC), enable);
    }
    if (channel_group == Channel_Group_Type::INSERTED_CHANNEL) {
        write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ETEIC), enable);
    }
    if (channel_group == Channel_Group_Type::REGULAR_INSERTED_CHANNEL) {
        write_bits_sequence(*this, ADC_Regs::CTL1,
                            static_cast<uint32_t>(CTL1_Bits::ETERC), enable,
                            static_cast<uint32_t>(CTL1_Bits::ETEIC), enable);
    }
}

/**
 * @brief Sets the external trigger source for the specified channel group.
 *
 * This function sets the external trigger source for the specified channel
 * group. The external trigger source is an external event that triggers the ADC
 * conversion. The channel group can be either the regular channel group, the
 * inserted channel group, or both.
 *
 * @param channel_group The channel group to set the external trigger source for.
 * @param source The external trigger source to set for the specified channel
 *               group.
 */
void ADC::set_external_group_source(Channel_Group_Type channel_group, External_Trigger_Source source) {
    const uint32_t ctl1_bits = (channel_group == Channel_Group_Type::REGULAR_CHANNEL) ?
                               static_cast<uint32_t>(CTL1_Bits::ETSRC) :
                               static_cast<uint32_t>(CTL1_Bits::ETSIC);

    write_bit_ranges(*this, ADC_Regs::CTL1, ctl1_bits, Clear, ctl1_bits, static_cast<uint32_t>(source));
}

/**
 * @brief Triggers a software start for the specified ADC channel group.
 *
 * This function sets the control bits to initiate a software-triggered
 * ADC conversion for the specified channel group. The channel group can
 * be the regular channel, the inserted channel, or both. No action is
 * taken if the channel group is set to disable.
 *
 * @param channel_group The ADC channel group to trigger. Can be one of
 *                      REGULAR_CHANNEL, INSERTED_CHANNEL, or
 *                      REGULAR_INSERTED_CHANNEL. If set to
 *                      CHANNEL_DISCON_DISABLE, no trigger is executed.
 */
void ADC::set_software_trigger_group(Channel_Group_Type channel_group) {
    uint32_t bits = (channel_group == Channel_Group_Type::REGULAR_CHANNEL) ?
                    (1U << static_cast<uint32_t>(CTL1_Bits::SWRCST)) :
                    (channel_group == Channel_Group_Type::INSERTED_CHANNEL) ?
                    (1U << static_cast<uint32_t>(CTL1_Bits::SWICST)) :
                    ((1U << static_cast<uint32_t>(CTL1_Bits::SWRCST)) |
                     (1U << static_cast<uint32_t>(CTL1_Bits::SWICST)));

    write_bits(*this, ADC_Regs::CTL1, bits, true);
}

/**
 * @brief Retrieves the converted data from the regular channel group.
 *
 * This function reads the data register associated with the regular
 * channel group and returns the result of the ADC conversion. It directly
 * accesses the ADC register to obtain the conversion data.
 *
 * @return The 32-bit result of the regular channel group ADC conversion.
 */
uint32_t ADC::get_regular_data() {
    return read_bit_range(*this, ADC_Regs::RDATA, static_cast<uint32_t>(RDATA_Bits::RDATA));
}

/**
 * @brief Retrieves the converted data from a specified inserted channel.
 *
 * This function accesses the ADC data register corresponding to the
 * specified inserted channel and returns the conversion result. If the
 * inserted channel is out of range, it returns 0.
 *
 * @param inserted_channel The inserted channel to retrieve data from.
 * @return The 32-bit result of the ADC conversion for the specified inserted channel.
 */
uint32_t ADC::get_inserted_data(Inserted_Channel inserted_channel) {
    static const ADC_Regs inserted_data_regs[] = {
        ADC_Regs::IDATA0, ADC_Regs::IDATA1, ADC_Regs::IDATA2, ADC_Regs::IDATA3
    };
    if (static_cast<uint32_t>(inserted_channel) >= sizeof(inserted_data_regs) / sizeof(inserted_data_regs[0])) {
        return 0U;
    }
    return read_bit_range(*this, inserted_data_regs[static_cast<size_t>(inserted_channel)], static_cast<uint32_t>(IDATAX_Bits::IDATAN));
}

/**
 * @brief Retrieves the converted data when in synchronization mode.
 *
 * In synchronization mode, the ADC conversion result is always stored in the
 * RDATA register. This function is used to retrieve the conversion result in
 * this mode.
 *
 * @return The 32-bit result of the ADC conversion in synchronization mode.
 */
uint32_t ADC::get_sync_mode_data() {
    return read_register<uint32_t>(*this, ADC_Regs::RDATA);
}

/**
 * @brief Enables the watchdog for a single ADC channel.
 *
 * This function configures the watchdog settings for a specified ADC channel.
 * It updates the control registers to enable the watchdog features. If the
 * specified channel is invalid, the function returns without making any changes.
 *
 * @param channel The ADC channel to enable the watchdog for. Must not be
 *                ADC_Channel::INVALID.
 */
void ADC::single_channel_watchdog_enable(ADC_Channel channel) {
    if (channel == ADC_Channel::INVALID) {
        return;
    }
    write_bits_sequence(*this, ADC_Regs::CTL0,
                        static_cast<uint32_t>(CTL0_Bits::RWDEN), false,
                        static_cast<uint32_t>(CTL0_Bits::IWDEN), false,
                        static_cast<uint32_t>(CTL0_Bits::WDSC), false);
    write_bit_range(*this, ADC_Regs::CTL0,
                    static_cast<uint32_t>(CTL0_Bits::WDCHSEL), static_cast<uint32_t>(channel));
    write_bits_sequence(*this, ADC_Regs::CTL0,
                        static_cast<uint32_t>(CTL0_Bits::RWDEN), true,
                        static_cast<uint32_t>(CTL0_Bits::IWDEN), true,
                        static_cast<uint32_t>(CTL0_Bits::WDSC), true);
}

/**
 * @brief Enables the watchdog for the specified ADC channel group.
 *
 * This function configures the watchdog settings for a specified ADC channel
 * group. It updates the control registers to enable the watchdog features.
 * If the specified channel group is Channel_Group_Type::CHANNEL_DISCON_DISABLE
 * or invalid, the function returns without making any changes.
 *
 * @param channel_group The ADC channel group to enable the watchdog for. Must
 *                      not be Channel_Group_Type::CHANNEL_DISCON_DISABLE.
 */
void ADC::group_channel_watchdog_enable(Channel_Group_Type channel_group) {
    write_bits_sequence(*this, ADC_Regs::CTL0,
                        static_cast<uint32_t>(CTL0_Bits::RWDEN), false,
                        static_cast<uint32_t>(CTL0_Bits::IWDEN), false,
                        static_cast<uint32_t>(CTL0_Bits::WDSC), false);

    switch (channel_group) {
        case Channel_Group_Type::REGULAR_CHANNEL:
            write_bit(*this, ADC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::RWDEN), true);
            break;
        case Channel_Group_Type::INSERTED_CHANNEL:
            write_bit(*this, ADC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::IWDEN), true);
            break;
        case Channel_Group_Type::REGULAR_INSERTED_CHANNEL:
            write_bits_sequence(*this, ADC_Regs::CTL0,
                                static_cast<uint32_t>(CTL0_Bits::RWDEN), true,
                                static_cast<uint32_t>(CTL0_Bits::IWDEN), true);
            break;
        case Channel_Group_Type::CHANNEL_DISCON_DISABLE:
        default:
            break;
    }
}

/**
 * @brief Disables the watchdog function for all ADC channels.
 *
 * This function disables the watchdog feature for all ADC channels. It clears the
 * watchdog enable bits and sets the watchdog channel selection to the clear
 * state. This function is used to disable the watchdog before entering a low
 * power mode or when the watchdog is no longer needed.
 */
void ADC::watchdog_disable() {
    write_bits_sequence(*this, ADC_Regs::CTL0,
                        static_cast<uint32_t>(CTL0_Bits::IWDEN), false,
                        static_cast<uint32_t>(CTL0_Bits::WDSC), false,
                        static_cast<uint32_t>(CTL0_Bits::RWDEN), false);
    write_bit_range(*this, ADC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::WDCHSEL), Clear);
}

/**
 * @brief Sets the watchdog threshold low and high values.
 *
 * This function sets the low and high threshold values for the watchdog feature.
 * The watchdog feature is used to monitor the ADC conversion result and trigger an
 * interrupt if the result is outside of the specified range. The low and high
 * threshold values are used to define this range. The function sets the low and
 * high threshold values by writing the specified values to the WDLT and WDHT
 * registers, respectively.
 *
 * @param low The low threshold value. Must be in the range 0-4095.
 * @param high The high threshold value. Must be in the range 0-4095.
 */
void ADC::set_watchdog_threshold(uint16_t low, uint16_t high) {
    write_bit_range(*this, ADC_Regs::WDLT, static_cast<uint32_t>(WDLT_Bits::WDLT), static_cast<uint32_t>(low));
    write_bit_range(*this, ADC_Regs::WDHT, static_cast<uint32_t>(WDHT_Bits::WDHT), static_cast<uint32_t>(high));
}

/**
 * @brief Configures the ADC oversampling feature.
 *
 * This function configures the ADC oversampling feature. The oversampling feature
 * is used to improve the accuracy of the ADC conversion result. The function sets
 * the oversampling mode, shift value, and ratio value by writing the specified
 * values to the OVSAMPCTL register.
 *
 * @param mode The oversampling mode. Must be one of the Oversampling_Conversion
 *             values.
 * @param shift The oversampling shift value. Must be one of the Oversampling_Shift
 *              values.
 * @param ratio The oversampling ratio value. Must be one of the Oversampling_Ratio
 *              values.
 */
void ADC::set_oversampling_configuration(Oversampling_Conversion mode, Oversampling_Shift shift, Oversampling_Ratio ratio) {
    write_bit(*this, ADC_Regs::OVSAMPCTL,
              static_cast<uint32_t>(OVSAMPCTL_Bits::TOVS), (mode == Oversampling_Conversion::OVERSAMPLING_CONVERT_ONE));
    write_bit_ranges(*this, ADC_Regs::OVSAMPCTL,
                     static_cast<uint32_t>(OVSAMPCTL_Bits::OVSS), static_cast<uint32_t>(shift),
                     static_cast<uint32_t>(OVSAMPCTL_Bits::OVSR), static_cast<uint32_t>(ratio));
}

/**
 * @brief Enables or disables the ADC oversampling feature.
 *
 * This function enables or disables the ADC oversampling feature. When enabled, the
 * ADC oversampling feature is used to improve the accuracy of the ADC conversion
 * result. The function sets the oversampling enable bit in the OVSAMPCTL register.
 *
 * @param enable The state of the ADC oversampling feature. If true, the ADC
 *               oversampling feature is enabled. If false, the ADC oversampling
 *               feature is disabled.
 */
void ADC::set_oversampling_enable(bool enable) {
    write_bit(*this, ADC_Regs::OVSAMPCTL, static_cast<uint32_t>(OVSAMPCTL_Bits::OVSEN), enable);
}

/**
 * @brief Checks the status of a specific ADC flag.
 *
 * This function reads the ADC status register to determine whether a
 * specified status flag is set or not.
 *
 * @param flag The status flag to check. Must be one of the Status_Flags
 *             enumeration values.
 * @return true if the specified flag is set, false otherwise.
 */
bool ADC::get_flag(Status_Flags flag) {
    return read_bit(*this, ADC_Regs::STAT, static_cast<uint32_t>(flag));
}

/**
 * @brief Clears a specific ADC flag.
 *
 * This function clears a specific ADC flag bit in the STAT register.
 *
 * @param flag The status flag to clear. Must be one of the Status_Flags
 *             enumeration values.
 */
void ADC::clear_flag(Status_Flags flag) {
    write_bit(*this, ADC_Regs::STAT, static_cast<uint32_t>(flag), false);
}

/**
 * @brief Checks if a specific ADC interrupt flag is set.
 *
 * This function evaluates whether a specified ADC interrupt flag is active
 * by checking both the status and control registers. It verifies if the
 * interrupt is enabled and the corresponding flag is set.
 *
 * @param flag The interrupt flag to check. Must be one of the Interrupt_Flags
 *             enumeration values.
 * @return true if the specified interrupt flag is set and enabled, false otherwise.
 */
bool ADC::get_interrupt_flag(Interrupt_Flags flag) {
    auto check_flag = [this](CTL0_Bits control_bit, Interrupt_Flags flag) -> bool {
        bool flag_status = read_bit(*this, ADC_Regs::STAT, static_cast<uint32_t>(flag));
        return flag_status && read_bit(*this, ADC_Regs::CTL0, static_cast<uint32_t>(control_bit));
    };

    switch (flag) {
        case Interrupt_Flags::INTR_FLAG_WDE: return check_flag(CTL0_Bits::WDEIE, flag);
        case Interrupt_Flags::INTR_FLAG_EOC: return check_flag(CTL0_Bits::EOCIE, flag);
        case Interrupt_Flags::INTR_FLAG_EOIC: return check_flag(CTL0_Bits::EOICIE, flag);
        default: return false;
    }
}

/**
 * @brief Clears the specified ADC interrupt flag.
 *
 * This function clears the specified interrupt flag by writing to the
 * STAT register. It is used to reset the status of interrupt flags
 * after they have been handled, ensuring no residual flags remain set.
 *
 * @param flag The interrupt flag to clear. Must be one of the
 *             Interrupt_Flags enumeration values.
 */
void ADC::clear_interrupt_flag(Interrupt_Flags flag) {
    write_bit(*this, ADC_Regs::STAT, static_cast<uint32_t>(flag), false);
}

/**
 * @brief Enables or disables a specific ADC interrupt.
 *
 * This function configures the interrupt settings for the ADC by enabling
 * or disabling the specified interrupt type. It modifies the control register
 * to reflect the desired interrupt configuration.
 *
 * @param type The type of interrupt to configure. Must be a value from the
 *             Interrupt_Type enumeration.
 * @param enable Set to true to enable the interrupt or false to disable it.
 */
void ADC::set_interrupt_enable(Interrupt_Type type, bool enable) {
    write_bit(*this, ADC_Regs::CTL0, static_cast<uint32_t>(type), enable);
}

/**
 * @brief Configures the sampling time for a given ADC channel.
 *
 * This function sets the sampling time for a specified ADC channel. The
 * sampling time is specified as a member of the ADC_Sample_Time enumeration.
 *
 * @param channel The ADC channel to configure. Must be a valid channel
 *                from the ADC_Channel enumeration.
 * @param sample_time The desired sampling time for the specified channel.
 *                    Must be a valid member of the ADC_Sample_Time enumeration.
 */
inline void ADC::set_sampling_time(ADC_Channel channel, ADC_Sample_Time sample_time) {
    if (channel == ADC_Channel::INVALID) {
        return;
    }

    const uint32_t chan = static_cast<uint32_t>(channel);
    const uint32_t sampt = static_cast<uint32_t>(sample_time);
    const uint32_t shift = (channel >= ADC_Channel::CHANNEL_10) ? (chan - 10U) * 3U : chan * 3U;
    const ADC_Regs reg = (channel >= ADC_Channel::CHANNEL_10) ? ADC_Regs::SAMPT0 : ADC_Regs::SAMPT1;

    const uint32_t mask = ~(static_cast<uint32_t>(0x7U) << shift);
    uint32_t reg_value = read_register<uint32_t>(*this, reg);
    reg_value = (reg_value & mask) | (sampt << shift);
    write_register(*this, reg, reg_value);

    // Handle temperature sensor channels
    if (channel == ADC_Channel::CHANNEL_16 || channel == ADC_Channel::CHANNEL_17) {
        set_temperature_voltage_reference_enable(true);
        // Calculate delay cycles
        const uint32_t delay_cycles = (RCU_I.get_system_clock() / 100'000U); // Simplified from (1M * 10)
        for (uint32_t i = 0; i < delay_cycles; ++i) {
            __asm__ volatile("nop");
        }
    }
}

///////////////////////////// OPERATIONAL MODE SPECIFIC METHODS /////////////////////////////

/**
 * @brief Configures the ADC for a regular conversion mode.
 *
 * This function sets up the ADC for a regular conversion mode. It does not
 * start the conversion, but rather configures the ADC to be ready for a
 * conversion. The function is used in conjunction with the start_regular_conversion()
 * function to complete a regular conversion.
 *
 * @note This function does not support internal channels.
 */
inline void ADC::setup_regular_conversion() {
    // Make sure flag is cleared
    clear_flag(Status_Flags::FLAG_EOC);
    // Internal channels not supported yet in this function
    write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::TSVREN), false);
    // In single conversion mode length is forced to zero
    write_bit_range(*this, ADC_Regs::RSQ0, static_cast<uint32_t>(RSQX_Bits::RL), Clear);
    // Disable conflicting modes
    write_bits_sequence(*this, ADC_Regs::CTL0,
                        static_cast<uint32_t>(CTL0_Bits::DISRC), false,
                        static_cast<uint32_t>(CTL0_Bits::SM), false);
    write_bit_range(*this, ADC_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SYNCM), Clear);
    write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::CTN), false);
}

/**
 * @brief Clean up after a regular conversion.
 *
 * This function is used to clean up the ADC after a regular conversion. It
 * resets the ADC back to its default state, so that it is ready for another
 * conversion. The function should be called after a regular conversion is
 * complete to ensure the ADC is properly cleaned up.
 */
inline void ADC::cleanup_regular_conversion() {
    write_register(*this, ADC_Regs::RSQ2, Clear);
    write_register(*this, ADC_Regs::RSQ1, Clear);
    write_register(*this, ADC_Regs::RSQ0, Clear);
    write_register(*this, ADC_Regs::SAMPT0, Clear);
    write_register(*this, ADC_Regs::SAMPT1, Clear);
    write_bit_range(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ETSRC), Clear);
    write_bits_sequence(*this, ADC_Regs::CTL1,
                        static_cast<uint32_t>(CTL1_Bits::ETERC), false,
                        static_cast<uint32_t>(CTL1_Bits::SWRCST), false);
}

/**
 * @brief Do a regular single conversion (no interrupt - software trigger only)
 *
 * Channels 0-15 ONLY!! No internal channels!!
 *
 * Handles register setup, starting, and retrieving the converted data.
 * Cleanup is done after the conversion.
 *
 * @param channel The ADC channel to convert
 * @param sample The sample time to use for the channel
 * @param resolution The resolution of the ADC conversion
 * @param calibrate If true, wait for the calibration to finish before starting the conversion
 * @return The converted data
 */
uint32_t ADC::start_regular_single_conversion(ADC_Channel channel, ADC_Sample_Time sample, ADC_Resolution resolution, bool calibrate) {
    // Disable to avoid any accidental conversion
    write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ADCON), false);
    while (read_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ADCON))) {
        // Wait for the ADC to be disabled
    }

    // Setup the ADC
    setup_regular_conversion();
    // Set the channel
    write_bit_range(*this, ADC_Regs::RSQ2, static_cast<uint32_t>(RSQX_Bits::RSQ_0_6_12), static_cast<uint32_t>(channel));
    // Set the sample time
    set_sampling_time(channel, sample);
    // Set resolution
    write_bit_range(*this, ADC_Regs::OVSAMPCTL, static_cast<uint32_t>(OVSAMPCTL_Bits::DRES), static_cast<uint32_t>(resolution));

    // Basic 16bit oversampling
    write_bit_ranges(*this, ADC_Regs::OVSAMPCTL,
                     static_cast<uint32_t>(OVSAMPCTL_Bits::OVSR), static_cast<uint32_t>(Oversampling_Ratio::OVERSAMPLING_RATIO_MUL16),
                     static_cast<uint32_t>(OVSAMPCTL_Bits::OVSS), static_cast<uint32_t>(Oversampling_Shift::OVERSAMPLING_SHIFT_4BIT));
    // Enable oversampling
    write_bits_sequence(*this, ADC_Regs::OVSAMPCTL,
                        static_cast<uint32_t>(OVSAMPCTL_Bits::TOVS), false,
                        static_cast<uint32_t>(OVSAMPCTL_Bits::OVSEN), true);

    // Set alignment
    write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::DAL), false);

    // Enable ADC
    write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ADCON), true);
    while (!read_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ADCON))) {
        // Wait for the ADC to be enabled
    }
    // If calibration is enabled, wait for the calibration to finish
    if (calibrate) {
        calibration_delay();
        calibration_enable();
    }

    // Set trigger and trigger source
    write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ETERC), true);
    write_bit_range(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ETSRC), static_cast<uint32_t>(External_Trigger_Source::ADC0_1_REGULAR_SOFTWARE));
    // Generate software trigger
    write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::SWRCST), true);
    // Wait for the flag to be set
    while (!get_flag(Status_Flags::FLAG_EOC));
    // Get converted data
    uint32_t converted_data = read_bit_range(*this, ADC_Regs::RDATA, static_cast<uint32_t>(RDATA_Bits::RDATA));
    // Clear the flag
    clear_flag(Status_Flags::FLAG_EOC);
    // Cleanup
    cleanup_regular_conversion();
    // Disable ADC after conversion for power saving
    write_bit(*this, ADC_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::ADCON), false);

    return converted_data;
}

/**
 * @brief Applies the required delay before calibration begins
 *
 * The delay is calculated based on the ADC prescaler and the system clock.
 * The ADC prescaler is obtained from the RCU interface.
 */
inline void ADC::calibration_delay() {
    // Hardware requires delay before starting calibration
    if (prescaler_ == 0U) {
        return;
    }

    volatile uint32_t wait_count = ((RCU_I.get_system_clock() /
                                    (RCU_I.get_clock_frequency(rcu::Clock_Frequency::CK_APB2) / prescaler_))
                                    * Calibration_Delay_Cycles);

    while (wait_count != 0) {
        wait_count = wait_count - 1U;
    }
}

/**
 * @brief Gets the current ADC prescaler value.
 *
 * This function retrieves the current ADC prescaler value from the RCU
 * interface and returns it as an unsigned 32-bit integer. The prescaler value
 * determines the division factor applied to the clock signal provided to the
 * ADC module.
 *
 * @return The current ADC prescaler value as an unsigned 32-bit integer.
 */
inline uint32_t ADC::get_prescaler_value() {
    rcu::ADC_Prescaler adc_prescaler = RCU_I.get_adc_prescaler();

    switch (adc_prescaler) {
        case rcu::ADC_Prescaler::CKAPB2_DIV2:
        case rcu::ADC_Prescaler::CKAPB2_DIV2B:
            return 2U;
        case rcu::ADC_Prescaler::CKAPB2_DIV4:
            return 4U;
        case rcu::ADC_Prescaler::CKAPB2_DIV6:
        case rcu::ADC_Prescaler::CKAHB_DIV6:
            return 6U;
        case rcu::ADC_Prescaler::CKAPB2_DIV8:
        case rcu::ADC_Prescaler::CKAPB2_DIV8B:
            return 8U;
        case rcu::ADC_Prescaler::CKAPB2_DIV12:
            return 12U;
        case rcu::ADC_Prescaler::CKAPB2_DIV16:
            return 16U;
        case rcu::ADC_Prescaler::CKAHB_DIV5:
            return 5U;
        case rcu::ADC_Prescaler::CKAHB_DIV10:
            return 10U;
        case rcu::ADC_Prescaler::CKAHB_DIV20:
            return 20U;
        case rcu::ADC_Prescaler::INVALID:
        default:
            return 0U;
    }
}

}  // namespace adc
