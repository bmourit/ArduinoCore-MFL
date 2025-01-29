//
// MFL gd32f30x TIMER peripheral register access in C++
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

#include "TIMER.hpp"
#include "RCU.hpp"

namespace timer {

template <TIMER_Base Base>
TIMER& get_instance_for_base() {
    static TIMER instance(Base);
    return instance;
}

Result<TIMER, TIMER_Error_Type> TIMER::get_instance(TIMER_Base Base) {
    switch (Base) {
    case TIMER_Base::TIMER0_BASE:
        return get_enum_instance<TIMER_Base, TIMER, TIMER_Error_Type>(
                   Base, TIMER_Base::TIMER0_BASE, get_instance_for_base<TIMER_Base::TIMER0_BASE>()
               );
    case TIMER_Base::TIMER1_BASE:
        return get_enum_instance<TIMER_Base, TIMER, TIMER_Error_Type>(
                   Base, TIMER_Base::TIMER1_BASE, get_instance_for_base<TIMER_Base::TIMER1_BASE>()
               );
    case TIMER_Base::TIMER2_BASE:
        return get_enum_instance<TIMER_Base, TIMER, TIMER_Error_Type>(
                   Base, TIMER_Base::TIMER2_BASE, get_instance_for_base<TIMER_Base::TIMER2_BASE>()
               );
    case TIMER_Base::TIMER3_BASE:
        return get_enum_instance<TIMER_Base, TIMER, TIMER_Error_Type>(
                   Base, TIMER_Base::TIMER3_BASE, get_instance_for_base<TIMER_Base::TIMER3_BASE>()
               );
    case TIMER_Base::TIMER4_BASE:
        return get_enum_instance<TIMER_Base, TIMER, TIMER_Error_Type>(
                   Base, TIMER_Base::TIMER4_BASE, get_instance_for_base<TIMER_Base::TIMER4_BASE>()
               );
    case TIMER_Base::TIMER5_BASE:
        return get_enum_instance<TIMER_Base, TIMER, TIMER_Error_Type>(
                   Base, TIMER_Base::TIMER5_BASE, get_instance_for_base<TIMER_Base::TIMER5_BASE>()
               );
    case TIMER_Base::TIMER6_BASE:
        return get_enum_instance<TIMER_Base, TIMER, TIMER_Error_Type>(
                   Base, TIMER_Base::TIMER6_BASE, get_instance_for_base<TIMER_Base::TIMER6_BASE>()
               );
    case TIMER_Base::TIMER7_BASE:
        return get_enum_instance<TIMER_Base, TIMER, TIMER_Error_Type>(
                   Base, TIMER_Base::TIMER7_BASE, get_instance_for_base<TIMER_Base::TIMER7_BASE>()
               );
    case TIMER_Base::INVALID:
    default:
        return RETURN_RESULT(TIMER, TIMER_Error_Type::INVALID_TIMER);
    }
}

std::array<bool, static_cast<size_t>(TIMER_Base::INVALID)> TIMER::clock_enabled_ = {false};

TIMER::TIMER(TIMER_Base Base) :
    base_(Base),
    TIMER_pclk_info_(TIMER_pclk_index[static_cast<size_t>(Base)]),
    base_address_(TIMER_baseAddress[static_cast<size_t>(Base)]),
    config_(default_config),
    break_config_(default_break),
    capture_config_(default_capture),
    compare_config_(default_compare)
{
    if (!clock_enabled_[static_cast<size_t>(Base)]) {
        RCU_I.set_pclk_enable(TIMER_pclk_info_.clock_reg, true);
        RCU_I.set_pclk_reset_enable(TIMER_pclk_info_.reset_reg, true);
        RCU_I.set_pclk_reset_enable(TIMER_pclk_info_.reset_reg, false);
        clock_enabled_[static_cast<size_t>(Base)] = true;
    }
    // Initialize default values
    init();
}

/**
 * @brief Resets the TIMER peripheral by toggling the reset register.
 *
 * This function enables the peripheral clock reset for the TIMER by setting
 * the reset register, then immediately disables the reset to complete the reset
 * operation. This effectively resets all registers of the TIMER peripheral to
 * their default values.
 */
void TIMER::reset() {
    RCU_I.set_pclk_reset_enable(TIMER_pclk_info_.reset_reg, true);
    RCU_I.set_pclk_reset_enable(TIMER_pclk_info_.reset_reg, false);
}

/**
 * @brief Initializes the TIMER peripheral.
 *
 * This function initializes the TIMER peripheral by setting the prescaler, counter
 * direction, alignment, and clock division. It also sets the auto-reload period and
 * repetition counter for advanced timers. The function generates an update event
 * to load prescaler and auto-reload values, and stores the configuration.
 *
 * @param config Configuration structure for the TIMER peripheral.
 */
void TIMER::init(TIMER_Config config) {
    // Configure counter direction, alignment, and clock division
    if (!is_basic_timer()) {
        write_bit(*this, TIMER_Regs::CTL0,
                   static_cast<uint32_t>(CTL0_Bits::DIR), config.counting_direction == Count_Direction::DOWN);
        // Clamp divider to maximum
        if (config.divider > Division_Ratio::DIV4) {
            config.divider = Division_Ratio::DIV4;
        }
        write_bit_ranges(*this, TIMER_Regs::CTL0,
                   static_cast<uint32_t>(CTL0_Bits::CKDIV), static_cast<uint32_t>(config.divider),
                   static_cast<uint32_t>(CTL0_Bits::CAM), static_cast<uint32_t>(config.align));
    }

    // Set auto-reload period
    write_register(*this, TIMER_Regs::CAR, config.period);

    // Set prescaler
    write_register(*this, TIMER_Regs::PSC, config.prescaler);

    // Set repetition counter for advanced timers
    if (is_advanced_timer()) {
        write_register(*this, TIMER_Regs::CREP, config.repetition_count);
    }

    // Generate an update event to load prescaler and auto-reload values
    write_bit(*this, TIMER_Regs::SWEVG, static_cast<uint32_t>(SWEVG_Bits::UPG), true);

    // Store the configuration
    config_ = config;
}

/**
 * @brief Enable the TIMER peripheral.
 *
 * This function enables the TIMER peripheral by setting the CEN bit in the CTL0 register.
 */
void TIMER::enable() {
    write_bit(*this, TIMER_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::CEN), true);
}

/**
 * @brief Disables the TIMER peripheral.
 *
 * This function disables the TIMER peripheral by clearing the CEN bit in the CTL0 register.
 */
void TIMER::disable() {
    write_bit(*this, TIMER_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::CEN), false);
}

/**
 * @brief Enable or disable the TIMER peripheral.
 *
 * This function sets or clears the CEN bit in the CTL0 register to control the
 * enable state of the TIMER peripheral.
 *
 * @param[in] enable Enable or disable the TIMER peripheral.
 */
void TIMER::set_enable(bool enable) {
    write_bit(*this, TIMER_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::CEN), enable);
}

/**
 * @brief Enables the update event generation.
 *
 * This function sets the UPDIS bit in the CTL0 register to false, which enables
 * the update event generation. The update event is generated when the counter
 * reaches the auto-reload value or when the counter is cleared.
 */
void TIMER::update_event_enable() {
    write_bit(*this, TIMER_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::UPDIS), false);
}

/**
 * @brief Disables the update event generation.
 *
 * This function sets the UPDIS bit in the CTL0 register to true, which disables
 * the update event generation. The update event is not generated when the
 * counter reaches the auto-reload value or when the counter is cleared.
 */
void TIMER::update_event_disable() {
    write_bit(*this, TIMER_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::UPDIS), true);
}

/**
 * @brief Enable or disable the update event generation.
 *
 * This function sets or clears the UPDIS bit in the CTL0 register to control
 * the enable state of the update event generation. The update event is
 * generated when the counter reaches the auto-reload value or when the counter
 * is cleared.
 *
 * @param[in] enable Enable or disable the update event generation.
 */
void TIMER::set_update_event_enable(bool enable) {
    write_bit(*this, TIMER_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::UPDIS), enable);
}

/**
 * @brief Generates a software event to trigger the TIMER peripheral.
 *
 * This function sets the specified event source bit in the SWEVG register to
 * generate a software event to trigger the TIMER peripheral. The event source
 * can be either the update event (EVENT_SRC_UPG) or the channel X event
 * (EVENT_SRC_CCX).
 *
 * @param[in] event The event source to be generated.
 */
void TIMER::generate_software_event(Event_Source event) {
    write_bit(*this, TIMER_Regs::SWEVG, static_cast<uint32_t>(event), true);
}

/**
 * @brief Sets the update source for the TIMER peripheral.
 *
 * This function sets the update source for the TIMER peripheral to either the
 * regular source (Update_Source::REGULAR_SOURCE) or the output compare source
 * (Update_Source::OUTPUT_COMPARE_SOURCE). The update event is generated when
 * the counter reaches the auto-reload value or when the counter is cleared.
 *
 * @param[in] source The update source to be set.
 */
void TIMER::set_update_source(Update_Source source) {
    write_bit(*this, TIMER_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::UPS), (source == Update_Source::REGULAR_SOURCE));
}

/**
 * @brief Sets the counter direction to count up.
 *
 * This function sets the counter direction bit (DIR) in the CTL0 register to
 * false, which causes the counter to count up.
 */
void TIMER::count_direction_up() {
    write_bit(*this, TIMER_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::DIR), false);
}

/**
 * @brief Sets the counter direction to count down.
 *
 * This function sets the counter direction bit (DIR) in the CTL0 register to
 * true, which causes the counter to count down.
 */
void TIMER::count_direction_down() {
    write_bit(*this, TIMER_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::DIR), true);
}

/**
 * @brief Sets the counter alignment to the specified value.
 *
 * This function sets the counter alignment mode bit (CAM) in the CTL0 register
 * to the specified value. The counter alignment mode is used to control the
 * alignment of the capture/compare mode.
 *
 * @param[in] alignment The counter alignment mode to be set.
 */
void TIMER::counter_alignment(Center_Align alignment) {
    write_bit_range(*this, TIMER_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::CAM), static_cast<uint32_t>(alignment));
}

/**
 * @brief Sets the current counter value of the TIMER peripheral.
 *
 * This function sets the current counter value of the TIMER peripheral. The
 * counter value is a 16-bit unsigned integer.
 *
 * @param[in] counter The counter value to be set.
 */
void TIMER::set_counter_value(uint16_t counter) {
    write_bit_range(*this, TIMER_Regs::CNT, static_cast<uint32_t>(CNT_Bits::CNT), static_cast<uint32_t>(counter));
}

/**
 * @brief Gets the current counter value of the TIMER peripheral.
 *
 * This function gets the current counter value of the TIMER peripheral. The
 * counter value is a 16-bit unsigned integer.
 *
 * @return The current counter value of the TIMER peripheral.
 */
uint32_t TIMER::get_counter() {
    return read_bit_range(*this, TIMER_Regs::CNT, static_cast<uint32_t>(CNT_Bits::CNT));
}

/**
 * @brief Sets the prescaler reload value for the TIMER peripheral.
 *
 * This function sets the prescaler reload value for the TIMER peripheral by
 * updating the prescaler register. The prescaler value is a 16-bit unsigned
 * integer. The function also sets the reload source to either the regular
 * source (PSC_Reload::REGULAR_RELOAD) or the output compare source
 * (PSC_Reload::OUTPUT_COMPARE_RELOAD). The reload event is generated when the
 * counter reaches the auto-reload value or when the counter is cleared.
 *
 * @param[in] prescaler The prescaler value to be set.
 * @param[in] reload The reload source to be set.
 */
void TIMER::set_prescaler_reload(uint16_t prescaler, PSC_Reload reload) {
    write_bit_range(*this, TIMER_Regs::PSC, static_cast<uint32_t>(PSC_Bits::PSC), static_cast<uint32_t>(prescaler));

    if (reload == PSC_Reload::RELOAD_NOW) {
        write_bit(*this, TIMER_Regs::SWEVG, static_cast<uint32_t>(SWEVG_Bits::UPG), true);
    }
}

/**
 * @brief Retrieves the current prescaler value of the TIMER peripheral.
 *
 * This function reads the prescaler register and returns the current prescaler
 * value as a 16-bit unsigned integer.
 *
 * @return The current prescaler value of the TIMER peripheral.
 */
uint16_t TIMER::get_prescaler() {
    return static_cast<uint16_t>(read_bit_range(*this, TIMER_Regs::PSC, static_cast<uint32_t>(PSC_Bits::PSC)));
}

/**
 * @brief Sets the auto-reload shadow enable bit for the TIMER peripheral.
 *
 * This function sets the auto-reload shadow enable bit (ARSE) in the CTL0
 * register to the specified value. When the auto-reload shadow enable bit is
 * set, the current auto-reload value is loaded from the shadow register.
 *
 * @param[in] enable The auto-reload shadow enable bit to be set.
 */
void TIMER::set_auto_reload_shadow_enable(bool enable) {
    write_bit(*this, TIMER_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::ARSE), enable);
}

/**
 * @brief Sets the auto-reload value for the TIMER peripheral.
 *
 * This function sets the auto-reload value (CARL) in the CAR register to the
 * specified value. The auto-reload value is the value that the counter is
 * reloaded with when the counter reaches zero.
 *
 * @param[in] auto_reload The auto-reload value to be set.
 */
void TIMER::set_auto_reload(uint16_t auto_reload) {
    write_bit_range(*this, TIMER_Regs::CAR, static_cast<uint32_t>(CAR_Bits::CARL), static_cast<uint32_t>(auto_reload));
}

/**
 * @brief Retrieves the current auto-reload value of the TIMER peripheral.
 *
 * This function reads the current auto-reload value (CARL) from the CAR
 * register and returns it as a 16-bit unsigned integer.
 *
 * @return The current auto-reload value of the TIMER peripheral.
 */
uint16_t TIMER::get_auto_reload() {
    return read_bit_range(*this, TIMER_Regs::CAR, static_cast<uint32_t>(CAR_Bits::CARL));
}

/**
 * @brief Configures the repetition counter for the TIMER peripheral.
 *
 * This function sets the repetition counter register (CREP) to the specified
 * value. The repetition counter determines how many times the auto-reload
 * value is reloaded when the counter reaches zero.
 *
 * @param[in] repeat The repetition counter value to be set.
 */
void TIMER::repetition_value_config(uint16_t repeat) {
    write_bit_range(*this, TIMER_Regs::CREP, static_cast<uint32_t>(CREP_Bits::CREP), static_cast<uint32_t>(repeat));
}

/**
 * @brief Sets the pulse mode for the TIMER peripheral.
 *
 * This function sets the pulse mode (SPM) bit in the CTL0 register to the
 * specified value. If the SPM bit is set to 1, the auto-reload value is
 * reloaded only once when the counter reaches zero. If the SPM bit is set
 * to 0, the auto-reload value is reloaded every time the counter reaches
 * zero.
 *
 * @param[in] pulse The pulse mode to be set. Pass Pulse_Mode::SINGLE_PULSE
 *                  to set the SPM bit to 1, or Pulse_Mode::REPEATED_PULSE
 *                  to set the SPM bit to 0.
 */
void TIMER::set_pulse_mode(Pulse_Mode pulse) {
    write_bit(*this, TIMER_Regs::CTL0, static_cast<uint32_t>(CTL0_Bits::SPM), (pulse == Pulse_Mode::SINGLE_PULSE));
}

/**
 * @brief Enables the DMA request for the specified DMA channel.
 *
 * This function enables the DMA request for the specified DMA channel by
 * setting the corresponding bit in the DMAINTEN register to 1. When the
 * DMA request is enabled and the DMA channel is configured to request
 * interrupts, an interrupt will be generated every time a DMA transfer
 * is completed.
 *
 * @param[in] dma The DMA channel to be enabled. The DMA channel can be
 *                one of the following: DMA_UP, DMA_CH0, DMA_CH1, DMA_CH2,
 *                DMA_CH3, DMA_CMT, DMA_TRG, or DMA_BRK.
 */
void TIMER::dma_enable(DMA_Select dma) {
    write_bit(*this, TIMER_Regs::DMAINTEN, static_cast<uint32_t>(dma), true);
}

/**
 * @brief Disables the DMA request for the specified DMA channel.
 *
 * This function disables the DMA request for the specified DMA channel by
 * clearing the corresponding bit in the DMAINTEN register to 0. When the DMA
 * request is disabled, no interrupts will be generated for the specified DMA
 * channel.
 *
 * @param[in] dma The DMA channel to be disabled. The DMA channel can be
 *                one of the following: DMA_UP, DMA_CH0, DMA_CH1, DMA_CH2,
 *                DMA_CH3, DMA_CMT, DMA_TRG, or DMA_BRK.
 */
void TIMER::dma_disable(DMA_Select dma) {
    write_bit(*this, TIMER_Regs::DMAINTEN, static_cast<uint32_t>(dma), false);
}

/**
 * @brief Enables or disables the DMA request for the specified DMA channel.
 *
 * This function sets the specified bit in the DMAINTEN register to the
 * specified value. When the DMA request is enabled and the DMA channel is
 * configured to request interrupts, an interrupt will be generated every
 * time a DMA transfer is completed.
 *
 * @param[in] dma The DMA channel to be enabled or disabled. The DMA channel
 *                can be one of the following: DMA_UP, DMA_CH0, DMA_CH1,
 *                DMA_CH2, DMA_CH3, DMA_CMT, DMA_TRG, or DMA_BRK.
 * @param[in] enable Set to true to enable the DMA request, false to disable
 *                   it.
 */
void TIMER::set_dma_enable(DMA_Select dma, bool enable) {
    write_bit(*this, TIMER_Regs::DMAINTEN, static_cast<uint32_t>(dma), enable);
}

/**
 * @brief Configures the DMA request source for the TIMER peripheral.
 *
 * This function sets the DMA request source bit (DMAS) in the CTL1 register
 * to the specified value. If the DMAS bit is set to 1, the DMA request is
 * generated upon an update event. If the DMAS bit is set to 0, the DMA
 * request is generated upon a channel event.
 *
 * @param[in] request The DMA request source to be set. Pass DMA_Request::UPDATE_EVENT
 *                    to generate the DMA request upon an update event, or
 *                    DMA_Request::CHANNEL_EVENT to generate the DMA request
 *                    upon a channel event.
 */
void TIMER::set_dma_request_source(DMA_Request request) {
    write_bit(*this, TIMER_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::DMAS), (request == DMA_Request::UPDATE_EVENT));
}

/**
 * @brief Configures the DMA transfer settings for the TIMER peripheral.
 *
 * This function sets the DMATA and DMATC bits in the DMACFG register to the
 * specified values. The DMATA bit specifies the DMA transfer address, and the
 * DMATC bit specifies the DMA transfer count (burst length). This function sets
 * the specified bits in the DMACFG register to the specified values. If the
 * DMATA bit is set to 0, the DMA transfer address is set to the default value
 * of DMATA_CTL0. If the DMATC bit is set to 0, the DMA transfer count is set
 * to the default value of DMATC_TRANSFER_COUNT1.
 *
 * @param[in] address The DMA transfer address to be set. The DMA transfer
 *                    address can be one of the following: DMATA_CTL0, DMATA_CTL1,
 *                    DMATA_SMCFG, DMATA_DMAINTEN, DMATA_INTF, DMATA_SWEVG, DMATA_CHCTL0,
 *                    DMATA_CHCTL1, DMATA_CHCTL2, DMATA_CNT, DMATA_PSC, DMATA_CAR, DMATA_CREP,
 *                    DMATA_CH0CV, DMATA_CH1CV, DMATA_CH2CV, DMATA_CH3CV, DMATA_CCHP, DMATA_DMACFG,
 *                    or DMATA_DMATB.
 * @param[in] length The DMA transfer count (burst length) to be set. The DMA
 *                   transfer count can be one of the following: DMATC_TRANSFER_COUNT1,
 *                   DMATC_TRANSFER_COUNT2, DMATC_TRANSFER_COUNT3, DMATC_TRANSFER_COUNT4,
 *                   DMATC_TRANSFER_COUNT5, DMATC_TRANSFER_COUNT6, DMATC_TRANSFER_COUNT7,
 *                   DMATC_TRANSFER_COUNT8, DMATC_TRANSFER_COUNT9, DMATC_TRANSFER_COUNT10,
 *                   DMATC_TRANSFER_COUNT11, DMATC_TRANSFER_COUNT12, DMATC_TRANSFER_COUNT13,
 *                   DMATC_TRANSFER_COUNT14, DMATC_TRANSFER_COUNT15, DMATC_TRANSFER_COUNT16,
 *                   DMATC_TRANSFER_COUNT17, or DMATC_TRANSFER_COUNT18.
 */
void TIMER::dma_transfer_config(DMA_Transfer_Address address, DMA_Burst_Length length) {
    write_bit_ranges(*this, TIMER_Regs::DMACFG,
               static_cast<uint32_t>(DMACFG_Bits::DMATA), static_cast<uint32_t>(address),
               static_cast<uint32_t>(DMACFG_Bits::DMATC), static_cast<uint32_t>(length));
}

/**
 * @brief Configures the break input for the TIMER peripheral.
 *
 * This function sets the break input configuration bits in the CCHP register
 * to the specified values. The break input configuration includes the break
 * input protection, dead time, break enable, break polarity, output auto enable,
 * reset on stop (ROS), and input on stop (IOS) states.
 *
 * @param[in] break_config The break input configuration to be set. The break
 *                         input configuration includes the break input protection,
 *                         dead time, break enable, break polarity, output auto enable,
 *                         reset on stop (ROS), and input on stop (IOS) states.
 */
void TIMER::break_init(TIMER_Break break_config) {
    write_bit_ranges(*this, TIMER_Regs::CCHP,
               static_cast<uint32_t>(CCHP_Bits::PROT), static_cast<uint32_t>(break_config.protection),
               static_cast<uint32_t>(CCHP_Bits::DTCFG), static_cast<uint32_t>(break_config.dead_time));
    write_bits_sequence(*this, TIMER_Regs::CCHP,
               static_cast<uint32_t>(CCHP_Bits::BRKEN), (break_config.break_state == Break_Input::BREAK_ENABLE),
               static_cast<uint32_t>(CCHP_Bits::BRKP), (break_config.break_polarity == Break_Polarity::BREAK_HIGH),
               static_cast<uint32_t>(CCHP_Bits::OAEN), (break_config.output_auto_state == Output_Auto::OUTPUT_AUTO_ENABLE),
               static_cast<uint32_t>(CCHP_Bits::ROS), (break_config.ros_state == ROS_State::ROS_ENABLE),
               static_cast<uint32_t>(CCHP_Bits::IOS), (break_config.ios_state == IOS_State::IOS_ENABLE));

    // Store the config
    break_config_ = break_config;
}

/**
 * @brief Enables the break input.
 *
 * This function sets the break enable bit in the CCHP register to true, enabling
 * the break input. The break input is used to detect external events such as
 * overcurrent, overvoltage, or encoder index events.
 */
void TIMER::break_enable() {
    write_bit(*this, TIMER_Regs::CCHP, static_cast<uint32_t>(CCHP_Bits::BRKEN), true);
}

/**
 * @brief Disables the break input.
 *
 * This function sets the break enable bit in the CCHP register to false, disabling
 * the break input. The break input is used to detect external events such as
 * overcurrent, overvoltage, or encoder index events.
 */
void TIMER::break_disable() {
    write_bit(*this, TIMER_Regs::CCHP, static_cast<uint32_t>(CCHP_Bits::BRKEN), false);
}

/**
 * @brief Sets the break enable state for the TIMER peripheral.
 *
 * This function sets the break enable bit in the CCHP register to the
 * specified value. The break input is used to detect external events such as
 * overcurrent, overvoltage, or encoder index events.
 *
 * @param[in] enable Set to true to enable the break input or false to disable
 *                   it.
 */
void TIMER::set_break_enable(bool enable) {
    write_bit(*this, TIMER_Regs::CCHP, static_cast<uint32_t>(CCHP_Bits::BRKEN), enable);
}

/**
 * @brief Enables the output auto mode.
 *
 * This function sets the output auto enable bit in the CCHP register to true,
 * enabling the output auto mode. In this mode, the output state is
 * automatically changed when the timer counter matches the compare value.
 */
void TIMER::output_auto_enable() {
    write_bit(*this, TIMER_Regs::CCHP, static_cast<uint32_t>(CCHP_Bits::OAEN), true);
}

/**
 * @brief Disables the output auto mode.
 *
 * This function sets the output auto enable bit in the CCHP register to false,
 * disabling the output auto mode. In this mode, the output state is
 * automatically changed when the timer counter matches the compare value.
 */
void TIMER::output_auto_disable() {
    write_bit(*this, TIMER_Regs::CCHP, static_cast<uint32_t>(CCHP_Bits::OAEN), false);
}

/**
 * @brief Enables or disables the output auto mode.
 *
 * This function sets the output auto enable bit in the CCHP register to the
 * specified value. In this mode, the output state is automatically changed
 * when the timer counter matches the compare value.
 *
 * @param enable Set to true to enable the output auto mode or false to
 *               disable it.
 */
void TIMER::set_output_auto_enable(bool enable) {
    write_bit(*this, TIMER_Regs::CCHP, static_cast<uint32_t>(CCHP_Bits::OAEN), enable);
}

/**
 * @brief Enables or disables the primary output.
 *
 * This function sets the primary output enable bit in the CCHP register to the
 * specified value. If the primary output is enabled, the primary output pin
 * will be driven according to the timer channel output level. If the primary
 * output is disabled, the primary output pin will not be driven.
 *
 * @param enable Set to true to enable the primary output or false to
 *               disable it.
 */
void TIMER::set_primary_output_enable(bool enable) {
    write_bit(*this, TIMER_Regs::CCHP, static_cast<uint32_t>(CCHP_Bits::POEN), enable);
}

/**
 * @brief Enables or disables the channel shadow feature.
 *
 * If the channel shadow feature is enabled, the timer channel compare
 * registers are updated with the shadow value when the counter matches the
 * auto reload value. If the channel shadow feature is disabled, the timer
 * channel compare registers are updated with the value of the CCRx register
 * directly.
 *
 * @param enable Set to true to enable the channel shadow feature or false to
 *               disable it.
 */
void TIMER::set_channel_shadow_enable(bool enable) {
    write_bit(*this, TIMER_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::CCSE), enable);
}

/**
 * @brief Configures the channel shadow update method.
 *
 * This function sets the CCUC bit in the CTL1 register to the specified value.
 * If the CCUC bit is set, the timer channel compare registers are updated with
 * the shadow value when the up-count trigger occurs. If the CCUC bit is
 * cleared, the timer channel compare registers are updated with the shadow
 * value when the counter matches the auto reload value.
 *
 * @param update Set to Shadow_Update::SHADOW_CCU to update the channel
 *               shadows when the counter matches the auto reload value or
 *               Shadow_Update::SHADOW_CCUTRI to update the channel shadows
 *               when the up-count trigger occurs.
 */
void TIMER::channel_shadow_update_configure(Shadow_Update update) {
    write_bit(*this, TIMER_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::CCUC), (update == Shadow_Update::SHADOW_CCUTRI));
}

/**
 * @brief Configures the output compare mode of the specified timer channel.
 *
 * This function configures the output compare mode of the specified timer
 * channel. The output compare mode is used to generate a pulse or a level
 * signal on the timer channel output pin. The pulse or level signal is
 * generated based on the compare value of the timer channel and the output
 * compare mode settings.
 *
 * @param[in] channel The timer channel to configure. Valid values are
 *                     Timer_Channel::CH0, Timer_Channel::CH1, Timer_Channel::CH2, or
 *                     Timer_Channel::CH3.
 * @param[in] compare_config The output compare mode configuration. The
 *                            configuration includes the output compare state,
 *                            polarity, idle state, and the companion output
 *                            compare state and polarity.
 */
void TIMER::output_compare_init(Timer_Channel channel, TIMER_Output_Compare compare_config) {
    if (channel == Timer_Channel::INVALID) {
        return;
    }

    // Channel-specific settings
    constexpr uint32_t CH_MS_BITS[] = { 
        static_cast<uint32_t>(CHCTL0_Bits::CH0MS),
        static_cast<uint32_t>(CHCTL0_Bits::CH1MS),
        static_cast<uint32_t>(CHCTL1_Bits::CH2MS),
        static_cast<uint32_t>(CHCTL1_Bits::CH3MS)
    };

    constexpr uint32_t CH_EN_BITS[] = {
        static_cast<uint32_t>(CHCTL2_Bits::CH0EN),
        static_cast<uint32_t>(CHCTL2_Bits::CH1EN),
        static_cast<uint32_t>(CHCTL2_Bits::CH2EN),
        static_cast<uint32_t>(CHCTL2_Bits::CH3EN)
    };

    constexpr uint32_t CH_P_BITS[] = {
        static_cast<uint32_t>(CHCTL2_Bits::CH0P),
        static_cast<uint32_t>(CHCTL2_Bits::CH1P),
        static_cast<uint32_t>(CHCTL2_Bits::CH2P),
        static_cast<uint32_t>(CHCTL2_Bits::CH3P)
    };

    constexpr uint32_t CH_NEN_BITS[] = {
        static_cast<uint32_t>(CHCTL2_Bits::CH0NEN),
        static_cast<uint32_t>(CHCTL2_Bits::CH1NEN),
        static_cast<uint32_t>(CHCTL2_Bits::CH2NEN),
        0U // CH3 doesn't have a companion channel
    };

    constexpr uint32_t CH_NP_BITS[] = {
        static_cast<uint32_t>(CHCTL2_Bits::CH0NP),
        static_cast<uint32_t>(CHCTL2_Bits::CH1NP),
        static_cast<uint32_t>(CHCTL2_Bits::CH2NP),
        0U // CH3 doesn't have a companion channel
    };

    constexpr uint32_t ISO_BITS[] = {
        static_cast<uint32_t>(CTL1_Bits::ISO0),
        static_cast<uint32_t>(CTL1_Bits::ISO1),
        static_cast<uint32_t>(CTL1_Bits::ISO2),
        static_cast<uint32_t>(CTL1_Bits::ISO3)
    };

    constexpr uint32_t ISO_N_BITS[] = {
        static_cast<uint32_t>(CTL1_Bits::ISO0N),
        static_cast<uint32_t>(CTL1_Bits::ISO1N),
        static_cast<uint32_t>(CTL1_Bits::ISO2N),
        0U // CH3 doesn't have a companion channel
    };

    // Clear the mode bits for the channel
    if (channel <= Timer_Channel::CH1) {
        write_bit_range(*this, TIMER_Regs::CHCTL0, CH_MS_BITS[static_cast<size_t>(channel)], Clear);
    } else {
        write_bit_range(*this, TIMER_Regs::CHCTL1, CH_MS_BITS[static_cast<size_t>(channel)], Clear);
    }

    // Configure output compare enable and polarity
    write_bits_sequence(*this, TIMER_Regs::CHCTL2,
                       CH_P_BITS[static_cast<size_t>(channel)], compare_config.polarity == Polarity_Select::LOW_FALLING,
                       CH_EN_BITS[static_cast<size_t>(channel)], compare_config.state == Output_Compare_State::OC_ENABLE);

    // Configure companion output compare (advanced timers only)
    if (is_advanced_timer() && CH_NEN_BITS[static_cast<size_t>(channel)] != 0U) {
        write_bits_sequence(*this, TIMER_Regs::CHCTL2,
                           CH_NP_BITS[static_cast<size_t>(channel)], compare_config.companion_polarity == Polarity_Select::LOW_FALLING,
                           CH_NEN_BITS[static_cast<size_t>(channel)], compare_config.companion_state == Output_Compare_State::OC_ENABLE);

        // Configure idle states
        write_bit_ranges(*this, TIMER_Regs::CTL1,
                           ISO_BITS[static_cast<size_t>(channel)], static_cast<uint32_t>(compare_config.idle_state),
                           ISO_N_BITS[static_cast<size_t>(channel)], static_cast<uint32_t>(compare_config.companion_idle_state));
    } else if (channel == Timer_Channel::CH3) {
        // Only for CH3: Configure idle state without companion
        write_bit_range(*this, TIMER_Regs::CTL1,
                           ISO_BITS[static_cast<size_t>(channel)], static_cast<uint32_t>(compare_config.idle_state));
    }

    // Store the configuration
    compare_config_ = compare_config;
}

/**
 * @brief Set the output mode for a given channel
 *
 * @param channel The channel to set the output mode for
 * @param mode The output mode to set
 *
 * The output mode is one of the following:
 * - OC_TIMING_MODE
 * - OC_OUTPUT_MODE
 * - OC_CLEAR_MODE
 * - OC_TOGGLE_MODE
 * - OC_FORCE_LOW_MODE
 * - OC_FORCE_HIGH_MODE
 *
 * @note This function does not affect the state of the channel. If the channel is disabled, calling this function will not enable it.
 */
void TIMER::set_output_mode(Timer_Channel channel, Output_Compare_Mode mode) {
    if (channel == Timer_Channel::INVALID) {
        return;
    }

    // Determine the register (CHCTL0 for CH0 and CH1, CHCTL1 for CH2 and CH3)
    const TIMER_Regs reg = (channel <= Timer_Channel::CH1) ? TIMER_Regs::CHCTL0 : TIMER_Regs::CHCTL1;

    // Calculate the bit offset for the COMCTL bits
    const uint32_t bit_offset = (channel == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL0_Bits::CH0COMCTL) :
                                (channel == Timer_Channel::CH1) ? static_cast<uint32_t>(CHCTL0_Bits::CH1COMCTL) :
                                (channel == Timer_Channel::CH2) ? static_cast<uint32_t>(CHCTL1_Bits::CH2COMCTL) :
                                                                  static_cast<uint32_t>(CHCTL1_Bits::CH3COMCTL);

    // Clear and set the COMCTL bits for the given channel
    write_bit_range(*this, reg, bit_offset, static_cast<uint32_t>(mode));
}

/**
 * @brief Sets the capture compare value for a given channel
 *
 * @param channel The channel to set the capture compare value for
 * @param cc_value The capture compare value to set
 *
 * @note This function sets the capture compare value for the specified channel. If the channel is disabled, calling this function will not enable it.
 */
void TIMER::set_capture_compare(Timer_Channel channel, uint32_t cc_value) {
    const TIMER_Regs reg = (channel == Timer_Channel::CH0) ? TIMER_Regs::CH0CV :
                           (channel == Timer_Channel::CH1) ? TIMER_Regs::CH1CV :
                           (channel == Timer_Channel::CH2) ? TIMER_Regs::CH2CV :
                                                             TIMER_Regs::CH3CV;

    write_bit_range(*this, reg, static_cast<uint32_t>(CHXCV_Bits::CHXCV), cc_value);
}

/**
 * @brief Enables or disables the output shadow feature for a given timer channel
 *
 * If the shadow feature is enabled, the output compare value will be loaded from the
 * shadow register on the next timer update. If the shadow feature is disabled, the
 * current output compare value will be used.
 *
 * @param channel The channel to set the shadow feature for
 * @param shadow The shadow feature to set. @see Output_Compare_Shadow
 */
void TIMER::set_output_shadow(Timer_Channel channel, Output_Compare_Shadow shadow) {
    if (channel == Timer_Channel::INVALID) {
        return;
    }

    // Determine the register (CHCTL0 for CH0 and CH1, CHCTL1 for CH2 and CH3)
    const TIMER_Regs reg = (channel <= Timer_Channel::CH1) ? TIMER_Regs::CHCTL0 : TIMER_Regs::CHCTL1;

    // Calculate the bit offset for the COMSEN bits
    const uint32_t bit_offset = (channel == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL0_Bits::CH0COMSEN) :
                                (channel == Timer_Channel::CH1) ? static_cast<uint32_t>(CHCTL0_Bits::CH1COMSEN) :
                                (channel == Timer_Channel::CH2) ? static_cast<uint32_t>(CHCTL1_Bits::CH2COMSEN) :
                                                                  static_cast<uint32_t>(CHCTL1_Bits::CH3COMSEN);

    // Clear and set the COMSEN bits for the given channel
    write_bit(*this, reg, bit_offset, (shadow == Output_Compare_Shadow::OC_SHADOW_ENABLE));
}

/**
 * @brief Enables or disables the fast output feature for a given timer channel
 *
 * If the fast output feature is enabled, the output compare value will be
 * updated immediately when the timer is updated. If the fast output feature is
 * disabled, the output compare value will be updated on the next timer
 * overflow event.
 *
 * @param channel The channel to set the fast output feature for
 * @param fast The fast output feature to set. @see Output_Compare_Fast
 */
void TIMER::set_output_fast(Timer_Channel channel, Output_Compare_Fast fast) {
    if (channel == Timer_Channel::INVALID) {
        return;
    }

    // Determine the register (CHCTL0 for CH0 and CH1, CHCTL1 for CH2 and CH3)
    const TIMER_Regs reg = (channel <= Timer_Channel::CH1) ? TIMER_Regs::CHCTL0 : TIMER_Regs::CHCTL1;

    // Calculate the bit offset for the COMFEN bits
    const uint32_t bit_offset = (channel == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL0_Bits::CH0COMFEN) :
                                (channel == Timer_Channel::CH1) ? static_cast<uint32_t>(CHCTL0_Bits::CH1COMFEN) :
                                (channel == Timer_Channel::CH2) ? static_cast<uint32_t>(CHCTL1_Bits::CH2COMFEN) :
                                                                  static_cast<uint32_t>(CHCTL1_Bits::CH3COMFEN);

    // Clear and set the COMFEN bits for the given channel
    write_bit(*this, reg, bit_offset, (fast == Output_Compare_Fast::OC_FAST_ENABLE));
}

/**
 * @brief Enables or disables the output clear feature for a given timer channel
 *
 * If the output clear feature is enabled, the output compare value will be
 * cleared (set to 0) when the timer overflows. If the output clear feature is
 * disabled, the output compare value will not be cleared.
 *
 * @param channel The channel to set the output clear feature for
 * @param oc_clear The output clear feature to set. @see Output_Compare_Clear
 */
void TIMER::set_output_clear(Timer_Channel channel, Output_Compare_Clear oc_clear) {
    if (channel == Timer_Channel::INVALID) {
        return;
    }

    // Determine the register (CHCTL0 for CH0 and CH1, CHCTL1 for CH2 and CH3)
    const TIMER_Regs reg = (channel <= Timer_Channel::CH1) ? TIMER_Regs::CHCTL0 : TIMER_Regs::CHCTL1;

    // Calculate the bit offset for the COMCEN bits
    const uint32_t bit_offset = (channel == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL0_Bits::CH0COMCEN) :
                                (channel == Timer_Channel::CH1) ? static_cast<uint32_t>(CHCTL0_Bits::CH1COMCEN) :
                                (channel == Timer_Channel::CH2) ? static_cast<uint32_t>(CHCTL1_Bits::CH2COMCEN) :
                                                                  static_cast<uint32_t>(CHCTL1_Bits::CH3COMCEN);

    // Clear and set the COMCEN bits for the given channel
    write_bit(*this, reg, bit_offset, (oc_clear == Output_Compare_Clear::OC_CLEAR_ENABLE));
}

/**
 * @brief Sets the output polarity for the specified timer channel.
 *
 * This function configures the output polarity of the given timer channel.
 * If the specified polarity is OUTPUT_LOW, the output signal will be inverted.
 * Otherwise, the output signal will remain non-inverted.
 *
 * @param channel The timer channel to configure the output polarity for.
 *                Valid values are Timer_Channel::CH0, Timer_Channel::CH1,
 *                Timer_Channel::CH2, Timer_Channel::CH3.
 * @param polarity The desired output polarity. Use Output_Polarity::OUTPUT_LOW
 *                 to set the output signal to low, or Output_Polarity::OUTPUT_HIGH
 *                 to keep the output signal high.
 *
 * @note Calling this function does not enable the channel. The channel must
 *       be enabled separately for the changes to take effect.
 */
void TIMER::set_output_polarity(Timer_Channel channel, Output_Polarity polarity) {
    if (channel == Timer_Channel::INVALID) {
        return;
    }

    // Calculate the bit offset for the CHxP bits
    const uint32_t bit_offset = (channel == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL2_Bits::CH0P) :
                                (channel == Timer_Channel::CH1) ? static_cast<uint32_t>(CHCTL2_Bits::CH1P) :
                                (channel == Timer_Channel::CH2) ? static_cast<uint32_t>(CHCTL2_Bits::CH2P) :
                                                                  static_cast<uint32_t>(CHCTL2_Bits::CH3P);

    // Clear and set the CHxP bits for the given channel
    write_bit(*this, TIMER_Regs::CHCTL2, bit_offset, (polarity == Output_Polarity::OUTPUT_LOW));
}

/**
 * @brief Sets the output polarity of the complement channel for the specified timer
 * channel.
 *
 * This function sets the output polarity of the complement channel for the
 * specified timer channel. The polarity can be set to either low (OUTPUT_LOW) or
 * high (OUTPUT_HIGH). If the specified polarity is OUTPUT_LOW, the complement
 * output signal will be low. Otherwise, the complement output signal will be
 * high.
 *
 * @param channel The timer channel to configure the complement output polarity
 *                for. Valid values are Timer_Channel::CH0, Timer_Channel::CH1,
 *                Timer_Channel::CH2, Timer_Channel::CH3.
 * @param polarity The desired output polarity. Use Output_Polarity::OUTPUT_LOW
 *                 to set the output signal to low, or Output_Polarity::OUTPUT_HIGH
 *                 to keep the output signal high.
 *
 * @note Calling this function does not enable the channel. The channel must
 *       be enabled separately for the changes to take effect.
 */
void TIMER::set_complement_output_polarity(Timer_Channel channel, Output_Polarity polarity) {
    if (channel == Timer_Channel::INVALID) {
        return;
    }

    // Calculate the bit offset for the CHxNP bits
    const uint32_t bit_offset = (channel == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL2_Bits::CH0NP) :
                                (channel == Timer_Channel::CH1) ? static_cast<uint32_t>(CHCTL2_Bits::CH1NP) :
                                                                  static_cast<uint32_t>(CHCTL2_Bits::CH2NP);

    // Clear and set the CHxNP bits for the given channel
    write_bit(*this, TIMER_Regs::CHCTL2, bit_offset, (polarity == Output_Polarity::OUTPUT_LOW));
}

/**
 * @brief Enables or disables the output of the specified timer channel.
 *
 * This function enables or disables the output of the specified timer channel.
 * Enabling the output will allow the channel to drive the external pin. Disabling
 * the output will prevent the channel from driving the external pin and will
 * instead leave the pin in a high-impedance state.
 *
 * @param channel The timer channel to enable or disable the output of. Valid
 *                values are Timer_Channel::CH0, Timer_Channel::CH1,
 *                Timer_Channel::CH2, Timer_Channel::CH3.
 * @param enable   If true, the output of the specified channel will be enabled.
 *                 If false, the output will be disabled.
 */
void TIMER::set_channel_output_enable(Timer_Channel channel, bool enable) {
    if (channel == Timer_Channel::INVALID) {
        return;
    }

    // Calculate the bit offset for the CHxEN bits
    const uint32_t bit_offset = (channel == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL2_Bits::CH0EN) :
                                (channel == Timer_Channel::CH1) ? static_cast<uint32_t>(CHCTL2_Bits::CH1EN) :
                                (channel == Timer_Channel::CH2) ? static_cast<uint32_t>(CHCTL2_Bits::CH2EN) :
                                                                  static_cast<uint32_t>(CHCTL2_Bits::CH3EN);

    // Clear and set the CHxEN bits for the given channel
    write_bit(*this, TIMER_Regs::CHCTL2, bit_offset, enable);
}

/**
 * @brief Enables or disables the compliment output of the specified timer channel.
 *
 * This function enables or disables the compliment output of the specified
 * timer channel. Enabling the compliment output will allow the channel to
 * drive the compliment external pin. Disabling the compliment output will
 * prevent the channel from driving the compliment external pin and will
 * instead leave the pin in a high-impedance state.
 *
 * @param channel The timer channel to enable or disable the compliment output
 *                of. Valid values are Timer_Channel::CH0, Timer_Channel::CH1,
 *                Timer_Channel::CH2, Timer_Channel::CH3.
 * @param enable   If true, the compliment output of the specified channel will
 *                 be enabled. If false, the compliment output will be
 *                 disabled.
 */
void TIMER::set_compliment_output_enable(Timer_Channel channel, bool enable) {
    if (channel == Timer_Channel::INVALID) {
        return;
    }

    // Calculate the bit offset for the CHxNEN bits
    const uint32_t bit_offset = (channel == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL2_Bits::CH0NEN) :
                                (channel == Timer_Channel::CH1) ? static_cast<uint32_t>(CHCTL2_Bits::CH1NEN) :
                                                                  static_cast<uint32_t>(CHCTL2_Bits::CH2NEN);

    // Clear and set the CHxNEN bits for the given channel
    write_bit(*this, TIMER_Regs::CHCTL2, bit_offset, enable);
}

/**
 * @brief Initializes the specified timer channel for input capture mode.
 *
 * This function initializes the specified timer channel for input capture
 * mode. The channel is first disabled and then configured according to the
 * given configuration. The channel is then enabled and the prescaler is set to
 * the value specified in the configuration.
 *
 * @param channel       The timer channel to initialize for input capture mode.
 *                      Valid values are Timer_Channel::CH0, Timer_Channel::CH1,
 *                      Timer_Channel::CH2, Timer_Channel::CH3.
 * @param capture_config The configuration for the input capture mode.
 */
void TIMER::input_capture_init(Timer_Channel channel, TIMER_Input_Capture capture_config) {
    if (channel == Timer_Channel::INVALID) {
        return;
    }

    // Channel-specific settings
    constexpr uint32_t CH_MS_BITS[] = { 
        static_cast<uint32_t>(CHCTL0_Bits::CH0MS),
        static_cast<uint32_t>(CHCTL0_Bits::CH1MS),
        static_cast<uint32_t>(CHCTL1_Bits::CH2MS),
        static_cast<uint32_t>(CHCTL1_Bits::CH3MS)
    };

    constexpr uint32_t CH_EN_BITS[] = {
        static_cast<uint32_t>(CHCTL2_Bits::CH0EN),
        static_cast<uint32_t>(CHCTL2_Bits::CH1EN),
        static_cast<uint32_t>(CHCTL2_Bits::CH2EN),
        static_cast<uint32_t>(CHCTL2_Bits::CH3EN)
    };

    constexpr uint32_t CH_P_BITS[] = {
        static_cast<uint32_t>(CHCTL2_Bits::CH0P),
        static_cast<uint32_t>(CHCTL2_Bits::CH1P),
        static_cast<uint32_t>(CHCTL2_Bits::CH2P),
        static_cast<uint32_t>(CHCTL2_Bits::CH3P)
    };

    constexpr uint32_t CH_NEN_BITS[] = {
        static_cast<uint32_t>(CHCTL2_Bits::CH0NEN),
        static_cast<uint32_t>(CHCTL2_Bits::CH1NEN),
        static_cast<uint32_t>(CHCTL2_Bits::CH2NEN),
        0U // CH3 doesn't have a companion channel
    };

    constexpr uint32_t CH_NP_BITS[] = {
        static_cast<uint32_t>(CHCTL2_Bits::CH0NP),
        static_cast<uint32_t>(CHCTL2_Bits::CH1NP),
        static_cast<uint32_t>(CHCTL2_Bits::CH2NP),
        0U // CH3 doesn't have a companion channel
    };

    constexpr uint32_t CH_CAPFLT_BITS[] = {
        static_cast<uint32_t>(CHCTL0_Bits::CH0CAPFLT),
        static_cast<uint32_t>(CHCTL0_Bits::CH1CAPFLT),
        static_cast<uint32_t>(CHCTL1_Bits::CH2CAPFLT),
        static_cast<uint32_t>(CHCTL1_Bits::CH3CAPFLT)
    };

    constexpr uint32_t CH_CAPPSC_BITS[] = {
        static_cast<uint32_t>(CHCTL0_Bits::CH0CAPPSC),
        static_cast<uint32_t>(CHCTL0_Bits::CH1CAPPSC),
        static_cast<uint32_t>(CHCTL1_Bits::CH2CAPPSC),
        static_cast<uint32_t>(CHCTL1_Bits::CH3CAPPSC)
    };

    // Disable companion channel and polarity
    if (CH_NEN_BITS[static_cast<size_t>(channel)] != 0U) {
        write_bits_sequence(*this, TIMER_Regs::CHCTL2,
                       CH_NEN_BITS[static_cast<size_t>(channel)], false,
                       CH_NP_BITS[static_cast<size_t>(channel)], false);
    }

    // Configure input capture polarity
    write_bits_sequence(*this, TIMER_Regs::CHCTL2,
                   CH_EN_BITS[static_cast<size_t>(channel)], false,
                   CH_P_BITS[static_cast<size_t>(channel)], capture_config.polarity == Polarity_Select::LOW_FALLING);

    const TIMER_Regs reg = (channel <= Timer_Channel::CH1) ? TIMER_Regs::CHCTL0 : TIMER_Regs::CHCTL1;

    // Configure input capture mode source and digital filter
    write_bit_ranges(*this, reg,
                   CH_CAPPSC_BITS[static_cast<size_t>(channel)], Clear,
                   CH_MS_BITS[static_cast<size_t>(channel)], static_cast<uint32_t>(capture_config.source_select),
                   CH_CAPFLT_BITS[static_cast<size_t>(channel)], static_cast<uint32_t>(capture_config.digital_filter));

    // Enable channel and set input capture prescaler
    write_bit(*this, TIMER_Regs::CHCTL2, CH_EN_BITS[static_cast<size_t>(channel)], true);
    write_bit_range(*this, reg, CH_CAPPSC_BITS[static_cast<size_t>(channel)], config_.prescaler);

    // Store config
    capture_config_ = capture_config;
}

/**
 * @brief Sets the input capture prescaler for the specified timer channel.
 *
 * This function sets the prescaler for the input capture mode of the specified
 * timer channel. The channel must have been initialized for input capture mode
 * before this function can be called.
 *
 * @param channel       The timer channel to set the prescaler for. Valid values
 *                      are Timer_Channel::CH0, Timer_Channel::CH1, Timer_Channel::CH2,
 *                      Timer_Channel::CH3.
 * @param prescaler     The prescaler to use for the input capture mode. Valid
 *                      values are Input_Capture_Prescaler::DIV1, Input_Capture_Prescaler::DIV2,
 *                      Input_Capture_Prescaler::DIV4, Input_Capture_Prescaler::DIV8,
 *                      Input_Capture_Prescaler::DIV16, Input_Capture_Prescaler::DIV32,
 *                      Input_Capture_Prescaler::DIV64, Input_Capture_Prescaler::DIV128,
 *                      Input_Capture_Prescaler::DIV256, Input_Capture_Prescaler::DIV512,
 *                      Input_Capture_Prescaler::DIV1024.
 */
void TIMER::set_input_capture_prescaler(Timer_Channel channel, Input_Capture_Prescaler prescaler) {
    if (channel == Timer_Channel::INVALID) {
        return;
    }

    // Determine the register (CHCTL0 for CH0 and CH1, CHCTL1 for CH2 and CH3)
    const TIMER_Regs reg = (channel <= Timer_Channel::CH1) ? TIMER_Regs::CHCTL0 : TIMER_Regs::CHCTL1;

    // Calculate the bit offset for the CHxCAPPSC bits
    const uint32_t bit_offset = (channel == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL0_Bits::CH0CAPPSC) :
                                (channel == Timer_Channel::CH1) ? static_cast<uint32_t>(CHCTL0_Bits::CH1CAPPSC) :
                                (channel == Timer_Channel::CH2) ? static_cast<uint32_t>(CHCTL1_Bits::CH2CAPPSC) :
                                                                  static_cast<uint32_t>(CHCTL1_Bits::CH3CAPPSC);

    // Clear and set the CHxCAPPSC bits for the given channel
    write_bit_range(*this, reg, bit_offset, static_cast<uint32_t>(prescaler));
}

/**
 * @brief Enables pwm input capture mode for the specified timer channel.
 * 
 * Enables the pwm input capture mode for the specified timer channel and configures
 * it according to the stored configuration. The configuration is stored in the
 * capture_config_ member variable and is set via the set_input_capture_config
 * function. The primary channel is configured to capture the rising edge of the
 * input signal and the secondary channel is configured to capture the falling
 * edge of the input signal.
 *
 * @param channel The timer channel to enable the input capture mode for. Valid
 *                values are Timer_Channel::CH0 and Timer_Channel::CH1.
 */
void TIMER::input_pwm_capture_enable(Timer_Channel channel) {
    if (channel == Timer_Channel::INVALID) {
        return;
    }

    // Determine the complementary channel and polarity/source settings
    Timer_Channel complementary_channel = (channel == Timer_Channel::CH0) ? Timer_Channel::CH1 : Timer_Channel::CH0;
    Polarity_Select input_polarity = (capture_config_.polarity == Polarity_Select::HIGH_RISING) ? Polarity_Select::LOW_FALLING : Polarity_Select::HIGH_RISING;
    Input_Capture_Select source = (capture_config_.source_select == Input_Capture_Select::IO_INPUT_CI0FE0) ? Input_Capture_Select::IO_INPUT_CI1FE0 : Input_Capture_Select::IO_INPUT_CI0FE0;

    // Helper lambda to configure a channel
    auto configure_channel = [this](Timer_Channel ch, Polarity_Select polarity, Input_Capture_Select src) {
        const uint32_t ch_en = (ch == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL2_Bits::CH0EN) : static_cast<uint32_t>(CHCTL2_Bits::CH1EN);
        const uint32_t ch_p = (ch == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL2_Bits::CH0P) : static_cast<uint32_t>(CHCTL2_Bits::CH1P);
        const uint32_t ch_np = (ch == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL2_Bits::CH0NP) : static_cast<uint32_t>(CHCTL2_Bits::CH1NP);
        const uint32_t ch_ms = (ch == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL0_Bits::CH0MS) : static_cast<uint32_t>(CHCTL0_Bits::CH1MS);
        const uint32_t ch_capflt = (ch == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL0_Bits::CH0CAPFLT) : static_cast<uint32_t>(CHCTL0_Bits::CH1CAPFLT);

        // Disable the channel and clear and set polarity
        write_bits_sequence(*this, TIMER_Regs::CHCTL2,
                           ch_en, false,
                           ch_np, false,
                           ch_p, polarity == Polarity_Select::LOW_FALLING);

        // Configure capture mode and digital filter
        write_bit_ranges(*this, TIMER_Regs::CHCTL0,
                         ch_ms, static_cast<uint32_t>(src),
                         ch_capflt, static_cast<uint32_t>(capture_config_.digital_filter));

        // Enable the channel
        write_bit(*this, TIMER_Regs::CHCTL2, ch_en, true);

        // Set input capture prescaler
        set_input_capture_prescaler(ch, capture_config_.prescaler);
    };

    // Configure the main channel and its complementary channel
    configure_channel(channel, capture_config_.polarity, capture_config_.source_select);
    configure_channel(complementary_channel, input_polarity, source);
}

/**
 * @brief Enables or disables the hall mode for the timer.
 *
 * This function sets the TI0S bit in the CTL1 register to enable or
 * disable the hall mode for the timer. In hall mode, the timer
 * generates trigger events based on the state of the TI0FP1 and
 * TI0FP2 pins.
 *
 * @param enable Set to true to enable the hall mode, or false to disable it.
 */
void TIMER::set_hall_mode_enable(bool enable) {
    write_bit(*this, TIMER_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::TI0S), enable);
}

/**
 * @brief Sets the internal trigger source for the timer.
 *
 * This function sets the TRGS bits in the SMCFG register to select the
 * internal trigger source for the timer. The trigger source is an internal
 * event that triggers the timer. The internal trigger source can be either
 * the Timer CH0 event, the Timer CH1 event, or the Timer CH2 event.
 *
 * @param trigger The internal trigger source to set for the timer. Must be a
 *                value from the Trigger_Select enumeration.
 */
void TIMER::set_input_trigger(Trigger_Select trigger) {
    write_bit_range(*this, TIMER_Regs::SMCFG,
               static_cast<uint32_t>(SMCFG_Bits::TRGS), static_cast<uint32_t>(trigger));
}

/**
 * @brief Configures the external trigger for the timer.
 *
 * This function sets the external trigger mode for the timer. The external
 * trigger is an external event that triggers the timer. The external trigger
 * mode includes the prescaler, polarity, and digital filter for the external
 * trigger.
 *
 * @param prescaler The prescaler value for the external trigger. Must be a
 *                  value from the External_Trigger_Prescaler enumeration.
 * @param polarity The polarity of the external trigger. Must be a value from
 *                 the Polarity_Select enumeration.
 * @param filter The digital filter value for the external trigger. The digital
 *               filter is a 4-bit value that specifies the number of
 *               successive edges of the external trigger that must be
 *               detected before the trigger is considered valid.
 */
void TIMER::external_trigger_configure(External_Trigger_Prescaler prescaler, Polarity_Select polarity, uint32_t filter) {
    write_bit(*this, TIMER_Regs::SMCFG,
               static_cast<uint32_t>(SMCFG_Bits::ETP), (polarity == Polarity_Select::LOW_FALLING));
    write_bit_ranges(*this, TIMER_Regs::SMCFG,
               static_cast<uint32_t>(SMCFG_Bits::ETPSC), static_cast<uint32_t>(prescaler),
               static_cast<uint32_t>(SMCFG_Bits::ETFC), filter);
}

/**
 * @brief Returns the capture compare value for a given channel
 *
 * @param channel The channel to get the capture compare value for
 * @return The capture compare value for the specified channel
 */
uint32_t TIMER::get_capture_compare(Timer_Channel channel) {
    if (channel == Timer_Channel::INVALID) {
        return 0;
    }

    const TIMER_Regs reg = (channel == Timer_Channel::CH0) ? TIMER_Regs::CH0CV :
                           (channel == Timer_Channel::CH1) ? TIMER_Regs::CH1CV :
                           (channel == Timer_Channel::CH2) ? TIMER_Regs::CH2CV :
                                                             TIMER_Regs::CH3CV;

    return read_register<uint32_t>(*this, reg);
}

/**
 * @brief Sets the master output trigger for the timer.
 *
 * This function sets the master mode control bits (MMC) in the CTL1 register to
 * select the master output trigger for the timer. The master output trigger is
 * used to generate output signals for the timer. The master output trigger can
 * be either the timer output clock, the timer output clock divided by 2, or the
 * timer output clock divided by 4.
 *
 * @param mode The master output trigger to set for the timer. Must be a
 *             value from the Master_Control enumeration.
 */
void TIMER::set_master_output_trigger(Master_Control mode) {
    write_bit_range(*this, TIMER_Regs::CTL1, static_cast<uint32_t>(CTL1_Bits::MMC), static_cast<uint32_t>(mode));
}

/**
 * @brief Sets the slave mode for the timer.
 *
 * This function sets the slave mode control bits (SMC) in the SMCFG register to
 * select the slave mode for the timer. The slave mode determines how the timer
 * responds to the trigger event. The slave mode can be either reset mode, gated
 * mode, trigger mode, or external clock mode.
 *
 * @param mode The slave mode to set for the timer. Must be a value from the
 *             Slave_Control enumeration.
 */
void TIMER::set_slave(Slave_Control mode) {
    write_bit_range(*this, TIMER_Regs::SMCFG, static_cast<uint32_t>(SMCFG_Bits::SMC), static_cast<uint32_t>(mode));
}

/**
 * @brief Enables or disables the master-slave mode for the timer.
 *
 * This function sets the MSM bit in the SMCFG register to enable or disable
 * the master-slave mode for the timer. When enabled, the timer will operate
 * in synchronization with a master or slave configuration.
 *
 * @param enable Set to true to enable master-slave mode, or false to disable it.
 */
void TIMER::set_master_slave_enable(bool enable) {
    write_bit(*this, TIMER_Regs::SMCFG, static_cast<uint32_t>(SMCFG_Bits::MSM), enable);
}

/**
 * @brief Configures the timer in quadrature decoder mode.
 *
 * This function sets the slave mode of the timer to quadrature decoder mode and
 * configures the input capture of channels 0 and 1 to capture the quadrature
 * encoding of the input pulse train. The polarity of the input signals can be
 * specified using the polarity1 and polarity2 parameters.
 *
 * @param mode The quadrature decoding mode to set, represented by the Decode_Mode
 *             enumeration.
 * @param polarity1 The polarity of channel 0, represented by the Polarity_Select
 *                  enumeration.
 * @param polarity2 The polarity of channel 1, represented by the Polarity_Select
 *                  enumeration.
 */
void TIMER::quadrature_decoder_configure(Decode_Mode mode, Polarity_Select polarity1, Polarity_Select polarity2) {
    // Set slave mode to quadrature decoder mode
    write_bit_range(*this, TIMER_Regs::SMCFG, static_cast<uint32_t>(SMCFG_Bits::SMC), static_cast<uint32_t>(mode));

    // Helper lambda to configure input capture and polarity for a channel
    auto configure_channel = [this](Timer_Channel ch, Polarity_Select polarity) {
        const uint32_t ch_ms = (ch == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL0_Bits::CH0MS) : static_cast<uint32_t>(CHCTL0_Bits::CH1MS);
        const uint32_t ch_p = (ch == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL2_Bits::CH0P) : static_cast<uint32_t>(CHCTL2_Bits::CH1P);
        const uint32_t ch_np = (ch == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL2_Bits::CH0NP) : static_cast<uint32_t>(CHCTL2_Bits::CH1NP);

        // Configure input capture source
        write_bit_range(*this, TIMER_Regs::CHCTL0, ch_ms, static_cast<uint32_t>(Input_Capture_Select::IO_INPUT_CI0FE0));

        // Configure polarity
        write_bits_sequence(*this, TIMER_Regs::CHCTL2,
                           ch_np, false,
                           ch_p, (polarity == Polarity_Select::LOW_FALLING));
    };

    // Configure both channels (CH0 and CH1)
    configure_channel(Timer_Channel::CH0, polarity1);
    configure_channel(Timer_Channel::CH1, polarity2);
}

/**
 * Sets the timer to use the internal clock as the clock source. This is
 * equivalent to setting the slave mode control (SMC) bit in the slave mode
 * configuration register (SMCFG) to 0.
 */
void TIMER::set_internal_clock() {
    write_bit_range(*this, TIMER_Regs::SMCFG, static_cast<uint32_t>(SMCFG_Bits::SMC), Clear);
}

/**
 * @brief Sets the timer to use the internal trigger as the clock source.
 *
 * This function configures the timer to use the selected internal trigger as
 * the clock source. The internal trigger source can be either the Timer CH0
 * event, the Timer CH1 event, or the Timer CH2 event. This function is
 * equivalent to setting the TRGS bits in the SMCFG register to select the
 * internal trigger source, and setting the SMC bit to 0 to enable the slave
 * mode.
 *
 * @param trigger The internal trigger source to set for the timer. Must be a
 *                value from the Trigger_Select enumeration.
 */
void TIMER::set_clock_from_internal_trigger(Trigger_Select trigger) {
    set_input_trigger(trigger);
    write_bit_range(*this, TIMER_Regs::SMCFG, static_cast<uint32_t>(SMCFG_Bits::SMC), static_cast<uint32_t>(Slave_Control::EXTERNAL0));
}

/**
 * @brief Sets the timer to use an external trigger as the clock source.
 *
 * This function configures the timer to use the selected external trigger as
 * the clock source. The external trigger source can be either the Timer CH0
 * event, the Timer CH1 event, or the Timer CH2 event. This function is
 * equivalent to setting the TRGS bits in the SMCFG register to select the
 * external trigger source, setting the polarity for the trigger, setting the
 * filter value for the trigger, and setting the SMC bit to 1 to enable the
 * slave mode.
 *
 * @param trigger The external trigger source to set for the timer. Must be a
 *                value from the Trigger_Select enumeration.
 * @param polarity The polarity of the trigger. Must be a value from the
 *                 Polarity_Select enumeration.
 * @param filter The filter value for the trigger. A value of 0 disables the
 *               filter, while a value of 15 enables a filter with a maximum
 *               duration of 15 timer clock cycles.
 */
void TIMER::set_clock_from_external_trigger(Trigger_Select trigger, Polarity_Select polarity, uint32_t filter) {
    // Determine the channel to configure based on the current trigger source
    Timer_Channel channel = (read_bit_range(*this, TIMER_Regs::SMCFG, static_cast<uint32_t>(SMCFG_Bits::TRGS)) == static_cast<uint32_t>(Trigger_Select::CI1FE1)) 
                            ? Timer_Channel::CH1 
                            : Timer_Channel::CH0;

    // Helper lambda to configure the selected channel
    auto configure_channel = [this](Timer_Channel ch, Polarity_Select polarity, uint32_t filter) {
        const uint32_t ch_en = (ch == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL2_Bits::CH0EN) : static_cast<uint32_t>(CHCTL2_Bits::CH1EN);
        const uint32_t ch_p = (ch == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL2_Bits::CH0P) : static_cast<uint32_t>(CHCTL2_Bits::CH1P);
        const uint32_t ch_np = (ch == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL2_Bits::CH0NP) : static_cast<uint32_t>(CHCTL2_Bits::CH1NP);
        const uint32_t ch_ms = (ch == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL0_Bits::CH0MS) : static_cast<uint32_t>(CHCTL0_Bits::CH1MS);
        const uint32_t ch_capflt = (ch == Timer_Channel::CH0) ? static_cast<uint32_t>(CHCTL0_Bits::CH0CAPFLT) : static_cast<uint32_t>(CHCTL0_Bits::CH1CAPFLT);

        // Disable the channel and clear polarity settings
        write_bits_sequence(*this, TIMER_Regs::CHCTL2,
                           ch_en, false,
                           ch_np, false,
                           ch_p, polarity == Polarity_Select::LOW_FALLING);

        // Configure input capture source and filter
        write_bit_ranges(*this, TIMER_Regs::CHCTL0,
                         ch_ms, static_cast<uint32_t>(Input_Capture_Select::IO_INPUT_CI0FE0),
                         ch_capflt, filter);

        // Enable the channel
        write_bit(*this, TIMER_Regs::CHCTL2, ch_en, true);
    };

    // Configure the determined channel
    configure_channel(channel, polarity, filter);

    // Set the external trigger source and slave control mode
    set_input_trigger(trigger);
    write_bit_range(*this, TIMER_Regs::SMCFG, static_cast<uint32_t>(SMCFG_Bits::SMC), static_cast<uint32_t>(Slave_Control::EXTERNAL0));
}

/**
 * @brief Configures the timer to use the external trigger as the clock source
 *        in Mode 0.
 *
 * This function configures the timer to use the external trigger as the clock
 * source in Mode 0. The external trigger is configured according to the
 * specified prescaler, polarity, and filter. The timer is put into slave mode
 * (SMC = 1) and the TRGS bits are set to select the external trigger as the
 * clock source (TRGS = ETIFP).
 *
 * @param prescaler The prescaler value for the external trigger. Must be a
 *                  value from the External_Trigger_Prescaler enumeration.
 * @param polarity The polarity of the external trigger. Must be a value from
 *                 the Polarity_Select enumeration.
 * @param filter The filter value for the external trigger. A value of 0
 *               disables the filter, while a value of 15 enables a filter with
 *               a maximum duration of 15 timer clock cycles.
 */
void TIMER::set_clock_mode0(External_Trigger_Prescaler prescaler, Polarity_Select polarity, uint32_t filter) {
    external_trigger_configure(prescaler, polarity, filter);
    write_bit_ranges(*this, TIMER_Regs::SMCFG,
               static_cast<uint32_t>(SMCFG_Bits::SMC), static_cast<uint32_t>(Slave_Control::EXTERNAL0),
               static_cast<uint32_t>(SMCFG_Bits::TRGS), static_cast<uint32_t>(Trigger_Select::ETIFP));
}

/**
 * @brief Configures the timer to use the external trigger as the clock source
 *        in Mode 1.
 *
 * This function configures the timer to use the external trigger as the clock
 * source in Mode 1. The external trigger is configured according to the
 * specified prescaler, polarity, and filter. The timer is put into slave mode
 * (SMC1 = 1) and the TRGS bits are set to select the external trigger as the
 * clock source (TRGS = ETIFP).
 *
 * @param prescaler The prescaler value for the external trigger. Must be a
 *                  value from the External_Trigger_Prescaler enumeration.
 * @param polarity The polarity of the external trigger. Must be a value from
 *                 the Polarity_Select enumeration.
 * @param filter The filter value for the external trigger. A value of 0
 *               disables the filter, while a value of 15 enables a filter with
 *               a maximum duration of 15 timer clock cycles.
 */
void TIMER::set_clock_mode1(External_Trigger_Prescaler prescaler, Polarity_Select polarity, uint32_t filter) {
    external_trigger_configure(prescaler, polarity, filter);
    write_bit(*this, TIMER_Regs::SMCFG, static_cast<uint32_t>(SMCFG_Bits::SMC1), true);
    write_bit_range(*this, TIMER_Regs::SMCFG, static_cast<uint32_t>(SMCFG_Bits::SMC), Clear);
}

/**
 * @brief Disables the timer clock mode 1.
 *
 * This function disables the timer clock mode 1 by clearing the SMC1 bit in
 * the SMCFG register. In Mode 1, the timer is clocked by an external trigger.
 * When the timer is disabled, the external trigger is ignored and the timer
 * clock is stopped. The timer will not generate any interrupts when the timer
 * clock is stopped.
 */
void TIMER::clock_mode1_disable() {
    write_bit(*this, TIMER_Regs::SMCFG, static_cast<uint32_t>(SMCFG_Bits::SMC1), false);
}

/**
 * @brief Enables or disables writing the CHxVAL registers to the timer.
 *
 * If `enable` is `true`, the CHxVAL registers can be written to the timer. If
 * `enable` is `false`, the CHxVAL registers cannot be written to the timer.
 *
 * @param enable If `true`, writing to the CHxVAL registers is enabled. If
 *               `false`, writing to the CHxVAL registers is disabled.
 */
void TIMER::set_write_chxval_enable(bool enable) {
    write_bit(*this, TIMER_Regs::CFG, static_cast<uint32_t>(CFG_Bits::CHVSEL), enable);
}

/**
 * @brief Enables or disables writing to the CHxVAL registers to update the
 *        output values of the timer.
 *
 * If `enable` is `true`, writing to the CHxVAL registers will update the
 * output values of the timer. If `enable` is `false`, writing to the CHxVAL
 * registers will not update the output values of the timer.
 *
 * @param enable If `true`, writing to the CHxVAL registers will update the
 *               output values of the timer. If `false`, writing to the CHxVAL
 *               registers will not update the output values of the timer.
 */
void TIMER::set_output_value_enable(bool enable) {
    write_bit(*this, TIMER_Regs::CFG, static_cast<uint32_t>(CFG_Bits::OUTSEL), enable);
}

/**
 * @brief Retrieves the status of a flag in the timer.
 *
 * This function reads the specified flag from the timer status register
 * (INTF). If the flag is not applicable to the timer, this function returns
 * false.
 *
 * @param flag The status flag to retrieve. Must be a value from the
 *             Status_Flags enumeration.
 * @return true if the flag is set, false otherwise.
 */
bool TIMER::get_flag(Status_Flags flag) {
    if (flag == Status_Flags::INVALID) {
        return false;
    }
    return read_bit(*this, TIMER_Regs::INTF, static_cast<uint32_t>(flag));
}

/**
 * @brief Clears a specified status flag in the timer.
 *
 * This function clears a given status flag specified by the Status_Flags
 * enumeration. The flag is cleared by writing a 0 to the corresponding bit
 * in the timer status register (INTF). If the flag is not applicable to the
 * timer, this function does nothing.
 *
 * @param flag The status flag to clear. Must be a value from the
 *             Status_Flags enumeration.
 */
void TIMER::clear_flag(Status_Flags flag) {
    if (flag == Status_Flags::INVALID) {
        return;
    }
    write_bit(*this, TIMER_Regs::INTF, static_cast<uint32_t>(flag), false);
}

/**
 * @brief Retrieves the status of a specified interrupt flag in the timer.
 *
 * This function checks both the status and control registers to determine if
 * a specified interrupt flag is set and enabled. It evaluates different interrupt
 * flags such as update interrupt, channel interrupts, and various error
 * conditions. If both the status and control bits for the specified interrupt flag
 * are set, the function returns true, indicating that the interrupt condition is
 * active and enabled.
 *
 * @param flag The interrupt flag to check, specified as an Interrupt_Flags enumeration value.
 * @return true if the specified interrupt flag is set and enabled, false otherwise.
 */
bool TIMER::get_interrupt_flag(Interrupt_Flags flag) {
    if (flag == Interrupt_Flags::INVALID) {
        return false;
    }
    return (read_bit(*this, TIMER_Regs::INTF, static_cast<uint32_t>(flag)) &&
            read_bit(*this, TIMER_Regs::DMAINTEN, static_cast<uint32_t>(flag)));
}

/**
 * @brief Clears a specified interrupt flag in the timer.
 *
 * This function clears a given interrupt flag specified by the Interrupt_Flags
 * enumeration. The flag is cleared by writing a 0 to the corresponding bit
 * in the timer status register (INTF). If the flag is not applicable to the
 * timer, this function does nothing.
 *
 * @param flag The interrupt flag to clear. Must be a value from the
 *             Interrupt_Flags enumeration.
 */
void TIMER::clear_interrupt_flag(Interrupt_Flags flag) {
    if (flag == Interrupt_Flags::INVALID) {
        return;
    }
    write_bit(*this, TIMER_Regs::INTF, static_cast<uint32_t>(flag), false);
}

/**
 * @brief Clears all interrupt flags in the timer.
 *
 * This function clears all interrupt flags in the timer by writing a 0 to
 * the corresponding bits in the timer status register (INTF). This includes
 * the update interrupt flag and channel interrupt flags (CH0, CH1, CH2, CH3).
 */
void TIMER::clear_all_interrupt_flags() {
    const uint32_t all_flags = ((1U << static_cast<size_t>(Interrupt_Flags::INTR_FLAG_UP)) |
                                (1U << static_cast<size_t>(Interrupt_Flags::INTR_FLAG_CH0)) |
                                (1U << static_cast<size_t>(Interrupt_Flags::INTR_FLAG_CH1)) |
                                (1U << static_cast<size_t>(Interrupt_Flags::INTR_FLAG_CH2)) |
                                (1U << static_cast<size_t>(Interrupt_Flags::INTR_FLAG_CH3)));

    write_bits(*this, TIMER_Regs::INTF, all_flags, false);
}

/**
 * @brief Enables or disables a specified interrupt type in the timer.
 *
 * This function enables or disables the specified interrupt type in the timer.
 * The function takes a boolean parameter enable, which determines whether the
 * interrupt type should be enabled (true) or disabled (false). The function then
 * looks up the register offset and bit information associated with the interrupt
 * type in the interrupt_type_index array and uses the write_bit_range function
 * to set the appropriate bits in the register.
 *
 * @param[in] type The interrupt type to enable or disable, specified as an
 *                 Interrupt_Type enumeration value.
 * @param[in] enable A boolean value indicating whether the interrupt type should
 *                   be enabled (true) or disabled (false).
 */
void TIMER::set_interrupt_enable(Interrupt_Type type, bool enable) {
    if (type == Interrupt_Type::INVALID) {
        return;
    }
    write_bit(*this, TIMER_Regs::DMAINTEN, static_cast<uint32_t>(type), enable);
}

/**
 * @brief Disables all interrupts in the timer.
 *
 * This function clears the interrupt enable bits for all types of interrupts
 * associated with the timer, effectively disabling them. The interrupt types
 * include update interrupts and channel interrupts (CH0, CH1, CH2, CH3).
 * By clearing these bits in the DMAINTEN register, the timer will no longer
 * generate interrupts for the specified types.
 */
void TIMER::disable_all_interrupts() {
    const uint32_t all_types = ((1U << static_cast<uint32_t>(Interrupt_Type::INTR_UPIE)) |
                                (1U << static_cast<uint32_t>(Interrupt_Type::INTR_CH0IE)) |
                                (1U << static_cast<uint32_t>(Interrupt_Type::INTR_CH1IE)) |
                                (1U << static_cast<uint32_t>(Interrupt_Type::INTR_CH2IE)) |
                                (1U << static_cast<uint32_t>(Interrupt_Type::INTR_CH3IE)));

    write_bits(*this, TIMER_Regs::DMAINTEN, all_types, false);
}


} // namespace timer
