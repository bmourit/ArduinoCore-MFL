//
// MFL gd32f30x RCU peripheral register access in C++
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

#include "RCU.hpp"

namespace rcu {

RCU& RCU::get_instance() {
    static RCU instance;
    return instance;
}

RCU::RCU() : SystemCoreClock(static_cast<uint32_t>(IRC8M_VALUE)) {}

/**
 * @brief Reset RCU to its initial state after power-on reset.
 *
 * Resets all RCU registers to their initial state, except for the IRC8MEN bit in the CTL register
 * and the CKMIC bit in the INTR register, which are set to 1 (enabled). All oscillator sources are
 * reset to their initial state (not enabled), and the system clock source is set to the IRC8M.
 *
 * This function is intended to be called during the early stages of the device initialization, before
 * any other RCU functions are called.
 */
void RCU::reset() {
    // Enable IRC8M
    write_bit(*this, RCU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::IRC8MEN), true);
    while (is_osci_stable(OSCI_Select::IRC8M) == false) {
    }
    // Clear system clk source
    write_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::SCS), Clear);
    // Reset CTL register
    write_bits_sequence(*this, RCU_Regs::CTL,
                        static_cast<uint32_t>(CTL_Bits::HXTALEN), false,
                        static_cast<uint32_t>(CTL_Bits::CKMEN), false,
                        static_cast<uint32_t>(CTL_Bits::PLLEN), false,
                        static_cast<uint32_t>(CTL_Bits::HXTALBPS), false);
    // Reset CFG0 register
    write_bit_ranges(*this, RCU_Regs::CFG0,
                     static_cast<uint32_t>(CFG0_Bits::SCS), Clear,
                     static_cast<uint32_t>(CFG0_Bits::AHBPSC), Clear,
                     static_cast<uint32_t>(CFG0_Bits::APB1PSC), Clear,
                     static_cast<uint32_t>(CFG0_Bits::APB2PSC), Clear,
                     static_cast<uint32_t>(CFG0_Bits::ADCPSC), Clear,
                     static_cast<uint32_t>(CFG0_Bits::PLLMF), Clear,
                     static_cast<uint32_t>(CFG0_Bits::USBDPSC), Clear,
                     static_cast<uint32_t>(CFG0_Bits::CKOUT0SEL), Clear);
    write_bits_sequence(*this, RCU_Regs::CFG0,
                        static_cast<uint32_t>(CFG0_Bits::PLLSEL), false,
                        static_cast<uint32_t>(CFG0_Bits::PREDV0), false,
                        static_cast<uint32_t>(CFG0_Bits::PLLMF_4), false,
                        static_cast<uint32_t>(CFG0_Bits::ADCPSC_2), false,
                        static_cast<uint32_t>(CFG0_Bits::PLLMF_5), false,
                        static_cast<uint32_t>(CFG0_Bits::USBDPSC_2), false);
    write_register(*this, RCU_Regs::INTR,
                   static_cast<uint32_t>(INTR_Bits::CLEAR_ALL) |
                   (1U << static_cast<uint32_t>(INTR_Bits::CKMIC)));
    write_bits_sequence(*this, RCU_Regs::CFG1,
                        static_cast<uint32_t>(CFG1_Bits::ADCPSC_3), false,
                        static_cast<uint32_t>(CFG1_Bits::PLLPRESEL), false);
}

/**
 * @brief Enable or disable peripheral clock.
 *
 * @param[in] pclk Peripheral clock identifier.
 * @param[in] enable If true, the peripheral clock is enabled. If false, the peripheral clock is disabled.
 */
void RCU::set_pclk_enable(RCU_PCLK pclk, bool enable) {
    const auto& info = pclk_index[static_cast<size_t>(pclk)];
    write_bit_range(*this, info.register_offset, info.bit_info, enable ? Set : Clear);
}

/**
 * @brief Check if peripheral clock is enabled.
 *
 * @param[in] pclk Peripheral clock identifier.
 * @return true if the peripheral clock is enabled, false otherwise.
 */
bool RCU::get_pclk(RCU_PCLK pclk) {
    const auto& info = pclk_index[static_cast<size_t>(pclk)];
    return read_bit_range(*this, info.register_offset, info.bit_info) != Clear;
}

/**
 * @brief Enable or disable peripheral clock sleep mode.
 *
 * @param[in] pclk Peripheral clock sleep identifier.
 * @param[in] enable If true, the peripheral clock sleep mode is enabled. If false, it is disabled.
 */
void RCU::set_pclk_sleep_enable(RCU_PCLK_Sleep pclk, bool enable) {
    const auto& info = pclk_sleep_index[static_cast<size_t>(pclk)];
    write_bit_range(*this, info.register_offset, info.bit_info, enable ? Set : Clear);
}

/**
 * @brief Enable or disable peripheral clock reset.
 *
 * @param[in] pclk Peripheral clock reset identifier.
 * @param[in] enable If true, the peripheral clock reset is enabled. If false, the peripheral clock reset is disabled.
 */
void RCU::set_pclk_reset_enable(RCU_PCLK_Reset pclk, bool enable) {
    const auto& info = pclk_reset_index[static_cast<size_t>(pclk)];
    write_bit_range(*this, info.register_offset, info.bit_info, enable ? Set : Clear);
}

/**
 * @brief Enables or disables the backup domain reset.
 *
 * This function sets or clears the BKPRST bit in the BDCTL register
 * to initiate a reset of the backup domain.
 *
 * @param enable Set to true to enable the backup domain reset, false to disable it.
 */
void RCU::set_backup_reset_enable(bool enable) {
    write_bit(*this, RCU_Regs::BDCTL, static_cast<uint32_t>(BDCTL_Bits::BKPRST), enable);
}

/**
 * @brief Sets the system clock source.
 *
 * This function sets the system clock source using the System_Clock_Source
 * enum. If the specified source is System_Clock_Source::SOURCE_INVALID, the
 * system clock source is not changed.
 *
 * @param[in] source The system clock source to be set.
 */
void RCU::set_system_source(System_Clock_Source source) {
    if (source == System_Clock_Source::SOURCE_INVALID) {
        return;
    }
    write_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::SCS), static_cast<uint32_t>(source));
}

/**
 * @brief Retrieves the current system clock source.
 *
 * This function reads the system clock source selection from the CFG0 register
 * and maps the retrieved value to a corresponding System_Clock_Source enum value
 * using a predefined lookup table. If the retrieved clock source does not match
 * any known source, it returns System_Clock_Source::SOURCE_INVALID.
 *
 * @return The current system clock source as a System_Clock_Source enumeration.
 */
System_Clock_Source RCU::get_system_source() {
    uint32_t clock = read_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::SCSS));
    // Find the corresponding enum value using the lookup table
    for (const auto& mapping : source_mapping) {
        if (mapping.value == clock) {
            return mapping.source;
        }
    }
    // Handle invalid selection
    return System_Clock_Source::SOURCE_INVALID;
}

/**
 * @brief Sets the AHB prescaler.
 *
 * This function sets the AHB prescaler based on the AHB_Prescaler enum value.
 * If the specified prescaler is AHB_Prescaler::INVALID, the AHB prescaler is
 * not changed.
 *
 * @param[in] prescaler The AHB prescaler to be set.
 */
void RCU::set_ahb_prescaler(AHB_Prescaler prescaler) {
    if (prescaler == AHB_Prescaler::INVALID) { return; }
    write_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::AHBPSC), static_cast<uint32_t>(prescaler));
}

/**
 * @brief Retrieves the current AHB prescaler setting.
 *
 * This function reads the AHB prescaler value from the CFG0 register and
 * returns it as an AHB_Prescaler enumeration. The prescaler value determines
 * the division factor applied to the system clock to produce the AHB clock.
 *
 * @return The current AHB prescaler as an AHB_Prescaler enumeration.
 */
AHB_Prescaler RCU::get_ahb_prescaler() {
    uint32_t prescaler = read_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::AHBPSC));
    return static_cast<AHB_Prescaler>(prescaler);
}

/**
 * @brief Sets the APB1 prescaler.
 *
 * This function configures the APB1 prescaler with the specified value to
 * adjust the frequency of the APB1 peripheral clock. If the provided prescaler
 * is APB_Prescaler::INVALID, the function will return without making any changes.
 *
 * @param[in] prescaler The APB1 prescaler to be set.
 */
void RCU::set_apb1_prescaler(APB_Prescaler prescaler) {
    if (prescaler == APB_Prescaler::INVALID) { return; }
    write_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::APB1PSC), static_cast<uint32_t>(prescaler));
}

/**
 * @brief Retrieves the current APB1 prescaler setting.
 *
 * This function reads the APB1 prescaler value from the CFG0 register and
 * returns it as an APB_Prescaler enumeration. The prescaler value determines
 * the division factor applied to the AHB clock to produce the APB1 clock.
 *
 * @return The current APB1 prescaler as an APB_Prescaler enumeration.
 */
APB_Prescaler RCU::get_apb1_prescaler() {
    uint32_t prescaler = read_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::APB1PSC));
    return static_cast<APB_Prescaler>(prescaler);
}

/**
 * @brief Sets the APB2 prescaler.
 *
 * This function configures the APB2 prescaler with the specified value to
 * adjust the frequency of the APB2 peripheral clock. If the provided prescaler
 * is APB_Prescaler::INVALID, the function will return without making any changes.
 *
 * @param[in] prescaler The APB2 prescaler to be set.
 */
void RCU::set_apb2_prescaler(APB_Prescaler prescaler) {
    if (prescaler == APB_Prescaler::INVALID) { return; }
    write_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::APB2PSC), static_cast<uint32_t>(prescaler));
}

/**
 * @brief Retrieves the current APB2 prescaler setting.
 *
 * This function reads the APB2 prescaler value from the CFG0 register and
 * returns it as an APB_Prescaler enumeration. The prescaler value determines
 * the division factor applied to the AHB clock to produce the APB2 clock.
 *
 * @return The current APB2 prescaler as an APB_Prescaler enumeration.
 */
APB_Prescaler RCU::get_apb2_prescaler() {
    uint32_t prescaler = read_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::APB2PSC));
    return static_cast<APB_Prescaler>(prescaler);
}

/**
 * @brief Sets the CKOUT0 source.
 *
 * This function configures the CKOUT0 source by setting the appropriate bits
 * in the CFG0 register. The CKOUT0 source determines the clock output signal
 * from the microcontroller, which can be used to drive external devices.
 *
 * @param[in] source The CKOUT0 source to be set, represented by the CKOUT0_Source enum.
 */
void RCU::set_ckout0_source(CKOUT0_Source source) {
    write_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::CKOUT0SEL), static_cast<uint32_t>(source));
}

/**
 * @brief Configures the PLL (Phase-Locked Loop) settings.
 *
 * This function sets the PLL source and multiplier to the specified values.
 * If the source is PLL_Source::PLLSRC_INVALID, the function will return without
 * making any changes.
 *
 * @param[in] source The PLL source to be set, represented by the PLL_Source enum.
 * @param[in] multiplier The PLL multiplier to be set, represented by the PLLMF_Select enum.
 */
void RCU::set_pll_config(PLL_Source source, PLLMF_Select multiplier) {
    if (source == PLL_Source::PLLSRC_INVALID) { return; }
    // Set PLL source
    write_bit(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::PLLSEL), (source == PLL_Source::PLLSRC_HXTAL_IRC48M));
    uint32_t bits = static_cast<uint32_t>(multiplier);
    write_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::PLLMF), bits & 0xFU);
    write_bits_sequence(*this, RCU_Regs::CFG0,
                        static_cast<uint32_t>(CFG0_Bits::PLLMF_4), (bits & 0x10U) >> 4U == 1U,
                        static_cast<uint32_t>(CFG0_Bits::PLLMF_5), (bits & 0x20U) >> 5U == 1U);
}

/**
 * @brief Retrieves the current PLL source setting.
 *
 * This function reads the PLL source value from the CFG0 register and
 * returns it as a PLL_Source enumeration. The source value determines
 * the clock source used for the PLL.
 *
 * @return The current PLL source as a PLL_Source enumeration.
 */
PLL_Source RCU::get_pll_source() {
    bool clock = read_bit(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::PLLSEL));
    for (const auto& mapping : pll_mapping) {
        if (mapping.value == clock) {
            return mapping.source;
        }
    }
    // Handle invalid selection
    return PLL_Source::PLLSRC_INVALID;
}

/**
 * @brief Configures the PLL prescaler setting.
 *
 * This function sets the PLL prescaler to the specified value. If the value is
 * PLL_Presel::PLLPRESRC_INVALID, the function will return without making any
 * changes.
 *
 * @param[in] presel The PLL prescaler to be set, represented by the PLL_Presel enum.
 */
void RCU::set_pll_presel(PLL_Presel presel) {
    if (presel == PLL_Presel::PLLPRESRC_INVALID) { return; }
    write_bit(*this, RCU_Regs::CFG1, static_cast<uint32_t>(CFG1_Bits::PLLPRESEL), (presel == PLL_Presel::PLLPRESRC_IRC48M));
}

/**
 * @brief Retrieves the current PLL prescaler setting.
 *
 * This function reads the PLL prescaler value from the CFG1 register and
 * returns it as a PLL_Presel enumeration. The prescaler value determines
 * the clock source used for the PLL.
 *
 * @return The current PLL prescaler as a PLL_Presel enumeration.
 */
PLL_Presel RCU::get_pll_presel() {
    bool clock = read_bit(*this, RCU_Regs::CFG1, static_cast<uint32_t>(CFG1_Bits::PLLPRESEL));
    for (const auto& mapping : pll_presel_mapping) {
        if (mapping.value == clock) {
            return mapping.source;
        }
    }
    // Handle invalid selection
    return PLL_Presel::PLLPRESRC_INVALID;
}

/**
 * @brief Configures the PREDV0 (predvider) clock source for the PLL.
 *
 * This function sets the PREDV0 clock speed to either the clock source speed
 * or the clock source speed divided by 2. If the value is true, the source clock
 * is divided by 2; otherwise, the source clock speed is used.
 *
 * @param[in] div value for PREDV0 to be set, where true
 *                denotes the source clock will be divided by 2
 *                and false denotes no predivision.
 */
void RCU::set_predv0_config(bool div) {
    write_bit(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::PREDV0), div);
}

/**
 * @brief Sets the ADC prescaler.
 *
 * This function configures the ADC prescaler by setting the appropriate bits
 * in the CFG0 and CFG1 registers. It first validates the input prescaler, and if
 * it is valid, it clears the current prescaler bits and writes the new prescaler
 * value. The prescaler determines the division factor applied to the clock signal
 * provided to the ADC module.
 *
 * @param[in] prescaler The ADC prescaler to be set, represented by the ADC_Prescaler enum.
 *                      If ADC_Prescaler::INVALID is provided, the function will not modify
 *                      any registers.
 */
void RCU::set_adc_prescaler(ADC_Prescaler prescaler) {
    if (prescaler == ADC_Prescaler::INVALID) {
        return;
    }
    // Reset prescaler
    write_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::ADCPSC), Clear);
    write_bit(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::ADCPSC_2), false);
    write_bit(*this, RCU_Regs::CFG1, static_cast<uint32_t>(CFG1_Bits::ADCPSC_3), false);

    uint32_t bits = static_cast<uint32_t>(prescaler);

    // Write bit 0 and 1 to ADCPSC and bit 2 to ADCPSC_2
    write_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::ADCPSC), (bits & 0x3U));
    write_bit(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::ADCPSC_2), ((bits & 0x4U) >> 2U) == 1U);
    // Write bit 3 to ADCPSC_3
    write_bit(*this, RCU_Regs::CFG1, static_cast<uint32_t>(CFG1_Bits::ADCPSC_3), ((bits & 0x8U) >> 3U) == 1U);
}

/**
 * @brief Retrieves the current ADC prescaler setting.
 *
 * This function reads the ADC prescaler value from the CFG0 and CFG1 registers and
 * returns it as an ADC_Prescaler enumeration. The prescaler value determines the
 * division factor applied to the clock signal provided to the ADC module.
 *
 * @return The current ADC prescaler as an ADC_Prescaler enumeration.
 */
ADC_Prescaler RCU::get_adc_prescaler() {
    uint32_t bits = read_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::ADCPSC));
    bits |= (2U << (read_bit(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::ADCPSC_2))) == true ? 1U : 0U);
    bits |= (3U << (read_bit(*this, RCU_Regs::CFG1, static_cast<uint32_t>(CFG1_Bits::ADCPSC_3))) == true ? 1U : 0U);
    return static_cast<ADC_Prescaler>(bits);
}

/**
 * @brief Sets the prescaler for the USB module.
 *
 * This function sets the prescaler for the USB module based on the USB_Prescaler
 * enumeration. If USB_Prescaler::INVALID is provided, the function will not
 * modify any registers.
 *
 * @param[in] prescaler The USB prescaler to be set, represented by the USB_Prescaler
 *                      enumeration. If USB_Prescaler::INVALID is provided, the function
 *                      will not modify any registers.
 */
void RCU::set_usb_prescaler(USB_Prescaler prescaler) {
    uint32_t bits = static_cast<uint32_t>(prescaler);
    // Write the lower bits (0, 1) to USBDPSC and bit 2 to USBDPSC_2
    write_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::USBDPSC), bits & 0x3U);
    write_bit(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::USBDPSC_2), ((bits & 0x4U) >> 2U));
}

/**
 * @brief Selects the clock source for the RTC (Real-Time Clock) module.
 *
 * This function sets the clock source for the RTC module based on the
 * RTC_Source enumeration. The clock source can either be the IRC8M or
 * the LSE (Low-Speed External) clock.
 *
 * @param[in] source The clock source to be set, represented by the
 *                   RTC_Source enumeration.
 */
void RCU::set_rtc_source(RTC_Source source) {
    write_bit_range(*this, RCU_Regs::BDCTL, static_cast<uint32_t>(BDCTL_Bits::RTCSRC), static_cast<uint32_t>(source));
}

/**
 * @brief Sets the clock source for the CK48M clock.
 *
 * This function sets the clock source for the CK48M clock to either the IRC48M
 * clock or the HXTAL clock divided by 2. If the source is CK48M_Source::CK48MSRC_IRC48M,
 * the IRC48M clock is selected; otherwise, the HXTAL clock divided by 2 is selected.
 *
 * @param[in] source The clock source to be set, represented by the CK48M_Source
 *                   enumeration.
 */
void RCU::set_ck48m_source(CK48M_Source source) {
    write_bit(*this, RCU_Regs::ADDCTL, static_cast<uint32_t>(ADDCTL_Bits::CK48MSEL), (source == CK48M_Source::CK48MSRC_IRC48M));
}

/**
 * @brief Sets the drive capability for the LXTAL (Low-Speed External Crystal Oscillator).
 *
 * This function configures the drive capability of the LXTAL by writing the
 * specified drive value to the LXTALDRI bits in the BDCTL register. The drive
 * capability is set using the LXTAL_Drive enumeration, which defines the
 * drive strength levels available for the LXTAL.
 *
 * @param[in] drive The drive capability to be set, represented by the LXTAL_Drive
 *                  enumeration.
 */
void RCU::set_lxtal_drive_capability(LXTAL_Drive drive) {
    write_bit_range(*this, RCU_Regs::BDCTL, static_cast<uint32_t>(BDCTL_Bits::LXTALDRI), static_cast<uint32_t>(drive));
}

/**
 * @brief Enables or disables the specified oscillator source.
 *
 * This function enables or disables the specified oscillator source, given as an
 * OSCI_Select enumeration. The function takes a boolean parameter enable, which
 * determines whether the oscillator source should be enabled (true) or disabled
 * (false). The function then looks up the register offset and bit information
 * associated with the oscillator source in the osci_select_index array and uses
 * the write_bit_range function to set the appropriate bits in the register.
 *
 * @param[in] osci The oscillator source to be enabled or disabled, represented by
 *                 the OSCI_Select enumeration.
 * @param[in] enable A boolean value indicating whether the oscillator source
 *                   should be enabled (true) or disabled (false).
 */
void RCU::set_osci_enable(OSCI_Select osci, bool enable) {
    const auto& info = osci_select_index[static_cast<size_t>(osci)];
    write_bit_range(*this, info.register_offset, info.bit_info, enable ? Set : Clear);
}

/**
 * @brief Checks the stability of a specified oscillator.
 *
 * This function determines whether a given oscillator source is stable by checking
 * its corresponding status flag within a specified timeout period. The function
 * continuously polls the stability flag of the oscillator until it becomes stable
 * or the timeout is reached.
 *
 * @param[in] osci The oscillator source to be checked for stability, represented by
 *                 the OSCI_Select enumeration.
 * @return true if the oscillator is stable within the timeout period, false otherwise.
 */
bool RCU::is_osci_stable(OSCI_Select osci) {
    volatile uint32_t count = 0U;
    bool osci_stable = false;

    const uint32_t timeout = (osci == OSCI_Select::HXTAL) ? HXTAL_STARTUP_TIMEOUT :
                             (osci == OSCI_Select::LXTAL) ? LXTAL_STARTUP_TIMEOUT :
                             (osci == OSCI_Select::IRC8M) ? IRC8M_STARTUP_TIMEOUT :
                             OSC_STARTUP_TIMEOUT;  // Default for IRC48M, IRC40K, PLL_CK

    const Status_Flags flag = (osci == OSCI_Select::HXTAL) ? Status_Flags::FLAG_HXTALSTB :
                              (osci == OSCI_Select::LXTAL) ? Status_Flags::FLAG_LXTALSTB :
                              (osci == OSCI_Select::IRC8M) ? Status_Flags::FLAG_IRC8MSTB :
                              (osci == OSCI_Select::IRC48M) ? Status_Flags::FLAG_IRC48MSTB :
                              (osci == OSCI_Select::IRC40K) ? Status_Flags::FLAG_IRC40KSTB :
                              Status_Flags::FLAG_PLLSTB; // Default for PLL_CK

    while ((osci_stable == false) && (count != timeout)) {
        osci_stable = get_flag(flag);
        count = count + 1U;
    }

    return osci_stable;
}

/**
 * @brief Gets the frequency of the specified system clock source.
 *
 * This function takes a System_Clock_Source enumeration as input and returns
 * the frequency of the specified system clock source in Hz. If the input is
 * System_Clock_Source::SOURCE_INVALID or any other invalid value, the function
 * returns the frequency of the IRC8M as a default value.
 *
 * @param[in] source The system clock source to get the frequency of, represented
 *                   by the System_Clock_Source enumeration.
 * @return The frequency of the specified system clock source in Hz.
 */
uint32_t RCU::get_clock_source_frequency(System_Clock_Source source) {
    switch (source) {
        case System_Clock_Source::SOURCE_IRC8M: return IRC8M_VALUE;
        case System_Clock_Source::SOURCE_HXTAL: return HXTAL_VALUE;
        case System_Clock_Source::SOURCE_PLL: return calculate_pll_frequency();
        case System_Clock_Source::SOURCE_INVALID:
        default: return IRC8M_VALUE;
    }
}

/**
 * @brief Calculates the frequency of the PLL clock based on its source and prescaler settings.
 *
 * This function first determines the PLL source (HXTAL or IRC8M/IRC48M) based on the
 * PLLSRC bits in the CFG0 register. If the source is HXTAL or IRC48M, it then determines
 * whether the PLL is prescaled by 2 based on the PREDV0 bit in the CFG0 register.
 * Finally, it multiplies the source frequency by the PLL multiplier (calculated by
 * get_pll_multiplier) to get the final PLL frequency.
 *
 * @return The frequency of the PLL clock in Hz. If the PLL source is invalid or
 *         cannot be determined, the function returns 0.
 */
uint32_t RCU::calculate_pll_frequency() {
    uint32_t osci_source = 0U;

    // Determine PLL source (HXTAL or IRC8M/IRC48M)
    PLL_Source pll_source = get_pll_source();
    if (pll_source == PLL_Source::PLLSRC_INVALID) {
        return 0U;
    }

    if (pll_source == PLL_Source::PLLSRC_HXTAL_IRC48M) {
        PLL_Presel pll_presel = get_pll_presel();
        if (pll_presel == PLL_Presel::PLLPRESRC_INVALID) {
            return 0U;
        }
        osci_source = (pll_presel == PLL_Presel::PLLPRESRC_HXTAL) ? HXTAL_VALUE : IRC48M_VALUE;
        if (read_bit(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::PREDV0))) {
            osci_source /= 2U;
        }
    } else {
        osci_source = IRC8M_VALUE / 2U;  // PLL source is IRC8M/2
    }

    // Calculated PLL multiplier
    return osci_source * get_pll_multiplier();
}

/**
 * @brief Calculates the PLL multiplier based on the PLLMF and PLLMF_4/_5 bits in the CFG0 register.
 *
 * This function first reads the PLLMF bits in the CFG0 register and then adjusts the multiplier based
 * on the value. If the PLLMF bits are less than 15, the function adds 2 to the multiplier. If the PLLMF
 * bits are between 15 and 62 (inclusive), the function adds 1 to the multiplier. For any other value, the
 * function returns 63 as the multiplier.
 *
 * @return The PLL multiplier calculated from the PLLMF and PLLMF_4/_5 bits in the CFG0 register.
 */
uint32_t RCU::get_pll_multiplier() {
    uint32_t multiplier = read_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::PLLMF));
    if (read_bit(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::PLLMF_4))) {
        multiplier |= 0x10U;
    }
    if (read_bit(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::PLLMF_5))) {
        multiplier |= 0x20U;
    }
    // Adjust the multiplier based on the value
    return (multiplier < 15U) ? (multiplier + 2U) : (multiplier <= 62U) ? (multiplier + 1U) : 63U;
}

/**
 * @brief Returns the frequency of the specified clock source.
 *
 * @param clock The clock source to return the frequency for.
 * @return The frequency of the specified clock source in Hz.
 *
 * @note The returned frequency is based on the current system clock source and its prescaler settings.
 * @note If the specified clock source is invalid (i.e. the clock source is not enabled), the function returns 0.
 */
uint32_t RCU::get_clock_frequency(Clock_Frequency clock) {
    System_Clock_Source current_source = get_system_source();

    if (current_source == System_Clock_Source::SOURCE_INVALID) { return 0; }
    uint32_t cksys_freq = get_clock_source_frequency(current_source);

    // Calculate AHB frequency
    uint32_t ahb_prescaler_idx = read_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::AHBPSC));
    uint32_t ahb_freq = cksys_freq >> AHB_EXP[ahb_prescaler_idx];

    // Calculate APB1 frequency
    uint32_t apb1_prescaler_idx = read_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::APB1PSC));
    uint32_t apb1_freq = ahb_freq >> APB1_EXP[apb1_prescaler_idx];

    // Calculate APB2 frequency
    uint32_t apb2_prescaler_idx = read_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::APB2PSC));
    uint32_t apb2_freq = ahb_freq >> APB2_EXP[apb2_prescaler_idx];

    // Return requested clock frequency
    switch (clock) {
        case Clock_Frequency::CK_SYS: return cksys_freq;
        case Clock_Frequency::CK_AHB: return ahb_freq;
        case Clock_Frequency::CK_APB1: return apb1_freq;
        case Clock_Frequency::CK_APB2: return apb2_freq;
        default: return 0U;  // Handle invalid clock type
    }
}

/**
 * @brief Sets the bypass mode enable for the specified oscillator source.
 *
 * This function sets the bypass mode enable for the specified oscillator source, given as an
 * OSCI_Select enumeration. The function takes a boolean parameter enable, which determines whether
 * the bypass mode should be enabled (true) or disabled (false). The function then looks up the
 * register offset and bit information associated with the oscillator source in the osci_select_index
 * array and uses the write_bit_range function to set the appropriate bits in the register.
 *
 * @param[in] osci The oscillator source to set the bypass mode for, represented by the OSCI_Select
 *                 enumeration. Only HXTAL or LXTAL support this feature.
 * @param[in] enable A boolean value indicating whether the bypass mode should be enabled (true) or
 *                   disabled (false).
 */
void RCU::set_bypass_mode_enable(OSCI_Select osci, bool enable) {
    if (osci != OSCI_Select::HXTAL && osci != OSCI_Select::LXTAL) {
        return;
    }
    const RCU_Regs reg = (osci == OSCI_Select::HXTAL) ? RCU_Regs::CTL : RCU_Regs::BDCTL;
    const uint32_t bits = (osci == OSCI_Select::HXTAL) ? static_cast<uint32_t>(CTL_Bits::HXTALBPS) : static_cast<uint32_t>(BDCTL_Bits::LXTALBPS);
    set_osci_enable(osci, false);
    write_bit(*this, reg, bits, enable);
}

/**
 * @brief Sets the IRC8M adjustment value.
 *
 * This function sets the IRC8M adjustment value to the given value. The value is written to the
 * IRC8MADJ field of the CTL register. The IRC8MADJ field is used to adjust the frequency of the
 * IRC8M oscillator. The adjustment value ranges from 0 to 63, and it should be set to the value
 * recommended by the device datasheet.
 *
 * @param[in] value The IRC8M adjustment value (0 to 63)
 */
void RCU::set_irc8m_adjustment_value(uint32_t value) {
    write_bit_range(*this, RCU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::IRC8MADJ), static_cast<uint32_t>(value));
}

/**
 * @brief Enables or disables the HXTAL monitor function.
 *
 * This function enables or disables the HXTAL monitor function. When the HXTAL monitor is enabled, the
 * HXTAL will be monitored and an interrupt will be generated if the HXTAL frequency is lower than the
 * threshold value set by the HXTALCFG register.
 *
 * @param[in] enable A boolean value indicating whether the HXTAL monitor should be enabled (true) or
 *                   disabled (false).
 */
void RCU::set_hxtal_monitor_enable(bool enable) {
    write_bit(*this, RCU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::CKMEN), enable);
}

/**
 * @brief Sets the deep sleep voltage.
 *
 * This function configures the deep sleep voltage level by writing the specified
 * voltage value to the DSLPVS field in the DSV register. The voltage level is
 * defined by the DeepSleep_Voltage enum and determines the voltage used during
 * deep sleep mode.
 *
 * @param[in] voltage The desired deep sleep voltage level, specified by the
 *                    DeepSleep_Voltage enum.
 */
void RCU::set_deep_sleep_voltage(DeepSleep_Voltage voltage) {
    write_bit_range(*this, RCU_Regs::DSV, static_cast<uint32_t>(DSV_Bits::DSLPVS), static_cast<uint32_t>(voltage));
}

/**
 * @brief Retrieves the status of a specified flag in the RCU.
 *
 * This function reads the flag from the corresponding register and returns its status.
 *
 * @param flag The flag to retrieve, as a Status_Flags enumeration value.
 * @return true if the flag is set, false otherwise.
 */
bool RCU::get_flag(Status_Flags flag) {
    return get_value(flag);
}

/**
 * @brief Clears all reset flags in the RCU.
 *
 * This function writes to the RSTSCK register to clear all reset flags.
 * It is used to reset the status flags indicating the cause of the most
 * recent reset event, providing a clean state for subsequent operations.
 */
void RCU::clear_all_reset_flags() {
    write_bit(*this, RCU_Regs::RSTSCK, static_cast<uint32_t>(RSTSCK_Bits::RSTFC), true);
}

/**
 * @brief Checks the status of a specified interrupt flag in the RCU.
 *
 * This function retrieves the current status of a given interrupt flag
 * within the RCU. The status indicates whether the interrupt flag is
 * currently set or cleared.
 *
 * @param flag The interrupt flag to check, specified as an Interrupt_Flags
 *             enumeration value.
 * @return true if the specified interrupt flag is set, false otherwise.
 */
bool RCU::get_interrupt_flag(Interrupt_Flags flag) {
    return get_value(flag);
}

/**
 * @brief Clears a specified interrupt flag in the RCU.
 *
 * This function clears a given interrupt flag specified by the Clear_Flags enumeration.
 * The flag is cleared by writing a 1 to the corresponding bit in the register specified
 * by the clear_flag_index array.
 *
 * @param[in] flag The interrupt flag to clear, specified as a Clear_Flags enumeration value.
 */
void RCU::clear_interrupt_flag(Clear_Flags flag) {
    const auto& info = clear_flag_index[static_cast<size_t>(flag)];
    write_bit_range(*this, info.register_offset, info.bit_info, Set);
}

/**
 * @brief Enables or disables the specified interrupt type in the RCU.
 *
 * This function enables or disables the specified interrupt type in the RCU.
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
void RCU::set_interrupt_enable(Interrupt_Type type, bool enable) {
    const auto &info = interrupt_type_index[static_cast<size_t>(type)];
    write_bit_range(*this, info.register_offset, info.bit_info, enable ? Set : Clear);
}

/**
 * @brief Updates the SystemCoreClock variable based on the current system clock source
 *
 * This function is called whenever the system clock source changes. It reads the current
 * system clock source and updates the SystemCoreClock variable accordingly. The SystemCoreClock
 * variable is used by the system to determine the clock frequency of the processor.
 *
 * The SystemCoreClock value is calculated as follows:
 *  - For IRC8M source: IRC8M_VALUE
 *  - For HXTAL source: HXTAL_VALUE
 *  - For PLL source: calculated by calling calculate_pll_frequency()
 *  - For invalid source: IRC8M_VALUE
 *
 * The SystemCoreClock value is then divided by the AHB clock prescaler to get the final
 * value. The AHB clock prescaler is read from the CFG0 register.
 */
void RCU::update_system_clock() {
    System_Clock_Source current_source = get_system_source();

    switch (current_source) {
        case System_Clock_Source::SOURCE_IRC8M: SystemCoreClock = IRC8M_VALUE; break;
        case System_Clock_Source::SOURCE_HXTAL: SystemCoreClock = HXTAL_VALUE; break;
        case System_Clock_Source::SOURCE_PLL: SystemCoreClock = calculate_pll_frequency(); break;
        case System_Clock_Source::SOURCE_INVALID: default: SystemCoreClock = IRC8M_VALUE; break;
    }

    uint32_t ahb_bit = read_bit_range(*this, RCU_Regs::CFG0, static_cast<uint32_t>(CFG0_Bits::AHBPSC));
    SystemCoreClock >>= AHBPrescaler[static_cast<size_t>(ahb_bit)];
}


} // namespace rcu

rcu::RCU& RCU_I = rcu::RCU::get_instance();
