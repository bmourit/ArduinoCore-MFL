//
// MFL gd32f30x STARTUP and clock initialization in C++
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

#include <stdint.h>

#include "STARTUP.hpp"
#include "FMC.hpp"
#include "PMU.hpp"
#include "RCU.hpp"
#include "CONFIG.hpp"
#include "CEE.hpp"

namespace startup {

STARTUP& STARTUP::get_instance() {
    static STARTUP instance;
    return instance;
}

STARTUP::STARTUP() {}

/**
 * @brief Initializes the startup sequence for the GD32F30x MCU family.
 *
 * This function will:
 *  1. Enable the High-Speed Crystal Oscillator (HXTAL)
 *  2. Wait until HXTAL is stable
 *  3. Set the LDO output voltage to HIGH (1.2V). This can be changed
 *     by uncommenting one of the other options below.
 *  4. Set the AHB clock to be the system clock (SYSCLK)
 *  5. Set the APB2 clock to be the system clock (SYSCLK) divided by 1
 *  6. Set the APB1 clock to be the system clock (SYSCLK) divided by 2
 *  7. Configure the PLL clock to be 120 MHz
 *  8. Enable the PLL
 *  9. Wait for the PLL to be stable
 *  10. Enable high-drive mode for high clock frequency
 *  11. Select high-drive mode
 *  12. Set the system clock to be the PLL clock
 *  13. Verify the system clock is set to the PLL clock
 *  14. Set the ADC prescaler to divide the APB2 clock by 8
 *  15. Enable the CEE enhanced mode, when applicable
 *
 * @note This function should be called as early as possible in the startup
 * sequence to ensure proper configuration of the MCU.
 */
void STARTUP::startup_init() {
    // Enable HXTAL
    rcu::RCU::get_instance().set_osci_enable(rcu::OSCI_Select::HXTAL, true);
    // Wait until HXTAL is stable
    while (!rcu::RCU::get_instance().is_osci_stable(rcu::OSCI_Select::HXTAL)) {}

    // TODO: Make this a configurable setting
    //  Note:
    //  According to the manual, this can only be set while main PLL
    //  is off. This setting takes effect on the 1.2V power domain
    //  when the main PLL is on. The manual states the when the main
    //  PLL is off, this is automatically set to LDO_VOLATGE_LOW
    //  This helps lower power requirements at the expense of driving
    //  capabilities.
    //  Since we need PLL off for this, we should make this a
    //  configurable setting for different user requirments.
    //  For now, if you need something different comment out the
    //  default (LOW) and uncomment one of the other choices below.
    //pmu::PMU::get_instance().set_ldo_output(pmu::Output_Voltage::LDO_VOLTAGE_LOW);
    //pmu::PMU::get_instance().set_ldo_output(pmu::Output_Voltage::LDO_VOLTAGE_MID);
    pmu::PMU::get_instance().set_ldo_output(pmu::Output_Voltage::LDO_VOLTAGE_HIGH);

    // AHB = SYSCLK
    rcu::RCU::get_instance().set_ahb_prescaler(rcu::AHB_Prescaler::CKSYS_DIV1);
    rcu::RCU::get_instance().set_apb2_prescaler(rcu::APB_Prescaler::CKAHB_DIV1);
    rcu::RCU::get_instance().set_apb1_prescaler(rcu::APB_Prescaler::CKAHB_DIV2);

    // CK_PLL = (CK_HXTAL / 2) * 30 = 120 MHz
    rcu::RCU::get_instance().set_predv0_config(true);
    rcu::RCU::get_instance().set_pll_config(rcu::PLL_Source::PLLSRC_HXTAL_IRC48M, rcu::PLLMF_Select::PLL_MUL30);

    // Enable PLL
    rcu::RCU::get_instance().set_osci_enable(rcu::OSCI_Select::PLL_CK, true);
    // Wait for PLL to stablize
    while (!rcu::RCU::get_instance().is_osci_stable(rcu::OSCI_Select::PLL_CK)) {}

    // Enable high-drive for high clock frequency
    pmu::PMU::get_instance().set_high_driver_enable(true);
    // Select high-drive mode
    pmu::PMU::get_instance().high_driver_switch(true);

    // PLL as system clock
    rcu::RCU::get_instance().set_system_source(rcu::System_Clock_Source::SOURCE_PLL);
    // Verify PLL is set as system clock
    while (rcu::RCU::get_instance().get_system_source() != rcu::System_Clock_Source::SOURCE_PLL) {}

    // ADC prescaler
    rcu::RCU::get_instance().set_adc_prescaler(rcu::ADC_Prescaler::CKAPB2_DIV8);    // 15 MHz

#ifndef DISABLE_CEE_ENHANCE
    // Set CEE enahnced mode
    cee::CEE::get_instance().set_enhanced_mode_enable(true);
#endif

    // Set the global variable
    rcu::RCU::get_instance().update_system_clock();
}


} // namespace startup
