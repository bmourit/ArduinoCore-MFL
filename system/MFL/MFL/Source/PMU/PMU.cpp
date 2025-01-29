//
// MFL gd32f30x PMU peripheral register access in C++
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

#include "PMU.hpp"
#include "F303RE.hpp"
#include "RCU.hpp"

namespace pmu {

PMU& PMU::get_instance() {
    static PMU instance;
    return instance;
}

PMU::PMU() : is_clock_enabled_(false) {
    if (!is_clock_enabled_) {
        RCU_I.set_pclk_enable(rcu::RCU_PCLK::PCLK_PMU, true);
        RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_PMURST, true);
        RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_PMURST, false);
        is_clock_enabled_ = true;
    }
}

/**
 * @brief Resets the PMU peripheral by toggling the reset control.
 * 
 * This function enables the peripheral clock reset for the PMU,
 * then disables it, effectively resetting all registers to their
 * default values.
 */
void PMU::reset() {
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_PMURST, true);
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_PMURST, false);
}

/**
 * @brief Enables low voltage detection (LVD) for the PMU with the specified threshold.
 * 
 * @param threshold The LVD threshold level to be set, defined by the LVD_Threshold enum.
 * 
 * This function enables the LVD by setting the LVDEN bit in the CTL register.
 * The LVDT bits are configured to the specified threshold level.
 */
void PMU::lvd_enable(LVD_Threshold threshold) {
    // Reset
    write_bits_sequence(*this, PMU_Regs::CTL,
               static_cast<uint32_t>(CTL_Bits::LVDEN), false,
               static_cast<uint32_t>(CTL_Bits::LVDEN), true);
    write_bit_ranges(*this, PMU_Regs::CTL,
               static_cast<uint32_t>(CTL_Bits::LVDT), Clear,
               static_cast<uint32_t>(CTL_Bits::LVDT), static_cast<uint32_t>(threshold));
}

/**
 * @brief Disables low voltage detection (LVD) for the PMU.
 *
 * This function clears the LVDEN bit in the CTL register, disabling
 * the LVD feature. The LVDT bits remain unchanged.
 */
void PMU::lvd_disable() {
    write_bit(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::LVDEN), false);
}

/**
 * @brief Sets the output voltage level for the low-dropout regulator (LDO).
 * 
 * @param level The output voltage level to be set, defined by the Output_Voltage enum.
 * 
 * This function sets the LDOVS bits in the CTL register to the specified output voltage level.
 * The LDOVS bits control the output voltage level of the LDO, which is used to power the CPU, peripherals,
 * and other components on the board.
 */
void PMU::set_ldo_output(Output_Voltage level) {
    write_bit_range(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::LDOVS), static_cast<uint32_t>(level));
}

/**
 * @brief Switches the high driver mode on or off.
 *
 * @param enable If true, enables high driver mode. If false, disables high driver mode.
 *
 * This function sets the HDS bit in the CTL register to the specified value.
 * It then waits for the HDSR flag to become set, indicating that the high driver has finished switching.
 */
void PMU::high_driver_switch(bool enable) {
    write_bit(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::HDS), enable);
    while (get_flag(Status_Flags::HDSR_FLAG) == false) {
        // Just wait
    }
}

/**
 * @brief Enables or disables high driver mode for the PMU.
 *
 * If `enable` is true, this function sets the HDEN bit in the CTL register to enable
 * high driver mode. It then waits for the HDR flag to become set, indicating that the
 * high driver has finished switching.
 *
 * If `enable` is false, this function clears the HDEN bit in the CTL register to disable
 * high driver mode.
 *
 * @param enable If true, enables high driver mode. If false, disables high driver mode.
 */
void PMU::set_high_driver_enable(bool enable) {
    write_bit(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::HDEN), enable);
    if (enable) {
        while (get_flag(Status_Flags::HDR_FLAG) == false) {
        }
    }
}

/**
 * @brief Enables or disables low driver mode during deep sleep for the PMU.
 *
 * This function sets the LDEN bits in the CTL register to enable or disable
 * low driver mode when the system enters deep sleep. When enabled, the low
 * driver mode helps reduce power consumption during deep sleep states.
 *
 * @param enable Set to true to enable low driver mode during deep sleep, 
 *               false to disable it.
 */
void PMU::set_low_driver_on_deep_sleep_enable(bool enable) {
    write_bit_range(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::LDEN), enable ? 3U : Clear);
}

/**
 * @brief Configures the low driver mode during deep sleep.
 *
 * This function sets the LDEN bits in the CTL register based on the provided
 * low driver mode. It allows for enabling or disabling low driver mode during
 * deep sleep, which can help in optimizing power consumption.
 *
 * @param mode The mode to set for low driver during deep sleep, specified by
 *             the Low_Driver enum.
 */
void PMU::set_low_driver_on_deep_sleep(Low_Driver mode) {
    write_bit_range(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::LDEN), static_cast<uint32_t>(mode));
}

/**
 * Sets the driver mode for low power operation.
 *
 * This function configures the PMU to operate in either normal or low driver mode
 * during low power states. It writes to the LDLP bit in the CTL register based on
 * the specified driver mode.
 *
 * @param driver The driver mode to set, defined by the Power_Driver enum. If set to
 *               LOW_DRIVER, the low power driver mode is enabled; otherwise, the
 *               normal driver mode is used.
 */
void PMU::set_driver_on_low_power(Power_Driver driver) {
    write_bit(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::LDLP), (driver == Power_Driver::LOW_DRIVER));
}

/**
 * Sets the driver mode for normal power operation.
 *
 * This function configures the PMU to operate in either normal or low driver mode
 * during normal power states. It writes to the LDNP bit in the CTL register based on
 * the specified driver mode.
 *
 * @param driver The driver mode to set, defined by the Power_Driver enum. If set to
 *               LOW_DRIVER, the low power driver mode is enabled; otherwise, the
 *               normal driver mode is used.
 */
void PMU::set_driver_on_normal_power(Power_Driver driver) {
    write_bit(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::LDNP), (driver == Power_Driver::LOW_DRIVER));
}

/**
 * Enables standby mode for the PMU.
 *
 * This function enables the standby mode by setting the STBMOD and WURST bits in the
 * CTL register. It then calls set_standby_mode() to put the PMU into standby mode.
 * This mode is a low power state that can be exited by either a reset or a wakeup
 * event.
 */
void PMU::set_standby_enable() {
    write_bits_sequence(*this, PMU_Regs::CTL,
               static_cast<uint32_t>(CTL_Bits::STBMOD), true,
               static_cast<uint32_t>(CTL_Bits::WURST), true);

    set_standby_mode();
}

/**
 * Enables sleep mode for the PMU.
 *
 * This function encodes the sleep mode command, specified by the PMU_Commands enum,
 * into a uint8_t value and calls set_sleep_mode_command() to set the sleep mode.
 *
 * @param cmd The sleep mode command to set, defined by the PMU_Commands enum.
 *            If set to WFI_CMD, the PMU will enter sleep mode with the WFI command.
 *            If set to WFE_CMD, the PMU will enter sleep mode with the WFE command.
 */
void PMU::set_sleep_enable(PMU_Commands cmd) {
    // encode the cmd as uint8_t value
    uint8_t value = (cmd == PMU_Commands::WFI_CMD) ? 1 : 2;
    set_sleep_mode_command(value);
}

/**
 * Enables deep sleep mode for the PMU.
 *
 * This function sets the low driver mode bits in the CTL register based on the
 * provided power driver mode. It then calls set_deep_sleep_mode_command() to set
 * the deep sleep mode command. If the PMU is not already in deep sleep mode, it
 * will exit deep sleep mode by setting the STBMOD bit to false.
 *
 * @param driver The driver mode to set, defined by the Power_Driver enum. If set to
 *               NORMAL_DRIVER, the normal driver mode is used; if set to
 *               LOW_DRIVER, the low power driver mode is enabled.
 * @param cmd The deep sleep command to set, defined by the PMU_Commands enum. If set to
 *            WFI_CMD, the PMU will enter deep sleep mode with the WFI command. If set to
 *            WFE_CMD, the PMU will enter deep sleep mode with the WFE command.
 * @param enable If true, enables deep sleep mode; if false, disables deep sleep mode.
 */
void PMU::set_deep_sleep_enable(Power_Driver driver, PMU_Commands cmd, bool enable) {
    uint8_t value = (cmd == PMU_Commands::WFI_CMD) ? 1 : 2;

    write_bit_range(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::LDEN), Clear);
    write_bits_sequence(*this, PMU_Regs::CTL,
               static_cast<uint32_t>(CTL_Bits::STBMOD), false,
               static_cast<uint32_t>(CTL_Bits::LDOLP), false,
               static_cast<uint32_t>(CTL_Bits::LDNP), false,
               static_cast<uint32_t>(CTL_Bits::LDLP), false,
               static_cast<uint32_t>(CTL_Bits::LDOLP), (driver == Power_Driver::LOW_DRIVER));
    // low drive mode config in deep-sleep mode
    if (enable) {
        if (driver == Power_Driver::NORMAL_DRIVER) {
            write_bit_range(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::LDEN), Set);
            write_bit(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::LDNP), true);
        } else {
            write_bit_ranges(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::LDEN), Set);
            write_bit(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::LDLP), true);
        }
    }

    set_deep_sleep_mode_command(value);
}

/**
 * Enables or disables the PMU's wakeup pin.
 *
 * @param enable Set to true to enable the PMU's wakeup pin, false to disable it.
 */
void PMU::set_wakeup_pin_enable(bool enable) {
    write_bit(*this, PMU_Regs::CS, static_cast<uint32_t>(CS_Bits::WUPEN), enable);
}

/**
 * Enables or disables write access to the backup domain.
 *
 * This function sets or clears the BKPWEN bit in the CTL register
 * to enable or disable write access to the backup domain.
 *
 * @param enable Set to true to enable write access, or false to disable it.
 */
void PMU::set_backup_write_enable(bool enable) {
    write_bit(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::BKPWEN), enable);
}

/**
 * Retrieves the status of a specified flag in the PMU.
 *
 * This function reads the flag from the CS register and returns its status.
 *
 * @param flag The flag to retrieve, as a Status_Flags enumeration value.
 * @return true if the flag is set, false otherwise.
 */
bool PMU::get_flag(Status_Flags flag) {
    return read_bit(*this, PMU_Regs::CS, static_cast<uint32_t>(flag));
}

/**
 * Clears a specified flag in the PMU.
 *
 * This function clears a given flag specified by the Clear_Flags enumeration.
 * The flag is cleared by writing a 1 to the corresponding bit in the CTL register.
 *
 * @param flag The flag to clear, specified as a Clear_Flags enumeration value.
 */
void PMU::clear_flag(Clear_Flags flag) {
    if (flag == Clear_Flags::RESET_WAKEUP_FLAG) {
        write_bit(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::WURST), true);
    } else if (flag == Clear_Flags::RESET_STANDBY_FLAG) {
        write_bit(*this, PMU_Regs::CTL, static_cast<uint32_t>(CTL_Bits::STBRST), true);
    }
}


} // namespace pmu

pmu::PMU& PMU_I = pmu::PMU::get_instance();


extern "C" {

    /**
     * Sets the sleep mode command for the PMU.
     *
     * @param value The sleep mode command value. 1 for WFI, 2 for WFE.
     *
     * This function sets the sleep mode command by calling either the __WFI() or
     * __WFE() intrinsic functions. If the value is invalid, the function will return
     * without doing anything.
     */
    void set_sleep_mode_command(uint8_t value) {
        if ((value < 1) || (value > 2)) {
            return;
        }

        // Clear sleepdeep bit of Cortex-M4 SCR
        SCB->SCR &= ~(static_cast<uint32_t>(SCB_SCR_SLEEPDEEP_Msk));

        if (value == 1) {
            __WFI();
        } else if (value == 2) {
            __WFE();
        }
    }

    /**
     * Sets the deep sleep mode command for the PMU.
     *
     * This function sets the deep sleep mode command by calling either the __WFI() or
     * __WFE() intrinsic functions. If the value is invalid, the function will return
     * without doing anything.
     *
     * Before calling the deep sleep command, the function will save the current
     * values of the registers at addresses 0xE000E010, 0xE000E100, 0xE000E104, and
     * 0xE000E108, and then restore them after waking up from deep sleep mode.
     *
     * @param value The deep sleep mode command value. 1 for WFI, 2 for WFE.
     */
    void set_deep_sleep_mode_command(uint8_t value) {
        if ((value < 1) || (value > 2)) {
            return;
        }

        static uint32_t register_snapshot[4];

        // Set sleepdeep bit of Cortex-M4 SCR
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

        register_snapshot[0] = (*(volatile uint32_t *)(0xE000E010));
        register_snapshot[1] = (*(volatile uint32_t *)(0xE000E100));
        register_snapshot[2] = (*(volatile uint32_t *)(0xE000E104));
        register_snapshot[3] = (*(volatile uint32_t *)(0xE000E108));

        (*(volatile uint32_t *)(0xE000E010)) &= 0x00010004;
        (*(volatile uint32_t *)(0xE000E180)) = 0XFF7FF83D;
        (*(volatile uint32_t *)(0xE000E184)) = 0XFFFFF8FF;
        (*(volatile uint32_t *)(0xE000E188)) = 0xFFFFFFFF;

        // select WFI or WFE command to enter deepsleep mode
        if (value == 1) {
            __WFI();
        } else if (value == 2) {
            __SEV();
            __WFE();
            __WFE();
        }

        (*(volatile uint32_t *)(0xE000E010)) = register_snapshot[0];
        (*(volatile uint32_t *)(0xE000E100)) = register_snapshot[1];
        (*(volatile uint32_t *)(0xE000E104)) = register_snapshot[2];
        (*(volatile uint32_t *)(0xE000E108)) = register_snapshot[3];

        // Reset sleepdeep bit of Cortex-M4 SCR
        SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
    }

    /**
     * Puts the device into standby mode.
     *
     * This function puts the device into standby mode by setting the
     * sleepdeep bit in the System Control Register (SCB->SCR) and
     * configuring the System Tick Timer (SYSTICK) to wake up the
     * device when a System Tick interrupt occurs.
     *
     * The device will wake up when a System Tick interrupt occurs
     * and the WAKEUP bit in the SYSTICK Control and Status Register
     * (SYSTICK_CTRL) is set.
     */
    void set_standby_mode(void) {
        // Set sleepdeep bit of Cortex-M4 SCR
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

        (*(volatile uint32_t *)(0xE000E010U)) &= 0x00010004U;
        (*(volatile uint32_t *)(0xE000E180U)) = 0XFFFFFFF7U;
        (*(volatile uint32_t *)(0xE000E184U)) = 0XFFFFFDFFU;
        (*(volatile uint32_t *)(0xE000E188U)) = 0xFFFFFFFFU;

        __WFI();
    }

} // extern "C"
