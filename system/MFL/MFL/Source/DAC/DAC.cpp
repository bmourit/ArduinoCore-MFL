//
// MFL gd32f30x DAC peripheral register access in C++
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

#include "DAC.hpp"
#include "RCU.hpp"

namespace dac {

DAC& DAC::get_instance() {
    static DAC instance;
    return instance;
}

DAC::DAC() : is_clock_enabled_(false) {
    if (!is_clock_enabled_) {
        RCU_I.set_pclk_enable(rcu::RCU_PCLK::PCLK_DAC, true);
        RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_DACRST, true);
        RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_DACRST, false);
        is_clock_enabled_ = true;
    }
}

/**
 * Resets the DAC peripheral by toggling the reset control.
 * This function enables the peripheral clock reset for the DAC,
 * then disables it, effectively resetting all registers to their
 * default values.
 */
void DAC::reset() {
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_DACRST, true);
    RCU_I.set_pclk_reset_enable(rcu::RCU_PCLK_Reset::PCLK_DACRST, false);
}

/**
 * @brief Enables the specified DAC internal device.
 *
 * This function sets the enable bit in the DAC control register for the 
 * specified internal device. If the device is invalid, the function returns
 * without making any changes.
 *
 * @param instance The internal DAC device to enable. Must be a value from 
 *                 the Internal_Device enumeration.
 */
void DAC::enable(Internal_Device instance) {
    if (instance == Internal_Device::INVALID) {
        return;
    }

    const uint32_t bits = (instance == Internal_Device::DAC0_BASE) ?
                            static_cast<uint32_t>(CTL_Bits::DEN0) :
                            static_cast<uint32_t>(CTL_Bits::DEN1);

    write_bit(*this, DAC_Regs::CTL, bits, true);
}

/**
 * @brief Disables the specified DAC internal device.
 *
 * This function clears the enable bit in the DAC control register for the 
 * specified internal device. If the device is invalid, the function returns
 * without making any changes.
 *
 * @param instance The internal DAC device to disable. Must be a value from 
 *                 the Internal_Device enumeration.
 */
void DAC::disable(Internal_Device instance) {
    if (instance == Internal_Device::INVALID) {
        return;
    }

    const uint32_t bits = (instance == Internal_Device::DAC0_BASE) ?
                            static_cast<uint32_t>(CTL_Bits::DEN0) :
                            static_cast<uint32_t>(CTL_Bits::DEN1);

    write_bit(*this, DAC_Regs::CTL, bits, false);
}

void DAC::set_enable(Internal_Device instance, bool enable) {
    if (instance == Internal_Device::INVALID) {
        return;
    }

    const uint32_t bits = (instance == Internal_Device::DAC0_BASE) ?
                            static_cast<uint32_t>(CTL_Bits::DEN0) :
                            static_cast<uint32_t>(CTL_Bits::DEN1);

    write_bit(*this, DAC_Regs::CTL, bits, enable);
}

/**
 * @brief Enables or disables DMA for the specified DAC internal device.
 *
 * This function sets or clears the DMA enable bit in the DAC control register
 * for the specified internal device. If the device is invalid, the function
 * returns without making any changes.
 *
 * @param instance The internal DAC device to enable or disable DMA for. Must
 *                 be a value from the Internal_Device enumeration.
 * @param enable    Set to true to enable DMA, false to disable it.
 */
void DAC::set_dma_enable(Internal_Device instance, bool enable) {
    if (instance == Internal_Device::INVALID) {
        return;
    }

    const uint32_t bits = (instance == Internal_Device::DAC0_BASE) ?
                            static_cast<uint32_t>(CTL_Bits::DDMAEN0) :
                            static_cast<uint32_t>(CTL_Bits::DDMAEN1);

    write_bit(*this, DAC_Regs::CTL, bits, enable);
}

/**
 * @brief Enables or disables the output buffer for the specified DAC internal device.
 *
 * This function sets or clears the output buffer enable bit in the DAC control
 * register for the specified internal device. If the device is invalid, the
 * function returns without making any changes.
 *
 * @param instance The internal DAC device to enable or disable the output buffer
 *                 for. Must be a value from the Internal_Device enumeration.
 * @param enable    Set to true to enable the output buffer, false to disable it.
 */
void DAC::set_output_buffer_enable(Internal_Device instance, bool enable) {
    if (instance == Internal_Device::INVALID) {
        return;
    }

    const uint32_t bits = (instance == Internal_Device::DAC0_BASE) ?
                            static_cast<uint32_t>(CTL_Bits::DBOFF0) :
                            static_cast<uint32_t>(CTL_Bits::DBOFF1);

    write_bit(*this, DAC_Regs::CTL, bits, enable);
}

/**
 * @brief Returns the current output value of the specified DAC internal device.
 *
 * This function reads the output value of the specified internal device and
 * returns it as a 16-bit unsigned integer. If the internal device is invalid,
 * the function returns zero.
 *
 * @param instance The internal DAC device to retrieve the output value from.
 *                 Must be a value from the Internal_Device enumeration.
 *
 * @return The current output value of the specified DAC internal device, or zero
 *         if the device is invalid.
 */
uint16_t DAC::get_output_value(Internal_Device instance) {
    if (instance == Internal_Device::INVALID) {
        return 0;
    }

    const DAC_Regs reg = (instance == Internal_Device::DAC0_BASE) ?
                            DAC_Regs::DAC0_DO :
                            DAC_Regs::DAC1_DO;

    const uint32_t bits = (instance == Internal_Device::DAC0_BASE) ?
                            static_cast<uint32_t>(DAC0_DO_Bits::DAC0_DO) :
                            static_cast<uint32_t>(DAC1_DO_Bits::DAC1_DO);

    return read_bit16_range(*this, reg, bits);
}

/**
 * @brief Sets the data value for the specified DAC internal device with the specified alignment.
 *
 * This function writes a 12-bit or 8-bit data value to the appropriate data holding register
 * of the specified DAC internal device, based on the specified data alignment. The data can be
 * right-aligned in 12 bits, left-aligned in 12 bits, or right-aligned in 8 bits.
 *
 * @param instance The internal DAC device to set the data for. Must be a value from the
 *                 Internal_Device enumeration.
 * @param align The data alignment for the DAC output. Must be a value from the Data_Align
 *              enumeration.
 * @param data The data value to write to the DAC. The format of this data depends on the
 *             specified alignment.
 */
void DAC::set_data(Internal_Device instance, Data_Align align, uint16_t data) {
    if (instance == Internal_Device::INVALID) {
        return;
    }

    constexpr DAC_Regs R12DH_Regs[] = {
        DAC_Regs::DAC0_R12DH,
        DAC_Regs::DAC1_R12DH
    };

    constexpr uint32_t R12DH_Bits[] = {
        static_cast<uint32_t>(DAC0_R12DH_Bits::DAC0_DH),
        static_cast<uint32_t>(DAC1_R12DH_Bits::DAC1_DH)
    };

    constexpr DAC_Regs L12DH_Regs[] = {
        DAC_Regs::DAC0_L12DH,
        DAC_Regs::DAC1_L12DH
    };

    constexpr uint32_t L12DH_Bits[] = {
        static_cast<uint32_t>(DAC0_L12DH_Bits::DAC0_DH),
        static_cast<uint32_t>(DAC1_L12DH_Bits::DAC1_DH)
    };

    constexpr DAC_Regs R8DH_Regs[] = {
        DAC_Regs::DAC0_R8DH,
        DAC_Regs::DAC1_R8DH
    };

    constexpr uint32_t R8DH_Bits[] = {
        static_cast<uint32_t>(DAC0_R8DH_Bits::DAC0_DH),
        static_cast<uint32_t>(DAC1_R8DH_Bits::DAC1_DH)
    };

    switch (align) {
    case Data_Align::RIGHT_12B:
        write_bit_range(*this, R12DH_Regs[static_cast<size_t>(instance)], R12DH_Bits[static_cast<size_t>(instance)], static_cast<uint32_t>(data));
        break;
    case Data_Align::LEFT_12B:
        write_bit_range(*this, L12DH_Regs[static_cast<size_t>(instance)], L12DH_Bits[static_cast<size_t>(instance)], static_cast<uint32_t>(data));
        break;
    case Data_Align::RIGHT_8B:
        write_bit_range(*this, R8DH_Regs[static_cast<size_t>(instance)], R8DH_Bits[static_cast<size_t>(instance)], static_cast<uint32_t>(data));
        break;
    default:
        break;
    }
}

/**
 * @brief Enables or disables the external trigger for the specified DAC internal device.
 *
 * This function sets or clears the trigger enable bit in the DAC control register
 * for the specified internal device. If the device is invalid, the function
 * returns without making any changes.
 *
 * @param instance The internal DAC device to enable or disable the external trigger
 *                 for. Must be a value from the Internal_Device enumeration.
 * @param enable    Set to true to enable the external trigger, false to disable it.
 */
void DAC::set_trigger_enable(Internal_Device instance, bool enable) {
    if (instance == Internal_Device::INVALID) {
        return;
    }

    const uint32_t bits = (instance == Internal_Device::DAC0_BASE) ?
                            static_cast<uint32_t>(CTL_Bits::DTEN0) :
                            static_cast<uint32_t>(CTL_Bits::DTEN1);

    write_bit(*this, DAC_Regs::CTL, bits, enable);
}

/**
 * @brief Sets the external trigger source for the specified DAC internal device.
 *
 * This function sets the trigger source for the specified internal device. The
 * trigger source is an external event that triggers the DAC conversion. The
 * external trigger source can be either the Timer CH0 event, the Timer CH1 event,
 * or the Timer CH2 event. If the device is invalid, the function returns without
 * making any changes.
 *
 * @param instance The internal DAC device to set the external trigger source for.
 *                 Must be a value from the Internal_Device enumeration.
 * @param source    The external trigger source to set for the specified internal
 *                  device. Must be a value from the Trigger_Source enumeration.
 */
void DAC::set_trigger_source(Internal_Device instance, Trigger_Source source) {
    if (instance == Internal_Device::INVALID) {
        return;
    }

    const uint32_t bits = (instance == Internal_Device::DAC0_BASE) ?
                            static_cast<uint32_t>(CTL_Bits::DTSEL0) :
                            static_cast<uint32_t>(CTL_Bits::DTSEL1);

    write_bit_range(*this, DAC_Regs::CTL, bits, static_cast<uint32_t>(source));
}

/**
 * @brief Sets the software trigger enable for the specified internal DAC device.
 *
 * This function sets the software trigger enable bit for the specified internal
 * device. The software trigger is an internal event that triggers the DAC
 * conversion. The software trigger enable bit determines whether the DAC
 * conversion is triggered by the software trigger or not. If the device is
 * invalid, the function returns without making any changes.
 *
 * @param instance The internal DAC device to set the software trigger enable for.
 *                 Must be a value from the Internal_Device enumeration.
 * @param enable    Set to true to enable the software trigger, false to disable it.
 */
void DAC::set_software_trigger_enable(Internal_Device instance, bool enable) {
    if (instance == Internal_Device::INVALID) {
        return;
    }

    const uint32_t bits = (instance == Internal_Device::DAC0_BASE) ?
                            static_cast<uint32_t>(SWT_Bits::SWTR0) :
                            static_cast<uint32_t>(SWT_Bits::SWTR1);

    write_bit(*this, DAC_Regs::SWT, bits, enable);
}

/**
 * @brief Sets the wave mode for the specified internal DAC device.
 *
 * This function sets the wave mode for the specified internal device. The wave
 * mode determines how the DAC converts the data value. The wave mode can be
 * either the noise mode or the triangular mode. If the device is invalid, the
 * function returns without making any changes.
 *
 * @param instance The internal DAC device to set the wave mode for. Must be a
 *                 value from the Internal_Device enumeration.
 * @param type     The wave mode to set for the specified internal device. Must
 *                 be a value from the Wave_Type enumeration.
 */
void DAC::set_wave_mode(Internal_Device instance, Wave_Type type) {
    if (instance == Internal_Device::INVALID) {
        return;
    }

    const uint32_t bits = (instance == Internal_Device::DAC0_BASE) ?
                            static_cast<uint32_t>(CTL_Bits::DWM0) :
                            static_cast<uint32_t>(CTL_Bits::DWM1);

    write_bit_range(*this, DAC_Regs::CTL, bits, static_cast<uint32_t>(type));
}

/**
 * @brief Sets the wave bit width for the specified internal DAC device.
 *
 * This function sets the bit width for the specified internal device when
 * using the wave mode. The bit width determines how many bits are used to
 * represent the data value. If the device is invalid, the function returns
 * without making any changes.
 *
 * @param instance The internal DAC device to set the wave bit width for. Must
 *                 be a value from the Internal_Device enumeration.
 * @param width    The bit width to set for the specified internal device. Must
 *                 be a value from the Bit_Width enumeration.
 */
void DAC::set_wave_bit_width(Internal_Device instance, Bit_Width width) {
    if (instance == Internal_Device::INVALID) {
        return;
    }

    const uint32_t bits = (instance == Internal_Device::DAC0_BASE) ?
                            static_cast<uint32_t>(CTL_Bits::DWBW0) :
                            static_cast<uint32_t>(CTL_Bits::DWBW1);

    write_bit_range(*this, DAC_Regs::CTL, bits, static_cast<uint32_t>(width));
}

/**
 * @brief Sets the LFSR noise for the specified internal DAC device.
 *
 * This function sets the LFSR noise for the specified internal device when
 * using the wave mode. The LFSR noise determines the type of noise used to
 * generate the wave. If the device is invalid, the function returns without
 * making any changes.
 *
 * @param instance The internal DAC device to set the LFSR noise for. Must be a
 *                 value from the Internal_Device enumeration.
 * @param noise    The LFSR noise to set for the specified internal device. Must
 *                 be a value from the LFSR_Noise enumeration.
 */
void DAC::set_lfsr_noise(Internal_Device instance, LFSR_Noise noise) {
    if (instance == Internal_Device::INVALID) {
        return;
    }

    const uint32_t bits = (instance == Internal_Device::DAC0_BASE) ?
                            static_cast<uint32_t>(CTL_Bits::DWBW0) :
                            static_cast<uint32_t>(CTL_Bits::DWBW1);

    write_bit_range(*this, DAC_Regs::CTL, bits, static_cast<uint32_t>(noise));
}

/**
 * @brief Sets the triangle amplitude for the specified internal DAC device.
 *
 * This function sets the triangle amplitude for the specified internal device
 * when using the wave mode. The triangle amplitude determines the amplitude of
 * the generated triangle wave. If the device is invalid, the function returns
 * without making any changes.
 *
 * @param instance The internal DAC device to set the triangle amplitude for.
 *                 Must be a value from the Internal_Device enumeration.
 * @param amplitude The triangle amplitude to set for the specified internal
 *                 device. Must be a value from the Triangle_Amplitude
 *                 enumeration.
 */
void DAC::set_triangle_amplitude(Internal_Device instance, Triangle_Amplitude amplitude) {
    if (instance == Internal_Device::INVALID) {
        return;
    }

    const uint32_t bits = (instance == Internal_Device::DAC0_BASE) ?
                            static_cast<uint32_t>(CTL_Bits::DWBW0) :
                            static_cast<uint32_t>(CTL_Bits::DWBW1);

    write_bit_range(*this, DAC_Regs::CTL, bits, static_cast<uint32_t>(amplitude));
}

/**
 * @brief Sets the dual mode enable for the DAC.
 *
 * This function sets the dual mode enable for the DAC. When the dual mode is
 * enabled, the DAC0 and DAC1 will be paired together to form a single DAC to
 * output the same data. If the device is invalid, the function returns without
 * making any changes.
 *
 * @param enable Set to true to enable the dual mode, false to disable it.
 */
void DAC::set_dual_mode_enable(bool enable) {
    write_bits_sequence(*this, DAC_Regs::CTL,
               static_cast<uint32_t>(CTL_Bits::DEN0), enable,
               static_cast<uint32_t>(CTL_Bits::DEN1), enable);
}

/**
 * @brief Enables or disables the dual software trigger for the DAC.
 *
 * This function sets the software trigger enable bits for both DAC0 and DAC1.
 * When enabled, it allows both DAC channels to be triggered simultaneously
 * by software. This is useful for applications requiring synchronized output
 * from both DAC channels.
 *
 * @param enable Set to true to enable the dual software trigger, false to disable it.
 */
void DAC::set_dual_software_trigger_enable(bool enable) {
    write_bits_sequence(*this, DAC_Regs::SWT,
               static_cast<uint32_t>(SWT_Bits::SWTR0), enable,
               static_cast<uint32_t>(SWT_Bits::SWTR1), enable);
}

/**
 * @brief Enables or disables the dual output buffer for the DAC.
 *
 * This function sets the output buffer enable bits for both DAC0 and DAC1.
 * When enabled, the output data is latched in the output buffer before being
 * output to the pins. This is useful for applications requiring synchronized
 * output from both DAC channels. When disabled, the output data is sent
 * directly to the pins.
 *
 * @param enable Set to true to enable the output buffer, false to disable it.
 */
void DAC::set_dual_output_buffer_enable(bool enable) {
    write_bits_sequence(*this, DAC_Regs::CTL,
               static_cast<uint32_t>(CTL_Bits::DBOFF0), enable,
               static_cast<uint32_t>(CTL_Bits::DBOFF1), enable);
}

/**
 * @brief Sets the output data for both DAC channels in dual mode.
 *
 * This function sets the output data for both DAC channels based on the
 * specified data alignment. The data can be right-aligned in 12 bits,
 * left-aligned in 12 bits, or right-aligned in 8 bits.
 *
 * @param align The data alignment for the dual DAC output. Must be a value
 *              from the Data_Align enumeration.
 * @param data0 The data value to write to the DAC0 channel.
 * @param data1 The data value to write to the DAC1 channel.
 */
void DAC::set_dual_data(Data_Align align, uint16_t data0, uint16_t data1) {
    switch (align) {
    case Data_Align::RIGHT_12B:
        write_bit_ranges(*this, DAC_Regs::DACC_R12DH,
                   static_cast<uint32_t>(DACC_R12DH_Bits::DAC0_DH), static_cast<uint32_t>(data0),
                   static_cast<uint32_t>(DACC_R12DH_Bits::DAC1_DH), static_cast<uint32_t>(data1));
        break;
    case Data_Align::LEFT_12B:
        write_bit_ranges(*this, DAC_Regs::DACC_L12DH,
                   static_cast<uint32_t>(DACC_L12DH_Bits::DAC0_DH), static_cast<uint32_t>(data0),
                   static_cast<uint32_t>(DACC_L12DH_Bits::DAC1_DH), static_cast<uint32_t>(data1));
        break;
    case Data_Align::RIGHT_8B:
        write_bit_ranges(*this, DAC_Regs::DACC_R8DH,
                   static_cast<uint32_t>(DACC_R8DH_Bits::DAC0_DH), static_cast<uint32_t>(data0),
                   static_cast<uint32_t>(DACC_R8DH_Bits::DAC1_DH), static_cast<uint32_t>(data1));
        break;
    default:
        break;
    }
}

} // namespace dac
