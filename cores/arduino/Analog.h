
#pragma once

#include "Arduino.h"
#include "GeneralTimer.h"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"

#define DAC_PERIPH_COUNT    2
#define ADC_PERIPH_COUNT    3

#ifndef ADC_RESOLUTION
    #define ADC_RESOLUTION      12
#endif
#ifndef PWM_RESOLUTION
    #define PWM_RESOLUTION      8
#endif
#ifndef PWM_MAX_RESOLUTION
    #define PWM_MAX_RESOLUTION  16
#endif
#ifndef ADC_MAX_RESOLUTION
    #define ADC_MAX_RESOLUTION  12
#endif

// ADC
uint32_t getAdcValue(pin_size_t pin, uint32_t resolution);

// DAC
void setDacValue(pin_size_t pin, uint16_t value, bool is_initialized);
void dacStop(pin_size_t pin);

// PWM
void pwmStart(pin_size_t pin, uint32_t frequency, uint32_t value, CCFormat format);
void pwmStop(pin_size_t pin);

// Helpers

/**
 * Retrieves the ADC channel for a given pin.
 * @param pin The pin number for which to get the ADC channel
 * @return ADC_Channel enum value corresponding to the pin, or INVALID if no valid channel exists
 */
inline const adc::ADC_Channel getAdcChannel(pin_size_t pin) {
    static const adc::ADC_Channel channels[] = {
        adc::ADC_Channel::CHANNEL_0, adc::ADC_Channel::CHANNEL_1, adc::ADC_Channel::CHANNEL_2,
        adc::ADC_Channel::CHANNEL_3, adc::ADC_Channel::CHANNEL_4, adc::ADC_Channel::CHANNEL_5,
        adc::ADC_Channel::CHANNEL_6, adc::ADC_Channel::CHANNEL_7, adc::ADC_Channel::CHANNEL_8,
        adc::ADC_Channel::CHANNEL_9, adc::ADC_Channel::CHANNEL_10, adc::ADC_Channel::CHANNEL_11,
        adc::ADC_Channel::CHANNEL_12, adc::ADC_Channel::CHANNEL_13, adc::ADC_Channel::CHANNEL_14,
        adc::ADC_Channel::CHANNEL_15
    };
    uint32_t packedPinOps = getPackedPinOps(ADC_PinOps, pin);
    if (packedPinOps == invalidValue) {
        return adc::ADC_Channel::INVALID;
    }
    uint8_t channel = getPackedPinChannel(packedPinOps);
    return (channel < 16U) ? channels[channel] : adc::ADC_Channel::INVALID;
}

/**
 * Maps internal ADC pins to their corresponding ADC channels
 * @param pin Internal ADC pin (ADC_TEMP or ADC_VREF)
 * @return ADC channel enum class value for the pin, or INVALID if not an internal pin
 */
inline constexpr adc::ADC_Channel getAdcInternalChannel(pin_size_t pin) {
    return (pin == ADC_TEMP) ? adc::ADC_Channel::CHANNEL_16 :
           (pin == ADC_VREF) ? adc::ADC_Channel::CHANNEL_17 :
           adc::ADC_Channel::INVALID;
}

/**
 * Maps a pin number to its corresponding ADC channel.
 *
 * @param pin Pin number to convert to ADC channel. Can be a regular pin,
 *            ADC_TEMP, or ADC_VREF
 * @return ADC_Channel enum class representing the mapped channel, or INVALID if
 *         pin is out of range
 */
inline adc::ADC_Channel getAdcChannelFromPin(pin_size_t pin) {
    if (pin == static_cast<pin_size_t>(ADC_TEMP) || pin == static_cast<pin_size_t>(ADC_VREF)) {
        return getAdcInternalChannel(pin);
    }
    if (pin <= static_cast<pin_size_t>(TOTAL_PIN_COUNT)) {
        return getAdcChannel(pin);
    }
    return adc::ADC_Channel::INVALID;
}

/**
 * Returns the appropriate ADC sample time based on pin type.
 * @param pin The pin number to check
 * @return ADC_Sample_Time - Returns InternalSampleTime for ADC_TEMP/ADC_VREF pins,
 *         otherwise returns default SampleTime
 */
inline adc::ADC_Sample_Time getAdcSampleTime(pin_size_t pin) {
    return (pin == ADC_TEMP || pin == ADC_VREF) ? InternalSampleTime : SampleTime;
}

/**
 * Converts a numeric ADC resolution value to its corresponding enum class value.
 * @param resolution The bit resolution (6, 8, 10 or 12)
 * @return The corresponding ADC_Resolution enum class value, defaults to 12-bit
 */
inline constexpr adc::ADC_Resolution getAdcResolution(uint32_t resolution) {
    switch (resolution) {
        case 6: return adc::ADC_Resolution::RESOLUTION_6BIT;
        case 8: return adc::ADC_Resolution::RESOLUTION_8BIT;
        case 10: return adc::ADC_Resolution::RESOLUTION_10BIT;
        case 12: return adc::ADC_Resolution::RESOLUTION_12BIT;
        default: return adc::ADC_Resolution::RESOLUTION_12BIT;
    }
}

/**
 * Converts a numeric PWM resolution value to its corresponding CCFormat enum class value.
 * @param resolution The PWM resolution in bits (1-16)
 * @return CCFormat enum class valuere presenting the resolution, or INVALID if out of range
 */
inline CCFormat getPwmResolution(uint32_t resolution) {
    static const CCFormat formats[] = {
        CCFormat::B1, CCFormat::B2, CCFormat::B3, CCFormat::B4,
        CCFormat::B5, CCFormat::B6, CCFormat::B7, CCFormat::B8,
        CCFormat::B9, CCFormat::B10, CCFormat::B11, CCFormat::B12,
        CCFormat::B13, CCFormat::B14, CCFormat::B15, CCFormat::B16
    };
    return (resolution > 0 && resolution <= 16) ? formats[resolution - 1] : CCFormat::INVALID;
}

/**
 * Maps a value from one resolution to another resolution.
 *
 * @param value The input value to map
 * @param from Source resolution in bits
 * @param to Target resolution in bits
 * @return The mapped value in the target resolution
 */
inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) {
    if (from == to) {
        return value;
    }

    if (from > to) {
        uint32_t shift = from - to;
        return (value < (1U << shift)) ? 0U : ((value + 1U) >> shift) - 1U;
    }

    return (value != 0) ? ((value + 1U) << (to - from)) - 1U : 0U;
}
