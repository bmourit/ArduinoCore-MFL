
#include "Analog.h"
#include "PinConfigManager.hpp"

static int pwmResolution_ = PWM_RESOLUTION;
static int readResolution_ = ADC_RESOLUTION;
static int writeResolution_ = PWM_RESOLUTION;
static int internalReadResolution_ = (ADC_RESOLUTION > ADC_MAX_RESOLUTION) ? ADC_MAX_RESOLUTION : ADC_RESOLUTION;
static int internalWriteResolution_ = (PWM_RESOLUTION > PWM_MAX_RESOLUTION) ? PWM_MAX_RESOLUTION : PWM_RESOLUTION;
static uint32_t writeFrequency_ = static_cast<uint32_t>(PWM_FREQUENCY);

static constexpr adc::ADC_Sample_Time SampleTime = adc::ADC_Sample_Time::SAMPLETIME_13_5_CYCLES;
static constexpr adc::ADC_Sample_Time InternalSampleTime = adc::ADC_Sample_Time::SAMPLETIME_239_5_CYCLES;

// ADC

/**
 * @brief Reads the analog value from the specified pin with the given resolution.
 *
 * Performs an ADC conversion on the given pin using the specified resolution.
 * Handles internal ADC channels (ADC_TEMP and ADC_VREF) appropriately.
 * Configures the pin if it's not an internal ADC channel before conversion.
 *
 * @param pin The pin number to read from; can be an external pin or an internal ADC channel.
 * @param resolution The ADC resolution in bits (6, 8, 10, or 12).
 * @return The converted analog value as a 32-bit unsigned integer.
 */
uint32_t getAdcValue(pin_size_t pin, uint32_t resolution) {
    if (pin != ADC_TEMP && pin != ADC_VREF) {
        pinOpsPinout(ADC_PinOps, pin);
    }

    auto result = adc::ADC::get_instance(adc::ADC_Base::ADC0_BASE);
    if (result.error() != adc::ADC_Error_Type::OK) {
        return 0U;
    }
    auto& adcInstance = result.value();

    adc::ADC_Channel channel = getAdcChannelFromPin(pin);
    adc::ADC_Sample_Time sampleTime = getAdcSampleTime(pin);

    volatile uint32_t convertedData =
        adcInstance.start_regular_single_conversion(channel, sampleTime, getAdcResolution(resolution), true);

    return convertedData;
}

// DAC

/**
 * @brief Sets the DAC value for a specified pin.
 *
 * Configures and sets the DAC output value on the specified pin.
 * If `is_initialized` is false, initializes the DAC and configures the pin before setting the value.
 * If `is_initialized` is true, assumes the DAC is already initialized and only sets the output value.
 *
 * @param pin The pin number associated with the DAC output.
 * @param value The 12-bit value to set on the DAC output.
 * @param is_initialized Set to false to initialize the DAC before setting the value; true to skip initialization.
 */
void setDacValue(pin_size_t pin, uint16_t value, bool is_initialized) {
#if DAC_PERIPH_COUNT != 0
    auto dacInternal = getPinOpsPeripheralBase<DACPinOps, dac::Internal_Device>(DAC_PinOps, pin);

    if (dacInternal == dac::Internal_Device::INVALID) {
        return;
    }

    dac::DAC& dac_ = dac::DAC::get_instance();

    if (is_initialized != true) {
        dac_.reset();
        dac_.set_trigger_enable(dacInternal, false);
        dac_.set_wave_mode(dacInternal, dac::Wave_Type::DISABLE);
        dac_.set_output_buffer_enable(dacInternal, true);
        dac_.enable(dacInternal);
        dac_.set_data(dacInternal, dac::Data_Align::RIGHT_12B, value);
        pinOpsPinout(DAC_PinOps, pin);
    } else {
        // set dac value
        dac_.set_data(dacInternal, dac::Data_Align::RIGHT_12B, value);
    }
#endif
}

/**
 * Stops the DAC operation on the specified pin.
 *
 * @param pin The pin number on which to stop DAC operation
 * @return None
 */
void dacStop(pin_size_t pin) {
    auto dacInternal = getPinOpsPeripheralBase<DACPinOps, dac::Internal_Device>(DAC_PinOps, pin);
    if (dacInternal == dac::Internal_Device::INVALID) {
        return;
    }

    dac::DAC& dac_ = dac::DAC::get_instance();
    dac_.disable(dacInternal);
    dac_.reset();
}

// PWM

/**
 * Initializes or updates PWM output on a specified pin.
 *
 * @param pin Pin number to output PWM signal
 * @param frequency PWM frequency in Hz
 * @param value PWM compare value in specified format
 * @param format Format of the compare value (CCFormat enum class)
 *
 * @note If pin is not PWM capable or invalid, function returns without effect
 */
void pwmStart(pin_size_t pin, uint32_t frequency, uint32_t value, CCFormat format) {
    auto timer_base = getPinOpsPeripheralBase<TIMERPinOps, timer::TIMER_Base>(TIMER_PinOps, pin);
    if (timer_base == timer::TIMER_Base::INVALID) {
        return;
    }
    uint32_t packedPinOps = getPackedPinOps(TIMER_PinOps, pin);
    if (packedPinOps == invalidValue) {
        return;
    }
    uint8_t channel = getPackedPinChannel(packedPinOps);
    if (static_cast<timer::Timer_Channel>(channel) == timer::Timer_Channel::INVALID) {
        return;
    }

    auto& timerInstance = GeneralTimer::get_instance(timer_base);
    InputOutputMode previous = timerInstance.getChannelMode(channel);

    timerInstance.setRolloverValue(frequency, TimerFormat::HERTZ);
    timerInstance.setCaptureCompare(channel, value, format);

    if (previous != InputOutputMode::PWM0) {
        timerInstance.setChannelMode(channel, InputOutputMode::PWM0, pin);
        timerInstance.start();
    }
}

/**
 * Stops PWM output on the specified pin by halting the associated timer.
 * @param pin The pin number on which to stop PWM output
 */
void pwmStop(pin_size_t pin) {
    auto timer_base = getPinOpsPeripheralBase<TIMERPinOps, timer::TIMER_Base>(TIMER_PinOps, pin);
    if (timer_base == timer::TIMER_Base::INVALID) {
        return;
    }
    auto& timerInstance = GeneralTimer::get_instance(timer_base);
    timerInstance.stop();
}

// Arduino api functions

/**
 * Sets the reference voltage used for analog input.
 * @param mode The reference voltage mode to set.
 */
void analogReference(uint8_t mode) {
    // TODO: implement this
    (void)mode;
}

/**
 * Reads the analog value from a specified pin.
 * @param pin The analog pin to read from
 * @return The converted analog value mapped to current resolution (0 if NO_PIN)
 */
int analogRead(pin_size_t pin) {
    if (pin == NO_PIN) {
        return 0;
    }
    uint32_t value = getAdcValue(pin, internalReadResolution_);
    return mapResolution(value, internalReadResolution_, readResolution_);
}

/**
 * Writes an analog value to a pin using DAC, PWM, or digital output.
 *
 * @param pin Pin number to write to
 * @param value Value to write (range depends on write resolution)
 *
 * For DAC pins: Configures DAC and writes analog value
 * For PWM pins: Configures timer and starts PWM output
 * For other pins: Sets pin to digital output with HIGH/LOW based on value
 */
void analogWrite(pin_size_t pin, int value) {
    if (pin == NO_PIN) {
        return;
    }

    bool isInitialized = true;
    if (isPinInPinOps(DAC_PinOps, pin)) {
        if (!pinConfigManager.isPinConfigured(pin)) {
            isInitialized = false;
            pinConfigManager.setPinConfigured(pin);
        }
        value = mapResolution(static_cast<uint32_t>(value), writeResolution_, pwmResolution_);
        setDacValue(pin, static_cast<uint16_t>(value), isInitialized);
    } else if (isPinInPinOps(TIMER_PinOps, pin)) {
        if (!pinConfigManager.isPinConfigured(pin)) {
            pinConfigManager.setPinConfigured(pin);
        }
        value = mapResolution(static_cast<uint32_t>(value), writeResolution_, internalWriteResolution_);
        pwmStart(pin, writeFrequency_, static_cast<uint32_t>(value), getPwmResolution(internalWriteResolution_));
    } else {
        pinMode(pin, OUTPUT);
        value = mapResolution(static_cast<uint32_t>(value), writeResolution_, 8);
        digitalWrite(pin, (value >= 128) ? HIGH : LOW);
    }
}

/**
 * Sets the resolution for analog read operations.
 * @param resolution The desired ADC resolution (1-12 bits).
 * Resolution will be clamped to 8, 10 or 12 bits based on hardware limits.
 */
void analogReadResolution(int resolution) {
    if ((resolution > 0) && (resolution <= 12)) {
        readResolution_ = resolution;
        internalReadResolution_ = (resolution > ADC_MAX_RESOLUTION) ?
                                  ADC_MAX_RESOLUTION : (resolution <= 8) ?
                                  8 : (resolution <= 10) ?
                                  10 : 12;
    }
}

/**
 * Sets the resolution for PWM analog write operations.
 * @param resolution The desired PWM resolution (1-16 bits)
 * Resolution is capped at PWM_MAX_RESOLUTION if the requested value exceeds it
 */
void analogWriteResolution(int resolution) {
    if ((resolution > 0) && (resolution <= 16)) {
        writeResolution_ = resolution;
        internalWriteResolution_ = (resolution > PWM_MAX_RESOLUTION) ? PWM_MAX_RESOLUTION : resolution;
    }
}

/**
 * Sets the PWM frequency for analog write operations.
 * @param frequency The desired PWM frequency in Hz
 */
void analogWriteFrequency(uint32_t frequency) {
    writeFrequency_ = frequency;
}

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
