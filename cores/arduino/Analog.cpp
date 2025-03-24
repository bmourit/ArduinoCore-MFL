
#include "Analog.h"
#include "PinConfigManager.hpp"

static int pwmResolution_ = PWM_RESOLUTION;
static int readResolution_ = ADC_RESOLUTION;
static int writeResolution_ = PWM_RESOLUTION;
static int internalReadResolution_ = (ADC_RESOLUTION > ADC_MAX_RESOLUTION) ? ADC_MAX_RESOLUTION : ADC_RESOLUTION;
static int internalWriteResolution_ = (PWM_RESOLUTION > PWM_MAX_RESOLUTION) ? PWM_MAX_RESOLUTION : PWM_RESOLUTION;
static uint32_t writeFrequency_ = static_cast<uint32_t>(PWM_FREQUENCY);

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
    return static_cast<int>(mapResolution(value, internalReadResolution_, readResolution_));
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
        value = static_cast<int>(mapResolution(static_cast<uint32_t>(value), writeResolution_, pwmResolution_));
        setDacValue(pin, static_cast<uint16_t>(value), isInitialized);
    } else if (isPinInPinOps(TIMER_PinOps, pin)) {
        if (!pinConfigManager.isPinConfigured(pin)) {
            pinConfigManager.setPinConfigured(pin);
        }
        value = static_cast<int>(mapResolution(static_cast<uint32_t>(value), writeResolution_, internalWriteResolution_));
        pwmStart(pin, writeFrequency_, static_cast<uint32_t>(value), getPwmResolution(internalWriteResolution_));
    } else {
        pinMode(pin, OUTPUT);
        value = static_cast<int>(mapResolution(static_cast<uint32_t>(value), writeResolution_, 8));
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
