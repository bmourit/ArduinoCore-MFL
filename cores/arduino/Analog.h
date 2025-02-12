
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
inline const adc::ADC_Channel getAdcChannel(pin_size_t pin);
inline constexpr adc::ADC_Channel getAdcInternalChannel(pin_size_t pin);
inline adc::ADC_Channel getAdcChannelFromPin(pin_size_t pin);
inline adc::ADC_Sample_Time getAdcSampleTime(pin_size_t pin);
inline constexpr adc::ADC_Resolution getAdcResolution(uint32_t resolution);
inline CCFormat getPwmResolution(uint32_t resolution);
inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to);
