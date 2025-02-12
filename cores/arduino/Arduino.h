
#pragma once

#include "api/ArduinoAPI.h"
#include "api/itoa.h"
#include "api/deprecated-avr-comp/avr/dtostrf.h"
#include "CoreHandler.h"

#if defined(__cplusplus)
    #include "mfl_api.h"
    #include "variant.h"

    using namespace arduino;

#endif // __cplusplus

#define interrupts()      __enable_irq()
#define noInterrupts()    __disable_irq()

#ifdef __cplusplus
    // We currently support less than this
    inline constexpr pin_size_t NO_PIN = 70;
#else
    #define NO_PIN	70
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Extra analog functions
void analogReadResolution(int resolution);
void analogWriteResolution(int resolution);
void analogWriteFrequency(uint32_t frequency);
// Extra digital function
void digitalToggle(pin_size_t pin);
// Free debug pins for other use
void freeDebugPins(pin_size_t pin);

inline constexpr PinMode INPUT_ANALOG = static_cast<PinMode>(0x5);
inline constexpr IRQn_Type INVALID_IRQ = static_cast<IRQn_Type>(99);

#ifdef __cplusplus
}
#endif
