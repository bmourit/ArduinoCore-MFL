
#pragma once

#include "api/ArduinoAPI.h"
#include "api/itoa.h"
#include "api/deprecated-avr-comp/avr/dtostrf.h"
#include "CoreHandler.h"
#include "variant.h"

// libc porting layers for GCC
#if defined(__GNUC__)
    #define WEAK  __attribute__((weak))
#endif

#ifdef __cplusplus
    #include "mfl_api.h"
    using namespace arduino;
#endif  // __cplusplus

// Arduino core version number
#define VERSION_MAJOR   1   // Major version
#define VERSION_MINOR   0   // Minor version
#define VERSION_PATCH   5   // Patch version

#define interrupts()      __enable_irq()
#define noInterrupts()    __disable_irq()

#ifdef __cplusplus
    // We currently support less than this
    inline constexpr pin_size_t NO_PIN = 70;
    // Arduino API extensions
    inline constexpr PinMode INPUT_ANALOG = static_cast<PinMode>(0x5);
    inline constexpr IRQn_Type INVALID_IRQ = static_cast<IRQn_Type>(99);
    inline constexpr uint32_t CORE_VERSION = (VERSION_MAJOR << 16) | (VERSION_MINOR << 8) | (VERSION_PATCH);
#else
    #define NO_PIN  70
    #define INPUT_ANALOG    ((PinMode)0x5)
    #define INVALID_IRQ     ((IRQn_Type)99)
    #define CORE_VERSION    (VERSION_MAJOR << 16) | (VERSION_MINOR << 8) | (VERSION_PATCH)
#endif  // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif

    // Extra analog functions
    void analogReadResolution(int resolution);
    void analogWriteResolution(int resolution);
    void analogWriteFrequency(uint32_t frequency);

    // Opaque type for C code
    typedef uint32_t GPIO_Port_t;

    // Pin functions
    bool isPinNumberValid(pin_size_t pin);
    GPIO_Port_t digitalPinToPort(pin_size_t pin);
    pin_size_t digitalPinToInterrupt(pin_size_t pin);
    pin_size_t analogInputToDigitalPin(pin_size_t pin);

    // Port functions
    uint32_t portOutputRegister(GPIO_Port_t port);
    uint32_t portInputRegister(GPIO_Port_t port);
    uint32_t portSetRegister(GPIO_Port_t port);
    uint32_t portClearRegister(GPIO_Port_t port);

    // Extra digital function
    void digitalToggle(pin_size_t pin);

    // Free debug pins for other use
    void freeDebugPins(pin_size_t pin);

#ifdef __cplusplus
}
#endif
