#include "Arduino.h"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"

/**
 * @brief Free debug pins for other use
 *
 * This function takes a pin number as argument and frees the associated debug
 * pin for other use. The debug pins are JTAG-DP and SW-DP pins.
 *
 * @param pin The pin number to free
 */
void freeDebugPins(pin_size_t pin) {
#ifndef USE_SWD_DEBUG
    gpio::Pin_Number gpioPin = getPinInPort(pin);
    gpio::GPIO_Base gpioPort = getPortFromPin(pin);
    if (gpioPin == gpio::Pin_Number::INVALID || gpioPort == gpio::GPIO_Base::INVALID) {
        return;
    }

    // JTAG-DP disabled and SW-DP disabled
    if ((gpioPort == gpio::GPIO_Base::GPIOA_BASE) && ((gpioPin == gpio::Pin_Number::PIN_13) || (gpioPin == gpio::Pin_Number::PIN_14))) {
        AFIO_I.set_remap(gpio::Pin_Remap_Select::SWJ_ALL_DISABLED_REMAP);
        AFIO_I.set_remap(gpio::Pin_Remap_Select::SWJ_ALL_DISABLED_REMAP);
    }

    // JTAG-DP disabled and SW-DP enabled
    if (((gpioPort == gpio::GPIO_Base::GPIOA_BASE) && (gpioPin == gpio::Pin_Number::PIN_15)) ||
            ((gpioPort == gpio::GPIO_Base::GPIOB_BASE) && ((gpioPin == gpio::Pin_Number::PIN_3) || (gpioPin == gpio::Pin_Number::PIN_4)))) {
        AFIO_I.set_remap(gpio::Pin_Remap_Select::SWJ_DP_ONLY_REMAP);
        AFIO_I.set_remap(gpio::Pin_Remap_Select::SWJ_DP_ONLY_REMAP);
    }
#else
    UNUSED(pin);
#endif
}
