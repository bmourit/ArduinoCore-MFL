#include "Arduino.h"
#include "variant.h"

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
    const PortPinPair& pp = port_pin_map[pin];
    if (pp.pin == gpio::Pin_Number::INVALID || pp.port == gpio::GPIO_Base::INVALID) {
        return;
    }

    // JTAG-DP disabled and SW-DP disabled
    if ((pp.port == gpio::GPIO_Base::GPIOA_BASE) && ((pp.pin == gpio::Pin_Number::PIN_13) || (pp.pin == gpio::Pin_Number::PIN_14))) {
        AFIO_I.set_remap(gpio::Pin_Remap_Select::SWJ_ALL_DISABLED_REMAP);
    }

    // JTAG-DP disabled and SW-DP enabled
    if (((pp.port == gpio::GPIO_Base::GPIOA_BASE) && (pp.pin == gpio::Pin_Number::PIN_15)) ||
            ((pp.port == gpio::GPIO_Base::GPIOB_BASE) && ((pp.pin == gpio::Pin_Number::PIN_3) || (pp.pin == gpio::Pin_Number::PIN_4)))) {
        AFIO_I.set_remap(gpio::Pin_Remap_Select::SWJ_DP_ONLY_REMAP);
    }
#else
    UNUSED(pin);
#endif
}
