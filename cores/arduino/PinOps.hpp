#pragma once

#include "Arduino.h"

struct PinOps {
    gpio::GPIO_Base port;
    gpio::Pin_Number pin;
    uint32_t packedPinOps;
    bool operator==(const PinOps& other) const {
        return (port == other.port) && (pin == other.pin) && (packedPinOps == other.packedPinOps);
    }
    bool operator!=(const PinOps& other) const {
        return !(*this == other);
    }
};

struct USARTPinOps {
    usart::USART_Base peripheral;
    PinOps pinOps;
};

struct TIMERPinOps {
    timer::TIMER_Base peripheral;
    PinOps pinOps;
};

struct ADCPinOps {
    adc::ADC_Base peripheral;
    PinOps pinOps;
};

struct DACPinOps {
    dac::Internal_Device peripheral;
    PinOps pinOps;
};

struct I2CPinOps {
    i2c::I2C_Base peripheral;
    PinOps pinOps;
};

struct SPIPinOps {
    spi::SPI_Base peripheral;
    PinOps pinOps;
};

struct SDIOPinOps {
    sdio::SDIO_Base peripheral;
    PinOps pinOps;
};

enum class ConfigBits : uint32_t {
    MODE_SHIFT = 0U,
    MODE_MASK = 0x7U,
    SPEED_SHIFT = 3U,
    SPEED_MASK = 0x7U << SPEED_SHIFT,
    REMAP_SHIFT = 6U,
    REMAP_MASK = 0x3FU << REMAP_SHIFT,
    CHANNEL_SHIFT = 12U,
    CHANNEL_MASK = 0xFU << CHANNEL_SHIFT,
    CHON_SHIFT = 16U,
    CHON_MASK = 0x1U << CHON_SHIFT
};

void setPinOp(pin_size_t pin, uint32_t packedPinOps);
void setPinOp(gpio::GPIO_Base port, gpio::Pin_Number pin, uint32_t packedPinOps);
inline constexpr gpio::GPIO_Base getPortFromPin(pin_size_t pin);
inline constexpr gpio::Pin_Number getPinInPort(pin_size_t pin);
inline constexpr uint32_t createPackedPinOps(gpio::Pin_Mode mode, gpio::Output_Speed speed,
           gpio::Pin_Remap_Select remap = gpio::Pin_Remap_Select::NO_REMAP, 
           uint8_t channel = 0U, uint8_t chon = 0U);


/////////////////////////////// PIN IN PINOPS ///////////////////////////////

/**
 * @brief Checks if a given pin is present in a PinOps array.
 *
 * This function takes a PinOps array and a pin number as arguments and checks
 * if the given pin is present in the array. If the pin is found, the function
 * returns true. Otherwise, it returns false.
 *
 * @param[in] pinOpsArray The PinOps array to search in.
 * @param[in] pin The pin number to search for.
 * @return true if the pin is found in the array, false otherwise.
 */
template<typename T>
inline bool isPinInPinOps(const T* pinOpsArray, pin_size_t pin) {
    for (const T* ops = pinOpsArray; ops->pinOps.pin != gpio::Pin_Number::INVALID; ++ops) {
        if ((ops->pinOps.port == getPortFromPin(pin)) && (ops->pinOps.pin == getPinInPort(pin))) {
            return true;
        }
    }
    return false;
}


/////////////////////////////// CREATE PACKED PINOPS ///////////////////////////////

/**
 * @brief Creates a packed 32-bit word representing PinOps.
 *
 * The created packed PinOps word contains the following information:
 *  - Pin mode (lowest 3 bits)
 *  - Pin output speed (next 3 bits)
 *  - Pin remap (next 6 bits)
 *  - Timer channel (next 4 bits)
 *  - Timer channel companion (highest bit)
 *
 * @param[in] mode The pin mode to set.
 * @param[in] speed The pin output speed to set.
 * @param[in] remap The pin remap to set.
 * @param[in] channel The timer channel to set.
 * @param[in] chon The timer channel companion to set.
 * @return A packed 32-bit word representing the PinOps.
 */
inline constexpr uint32_t createPackedPinOps(gpio::Pin_Mode mode, gpio::Output_Speed speed, gpio::Pin_Remap_Select remap, uint8_t channel, uint8_t chon) {
    return (static_cast<uint32_t>(mode) & 0x7U) |
           ((static_cast<uint32_t>(speed) & 0x7U) << static_cast<uint32_t>(ConfigBits::SPEED_SHIFT)) |
           ((static_cast<uint32_t>(remap) & 0x3FU) << static_cast<uint32_t>(ConfigBits::REMAP_SHIFT)) |
           ((static_cast<uint32_t>(channel) & 0xFU) << static_cast<uint32_t>(ConfigBits::CHANNEL_SHIFT)) |
           ((static_cast<uint32_t>(chon) & 0x1U) << static_cast<uint32_t>(ConfigBits::CHON_SHIFT));
}

static inline constexpr uint32_t invalidValue = 0U;

static inline constexpr PinOps invalidPinOps = {
    .port = gpio::GPIO_Base::INVALID,
    .pin = gpio::Pin_Number::INVALID,
    .packedPinOps = createPackedPinOps(gpio::Pin_Mode::INVALID, gpio::Output_Speed::INVALID, gpio::Pin_Remap_Select::NO_REMAP, 0, 0)
};


/////////////////////////////// PINOPS BY PERIPHERAL ///////////////////////////////

/**
 * @brief Retrieves PinOps from a PinOps array by peripheral base.
 *
 * This function takes a PinOps array and a peripheral base as arguments and
 * returns the PinOps associated with the given peripheral base. If no matching
 * PinOps is found, the function returns an invalid PinOps.
 *
 * @param[in] pinOpsArray The PinOps array to search in.
 * @param[in] base The peripheral base to search for.
 * @return The PinOps associated with the given peripheral base, or an invalid
 *         PinOps if no matching PinOps is found.
 */
template<typename T, typename P>
inline constexpr PinOps getPinOpsByPeripheral(const T* pinOpsArray, P base) {
    for (const T* ops = pinOpsArray; ops->pinOps.pin != gpio::Pin_Number::INVALID; ++ops) {
        if (ops->peripheral == base) {
            return ops->pinOps;
        }
    }
    return invalidPinOps;
}


/////////////////////////////// PINOPS PERIPHERAL BASE ///////////////////////////////

/**
 * @brief Retrieves peripheral base from a PinOps array by pin number.
 *
 * This function takes a PinOps array and a pin number as arguments and returns
 * the peripheral base associated with the given pin. If no matching PinOps is
 * found, the function returns the invalid value of the peripheral base type.
 *
 * @param[in] pinOpsArray The PinOps array to search in.
 * @param[in] pin The pin number to search for.
 * @return The peripheral base associated with the given pin, or the invalid
 *         value of the peripheral base type if no matching PinOps is found.
 */
template<typename T, typename P>
inline constexpr P getPinOpsPeripheralBase(const T* pinOpsArray, pin_size_t pin) {
    for (const T* ops = pinOpsArray; ops->pinOps.pin != gpio::Pin_Number::INVALID; ++ops) {
        if ((ops->pinOps.port == getPortFromPin(pin)) && (ops->pinOps.pin == getPinInPort(pin))) {
            return ops->peripheral;
        }
    }
    return P::INVALID;
}


/////////////////////////////// PINOPS BY PIN ///////////////////////////////

/**
 * @brief Retrieves PinOps from a PinOps array by pin number.
 *
 * This function takes a PinOps array and a pin number as arguments and returns
 * the PinOps associated with the given pin. If no matching PinOps is found, the
 * function returns an invalid PinOps.
 *
 * @param[in] pinOpsArray The PinOps array to search in.
 * @param[in] pin The pin number to search for.
 * @return The PinOps associated with the given pin, or an invalid PinOps if no
 *         matching PinOps is found.
 */
template<typename T>
inline constexpr PinOps getPinOpsByPin(const T* pinOpsArray, pin_size_t pin) {
    for (const T* ops = pinOpsArray; ops->pinOps.pin != gpio::Pin_Number::INVALID; ++ops) {
        if ((ops->pinOps.port == getPortFromPin(pin)) && (ops->pinOps.pin == getPinInPort(pin))) {
            return ops->pinOps;
        }
    }
    return invalidPinOps;
}


/////////////////////////////// PACKED PINOPS BY PERIPHERL ///////////////////////////////

/**
 * @brief Retrieves packed PinOps from a PinOps array by peripheral base and pin number.
 *
 * This function takes a PinOps array, a peripheral base, and a pin number as
 * arguments and returns the packed PinOps associated with the given peripheral
 * base and pin. If no matching PinOps is found, the function returns the invalid
 * value.
 *
 * @param[in] pinOpsArray The PinOps array to search in.
 * @param[in] base The peripheral base to search for.
 * @param[in] pin The pin number to search for.
 * @return The packed PinOps associated with the given peripheral base and pin,
 *         or the invalid value if no matching PinOps is found.
 */
template<typename T, typename P>
inline constexpr uint32_t getPackedPinOps(const T* pinOpsArray, P base, pin_size_t pin) {
    for (const T* ops = pinOpsArray; ops->pinOps.pin != gpio::Pin_Number::INVALID; ++ops) {
        if ((ops->peripheral == base) && (ops->pinOps.port == getPortFromPin(pin)) && (ops->pinOps.pin == getPinInPort(pin))) {
            return ops->pinOps.packedPinOps;
        }
    }
    return invalidValue;
}


/////////////////////////////// PACKED PINOPS BY PIN ONLY ///////////////////////////////

/**
 * @brief Retrieves packed PinOps from a PinOps array by pin number.
 *
 * This function takes a PinOps array and a pin number as arguments and returns
 * the packed PinOps associated with the given pin. If no matching PinOps is
 * found, the function returns the invalid value.
 *
 * @param[in] pinOpsArray The PinOps array to search in.
 * @param[in] pin The pin number to search for.
 * @return The packed PinOps associated with the given pin, or the invalid value
 *         if no matching PinOps is found.
 */
template<typename T>
inline constexpr uint32_t getPackedPinOps(const T* pinOpsArray, pin_size_t pin) {
    for (const T* ops = pinOpsArray; ops->pinOps.pin != gpio::Pin_Number::INVALID; ++ops) {
        if ((ops->pinOps.port == getPortFromPin(pin)) && (ops->pinOps.pin == getPinInPort(pin))) {
            return ops->pinOps.packedPinOps;
        }
    }
    return invalidValue;
}


/////////////////////////////// PINOPS PINOUT ///////////////////////////////

/**
 * @brief Applies the pin configuration from a PinOps array to a specified pin.
 *
 * This function takes a PinOps array and a pin number as arguments and applies
 * the pin configuration stored in the PinOps array to the specified pin. If
 * no matching PinOps is found, the function does nothing.
 *
 * @param[in] pinOpsArray The PinOps array to search in.
 * @param[in] pin The pin number to apply the configuration to.
 */
template<typename T>
inline constexpr void pinOpsPinout(const T* pinOpsArray, pin_size_t pin) {
    for (const T* ops = pinOpsArray; ops->pinOps.pin != gpio::Pin_Number::INVALID; ++ops) {
        if ((ops->pinOps.port == getPortFromPin(pin)) && (ops->pinOps.pin == getPinInPort(pin))) {
            setPinOp(pin, ops->pinOps.packedPinOps);
        }
    }
}

/**
 * @brief Extracts the channel number from a packed pin operations value.
 *
 * This function takes a packed pin operations value as input and extracts the
 * channel number by applying a mask and shifting the bits. The extracted channel
 * number is returned as an unsigned 8-bit integer.
 *
 * @param packedPinOps The packed 32-bit pin operations value containing the channel information.
 * @return The channel number extracted from the packed pin operations.
 */
inline constexpr uint8_t getPackedPinChannel(uint32_t packedPinOps) {
    return static_cast<uint8_t>((packedPinOps & static_cast<uint32_t>(ConfigBits::CHANNEL_MASK)) >> static_cast<uint32_t>(ConfigBits::CHANNEL_SHIFT));
}

/**
 * @brief Extracts the channel-on number from a packed pin operations value.
 *
 * This function takes a packed pin operations value as input and extracts the
 * channel-on number by applying a mask and shifting the bits. The extracted
 * channel-on number is returned as an unsigned 8-bit integer.
 *
 * @param packedPinOps The packed 32-bit pin operations value containing the channel-on information.
 * @return The channel-on number extracted from the packed pin operations.
 */
inline constexpr uint8_t getPackedPinChOn(uint32_t packedPinOps) {
    return static_cast<uint8_t>((packedPinOps & static_cast<uint32_t>(ConfigBits::CHON_MASK)) >> static_cast<uint32_t>(ConfigBits::CHON_SHIFT));
}

/**
 * @brief Retrieves the pin mode from a packed PinOps value.
 *
 * This function takes a packed 32-bit PinOps value as input and extracts the
 * pin mode by applying a mask. The extracted pin mode is returned as an
 * unsigned 32-bit integer.
 *
 * @param packedPinOps The packed 32-bit PinOps value containing the pin mode
 *                     information.
 * @return The pin mode extracted from the packed PinOps value.
 */
inline constexpr gpio::Pin_Mode getPackedPinMode(uint32_t packedPinOps) {
    uint32_t pinMode = (packedPinOps & static_cast<uint32_t>(ConfigBits::MODE_MASK));
    return static_cast<gpio::Pin_Mode>(pinMode);
}

/**
 * @brief Retrieves the output speed from a packed PinOps value.
 *
 * This function takes a packed 32-bit PinOps value as input and extracts the
 * output speed by applying a mask and shifting the bits. The extracted output
 * speed is returned as an unsigned 32-bit integer and cast to an Output_Speed
 * enumeration.
 *
 * @param packedPinOps The packed 32-bit PinOps value containing the output
 *                     speed information.
 * @return The output speed extracted from the packed PinOps value.
 */
inline constexpr gpio::Output_Speed getPackedPinSpeed(uint32_t packedPinOps) {
    uint32_t pinSpeed = ((packedPinOps & static_cast<size_t>(ConfigBits::SPEED_MASK)) >> static_cast<size_t>(ConfigBits::SPEED_SHIFT));
    return static_cast<gpio::Output_Speed>(pinSpeed);
}

/**
 * @brief Retrieves the pin remap from a packed PinOps value.
 *
 * This function takes a packed 32-bit PinOps value as input and extracts the
 * pin remap by applying a mask and shifting the bits. The extracted pin remap
 * is returned as an unsigned 32-bit integer and cast to a Pin_Remap_Select
 * enumeration.
 *
 * @param packedPinOps The packed 32-bit PinOps value containing the pin remap
 *                     information.
 * @return The pin remap extracted from the packed PinOps value.
 */
inline constexpr gpio::Pin_Remap_Select getPackedPinRemap(uint32_t packedPinOps) {
    uint32_t remap = ((packedPinOps & static_cast<uint32_t>(ConfigBits::REMAP_MASK)) >> static_cast<uint32_t>(ConfigBits::REMAP_SHIFT));
    return static_cast<gpio::Pin_Remap_Select>(remap);
}

/**
 * @brief Retrieves a GPIO instance from a GPIO base address.
 *
 * This function takes a GPIO base address as input and returns the corresponding
 * GPIO instance. If the GPIO base address is invalid, the function logs a debug
 * message and returns the invalid instance.
 *
 * @param port The GPIO base address to retrieve the instance for.
 * @return The GPIO instance associated with the given base address, or the invalid
 *         instance if the base address is invalid.
 */
inline gpio::GPIO portToInstance(gpio::GPIO_Base port) {
    auto result = gpio::GPIO::get_instance(port);
    if (result.error() != gpio::GPIO_Error_Type::OK) {
        core_debug("Invalid gpio instance!");
    }
    return result.value();
}

/**
 * @brief Converts a pin number to its corresponding GPIO base address.
 *
 * This function takes a pin number as input and extracts the GPIO base address
 * by shifting the pin number to the right by 4 bits. The extracted base address
 * is checked for validity and returned as a GPIO_Base enumeration.
 *
 * @param pin The pin number to convert.
 * @return The GPIO base address associated with the given pin number, or
 *         GPIO_Base::INVALID if the pin number is invalid.
 */
inline constexpr gpio::GPIO_Base getPortFromPin(pin_size_t pin) {
    uint32_t portIndex = static_cast<uint32_t>(pin >> static_cast<uint32_t>(4U));
    if (portIndex >= static_cast<uint32_t>(gpio::GPIO_Base::INVALID)) {
        return gpio::GPIO_Base::INVALID;
    }
    return static_cast<gpio::GPIO_Base>(portIndex);
}

/**
 * @brief Retrieves the pin number in the port for the given pin number.
 *
 * This function takes a pin number as input and extracts the pin number in the
 * port by taking the modulus of the pin number with 16. The extracted pin number
 * is checked for validity and returned as a Pin_Number enumeration.
 *
 * @param pin The pin number to retrieve the pin number in the port for.
 * @return The pin number in the port associated with the given pin number, or
 *         Pin_Number::INVALID if the pin number is invalid.
 */
inline constexpr gpio::Pin_Number getPinInPort(pin_size_t pin) {
    uint32_t pinInPort = static_cast<uint32_t>(pin & 0xF);

    if (pinInPort <= 15U) {
        return static_cast<gpio::Pin_Number>(pinInPort);
    }
    return gpio::Pin_Number::INVALID;
}

/**
 * @brief Computes the bit mask value for a given pin number in its port.
 *
 * This function takes a pin number as input and computes the bit mask value for
 * the pin in its port. The bit mask value is computed by shifting 1 to the
 * right by the pin number modulo 16.
 *
 * @param pin The pin number to compute the bit mask value for.
 * @return The bit mask value for the given pin number in its port.
 */
inline constexpr uint32_t getPinBitMaskValueInPort(pin_size_t pin) {
    return static_cast<uint32_t>(1U << (pin & 0xF));
}

/**
 * @brief Configures a pin on the specified port with the given mode.
 *
 * This function takes a GPIO base address, a pin number, and a pin mode as
 * arguments and configures the pin with the given mode. The function does not
 * verify the validity of the pin number or the pin mode and directly sets the
 * pin mode.
 *
 * @param port The GPIO base address to configure the pin on.
 * @param pin The pin number to configure.
 * @param mode The pin mode to set.
 */
inline void setPinOpsFast(gpio::GPIO_Base port, gpio::Pin_Number pin, gpio::Pin_Mode mode) {
    auto& instance = gpio::GPIO::get_instance(port).value();
    instance.set_pin_mode(pin, mode);
}
