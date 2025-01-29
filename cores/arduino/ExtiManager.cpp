
#include "Arduino.h"
#include "ExtiManager.h"

constexpr uint8_t extiIRQPriority = EXTI_IRQ_PRIORITY;
constexpr uint8_t extiIRQSubPriority = EXTI_IRQ_SUBPRIORITY;
constexpr uint8_t maxExtiLines_ = MAX_EXTI_LINES;

ExtiManager& ExtiManager::get_instance() {
    static ExtiManager instance;
    return instance;
}

std::array<exti_to_irq, maxExtiLines_> ExtiManager::irq_index {{
    {0U, EXTI0_IRQn}, {1U, EXTI1_IRQn}, {2U, EXTI2_IRQn}, {3U, EXTI3_IRQn},
    {4U, EXTI4_IRQn}, {5U, EXTI5_9_IRQn}, {6U, EXTI5_9_IRQn}, {7U, EXTI5_9_IRQn},
    {8U, EXTI5_9_IRQn}, {9U, EXTI5_9_IRQn}, {10U, EXTI10_15_IRQn}, {11U, EXTI10_15_IRQn},
    {12U, EXTI10_15_IRQn}, {13U, EXTI10_15_IRQn}, {14U, EXTI10_15_IRQn}, {15U, EXTI10_15_IRQn}
}};

ExtiManager::ExtiManager() :
    exti_(exti::EXTI::get_instance()),
    callbacks_{nullptr} {}

/**
 * @brief Enable an EXTI interrupt on a specific pin.
 *
 * This function enables EXTI interrupts on a given pin. It configures the pin
 * for input and sets the EXTI source in the AFIO. The EXTI interrupt is then
 * enabled and the NVIC priority is set.
 *
 * @param pin The pin for which to enable the EXTI interrupt
 * @param callback The callback function to be called when the interrupt occurs
 * @param type The type of EXTI interrupt to enable (RISING, FALLING, or BOTH)
 *
 * @note This function configures the pin to be an input pin and sets the EXTI
 * source in the AFIO. The EXTI interrupt is enabled and the NVIC priority is
 * set.
 */
void ExtiManager::enablePinExtiInterrupt(pin_size_t pin, EXTICallback callback, exti::EXTI_Trigger type) {
    gpio::GPIO_Base gpioPort = getPortFromPin(pin);
    gpio::Pin_Number pinNumber = getPinInPort(pin);
    if (pinNumber == gpio::Pin_Number::INVALID || gpioPort == gpio::GPIO_Base::INVALID) {
        return;
    }
    uint8_t id = static_cast<uint8_t>(pinNumber);
    if (id >= maxExtiLines_) {
        return;
    }
    callbacks_[id] = callback;

    auto portResult = gpio::GPIO::get_instance(gpioPort);
    if (portResult.error() != gpio::GPIO_Error_Type::OK) {
        return;
    }
    auto& instance = portResult.value();
    gpio::Pin_Mode currentMode = instance.get_pin_mode(pinNumber);

    // Use the currently set mode (input) in case user has already configured
    // the pin using the digital pinMode interface.
    //
    // NOTE:
    //  Only input only here as only input pins can have an exti
    if (currentMode != gpio::Pin_Mode::INPUT_FLOATING &&
        currentMode != gpio::Pin_Mode::INPUT_PULLUP &&
        currentMode != gpio::Pin_Mode::INPUT_PULLDOWN) {
        instance.set_pin_mode(pinNumber, gpio::Pin_Mode::INPUT_FLOATING);
    }

    instance.set_pin_mode(pinNumber, currentMode);

    gpio::Source_Port sourcePort = gpio::Source_Port::INVALID;

    switch (gpioPort) {
    case gpio::GPIO_Base::GPIOA_BASE:
        sourcePort = gpio::Source_Port::SOURCE_IS_GPIOA;
        break;
    case gpio::GPIO_Base::GPIOB_BASE:
        sourcePort = gpio::Source_Port::SOURCE_IS_GPIOB;
        break;
    case gpio::GPIO_Base::GPIOC_BASE:
        sourcePort = gpio::Source_Port::SOURCE_IS_GPIOC;
        break;
    case gpio::GPIO_Base::GPIOD_BASE:
        sourcePort = gpio::Source_Port::SOURCE_IS_GPIOD;
        break;
    case gpio::GPIO_Base::INVALID:
    default:
        break;
    }

    // Set the source
    gpio::AFIO::get_instance().set_exti_source(sourcePort, pinNumber);
    // Make sure the flag is clear in case of previous use
    exti_.clear_interrupt_flag(static_cast<exti::Interrupt_Flags>(id));
    // Enable the exti for the pin
    exti_.init(static_cast<exti::EXTI_Line>(id), exti::EXTI_Mode::EXTI_INTERRUPT, type);

    // Set NVIC priority and enable interrupt
    NVIC_SetPriority(extiToIrq(id), extiIRQPriority);
    NVIC_EnableIRQ(extiToIrq(id));
}

/**
 * @brief Disable an EXTI interrupt on a specific pin.
 *
 * This function disables EXTI interrupts on a given pin. It checks if there are
 * any other callbacks registered for the same interrupt line and if so, it
 * doesn't disable the IRQ. If there are no other callbacks, it disables the IRQ.
 *
 * @param pin The pin for which to disable the EXTI interrupt
 *
 * @note This function doesn't set the pin to be an output pin or set the EXTI
 * source in the AFIO. The EXTI interrupt is disabled and the NVIC priority is
 * set.
 */
void ExtiManager::disablePinExtiInterrupt(pin_size_t pin) {
    gpio::Pin_Number pinNum = getPinInPort(pin);
    if (pinNum == gpio::Pin_Number::INVALID) {
        return;
    }

    uint8_t id = static_cast<uint8_t>(pinNum);
    if (id >= maxExtiLines_) {
        return;
    }

    callbacks_[id] = nullptr;
    bool disableIRQ = true;

    // Check for unhandled callbacks first
    for (uint8_t i = 0U; i < maxExtiLines_; ++i) {
        if (extiToIrq(id) == extiToIrq(i) && callbacks_[i] != nullptr) {
            disableIRQ = false;
            break;
        }
    }

    if (disableIRQ) {
        NVIC_DisableIRQ(extiToIrq(id));
    }
}

/**
 * @brief Handles an EXTI interrupt and calls the registered callback.
 *
 * This function takes a Pin_Number and determines the EXTI line and the
 * corresponding callback. It checks if the EXTI flag is set and if so,
 * clears the flag and calls the callback. If the callback is nullptr,
 * nothing is called.
 *
 * @param pin The pin that triggered the EXTI interrupt
 */
void ExtiManager::handleCallback(gpio::Pin_Number pin) {
    uint8_t id = static_cast<uint8_t>(pin);
    exti::Interrupt_Flags flag = static_cast<exti::Interrupt_Flags>(id);
    if (exti_.get_interrupt_flag(flag)) {
        exti_.clear_interrupt_flag(flag);
        if (callbacks_[id] != nullptr) {
            callbacks_[id]();
        }
    }
}


// Interrupt handlers
extern "C" {

    void EXTI0_IRQHandler(void) {
        ExtiManager::get_instance().handleCallback(gpio::Pin_Number::PIN_0);
    }
    void EXTI1_IRQHandler(void) {
        ExtiManager::get_instance().handleCallback(gpio::Pin_Number::PIN_1);
    }
    void EXTI2_IRQHandler(void) {
        ExtiManager::get_instance().handleCallback(gpio::Pin_Number::PIN_2);
    }
    void EXTI3_IRQHandler(void) {
        ExtiManager::get_instance().handleCallback(gpio::Pin_Number::PIN_3);
    }
    void EXTI4_IRQHandler(void) {
        ExtiManager::get_instance().handleCallback(gpio::Pin_Number::PIN_4);
    }
    void EXTI5_9_IRQHandler(void) {
        for (uint32_t i = static_cast<uint32_t>(gpio::Pin_Number::PIN_5);
                   i <= static_cast<uint32_t>(gpio::Pin_Number::PIN_9); ++i) {
            ExtiManager::get_instance().handleCallback(static_cast<gpio::Pin_Number>(i));
        }
    }
    void EXTI10_15_IRQHandler(void) {
        for (uint32_t i = static_cast<uint32_t>(gpio::Pin_Number::PIN_10);
                   i <= static_cast<uint32_t>(gpio::Pin_Number::PIN_15); ++i) {
            ExtiManager::get_instance().handleCallback(static_cast<gpio::Pin_Number>(i));
        }
    }

} // extern "C"
