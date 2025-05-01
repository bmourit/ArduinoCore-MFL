
#include "Arduino.h"
#include "ExtiManager.h"
#include "variant.h"

constexpr uint8_t extiIRQPriority = EXTI_IRQ_PRIORITY;
constexpr uint8_t extiIRQSubPriority = EXTI_IRQ_SUBPRIORITY;

auto ExtiManager::get_instance() -> ExtiManager& {
    static ExtiManager instance;
    return instance;
}

std::array<exti_to_irq, maxExtiLines_> ExtiManager::irq_index {{
    {0U, EXTI0_IRQn},       {1U, EXTI1_IRQn},       {2U, EXTI2_IRQn},       {3U, EXTI3_IRQn},
    {4U, EXTI4_IRQn},       {5U, EXTI5_9_IRQn},     {6U, EXTI5_9_IRQn},     {7U, EXTI5_9_IRQn},
    {8U, EXTI5_9_IRQn},     {9U, EXTI5_9_IRQn},     {10U, EXTI10_15_IRQn},  {11U, EXTI10_15_IRQn},
    {12U, EXTI10_15_IRQn},  {13U, EXTI10_15_IRQn},  {14U, EXTI10_15_IRQn},  {15U, EXTI10_15_IRQn}
}};

ExtiManager::ExtiManager() :
    exti_(exti::EXTI::get_instance()),
    callbacks_{nullptr}
{
}

/**
 * @brief Enables an EXTI interrupt on the specified pin.
 *
 * This function enables an EXTI interrupt on the specified pin. The pin
 * must be configured as an input pin and the EXTI source must be set.
 * The callback function will be called when the EXTI interrupt occurs.
 *
 * @param pin The pin on which to enable the EXTI interrupt.
 * @param callback The callback function to call when the EXTI interrupt occurs.
 * @param type The type of EXTI interrupt to enable (RISING, FALLING, or CHANGE).
 */
void ExtiManager::enablePinExtiInterrupt(pin_size_t pin, EXTICallback callback, exti::EXTI_Trigger type) {
    const PortPinPair& pp = port_pin_map[pin];
    if (pp.pin == gpio::Pin_Number::INVALID || pp.port == gpio::GPIO_Base::INVALID) {
        return;
    }

    auto id = static_cast<uint8_t>(pp.pin);
    if (id >= maxExtiLines_) {
        return;
    }

    // Store callback
    callbacks_[id] = callback;

    // Get GPIO instance
    auto portResult = gpio::GPIO::get_instance(pp.port);
    if (portResult.error() != gpio::GPIO_Error_Type::OK) {
        return;
    }
    auto& instance = portResult.value();

    // Use the currently set mode (input) in case user has already configured
    // the pin using the digital pinMode interface.
    //
    // NOTE:
    //  Only input is valid here as only input pins can have an exti
    auto currentMode = instance.get_pin_mode(pp.pin);
    if (currentMode != gpio::Pin_Mode::INPUT_FLOATING &&
        currentMode != gpio::Pin_Mode::INPUT_PULLUP &&
        currentMode != gpio::Pin_Mode::INPUT_PULLDOWN) {
        instance.set_pin_mode(pp.pin, gpio::Pin_Mode::INPUT_FLOATING);
    }

    // Map GPIO port to source port
    gpio::Source_Port sourcePort;
    switch (pp.port) {
        case gpio::GPIO_Base::GPIOA_BASE: sourcePort = gpio::Source_Port::SOURCE_IS_GPIOA; break;
        case gpio::GPIO_Base::GPIOB_BASE: sourcePort = gpio::Source_Port::SOURCE_IS_GPIOB; break;
        case gpio::GPIO_Base::GPIOC_BASE: sourcePort = gpio::Source_Port::SOURCE_IS_GPIOC; break;
        case gpio::GPIO_Base::GPIOD_BASE: sourcePort = gpio::Source_Port::SOURCE_IS_GPIOD; break;
        case gpio::GPIO_Base::INVALID:
        default: return;    // Invalid port
    }

    // Configure EXTI
    gpio::AFIO::get_instance().set_exti_source(sourcePort, pp.pin);
    exti_.clear_interrupt_flag(static_cast<exti::Interrupt_Flags>(id));
    exti_.init(static_cast<exti::EXTI_Line>(id), exti::EXTI_Mode::EXTI_INTERRUPT, type);

    // Configure NVIC
    IRQn_Type irq = extiToIrq(id);
    NVIC_SetPriority(irq, extiIRQPriority);
    NVIC_EnableIRQ(irq);
}

/**
 * @brief Disable an EXTI interrupt and its NVIC IRQ.
 *
 * This function will disable an EXTI interrupt and its NVIC IRQ if there are
 * no more callbacks registered for the same IRQ.
 *
 * @param pin The pin to disable the EXTI interrupt for.
 */
void ExtiManager::disablePinExtiInterrupt(pin_size_t pin) {
    const PortPinPair& pp = port_pin_map[pin];
    if (pp.pin == gpio::Pin_Number::INVALID) {
        return;
    }

    auto id = static_cast<uint8_t>(pp.pin);
    if (id >= maxExtiLines_) {
        return;
    }

    callbacks_[id] = nullptr;

    // Get the IRQ for this EXTI line
    auto irq = extiToIrq(id);

    // Check for unhandled callbacks
    bool disableIRQ = true;
    for (uint8_t i = 0U; i < maxExtiLines_; ++i) {
        if (extiToIrq(i) == irq && callbacks_[i] != nullptr) {
            disableIRQ = false;
            break;
        }
    }

    if (disableIRQ) {
        NVIC_DisableIRQ(irq);
    }
}

/**
 * @brief Handle an EXTI interrupt callback.
 *
 * This function is called by the interrupt handlers for EXTI lines 0-15.
 * It will clear the EXTI interrupt flag and then call the callback
 * function set for the corresponding pin.
 *
 * @param pin The pin number of the interrupt source.
 */
void ExtiManager::handleCallback(gpio::Pin_Number pin) {
    auto id = static_cast<uint8_t>(pin);
    if (id >= maxExtiLines_) {
        return;
    }

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
        for (uint8_t i = static_cast<uint8_t>(gpio::Pin_Number::PIN_5);
                i <= static_cast<uint8_t>(gpio::Pin_Number::PIN_9); ++i) {
            ExtiManager::get_instance().handleCallback(static_cast<gpio::Pin_Number>(i));
        }
    }
    void EXTI10_15_IRQHandler(void) {
        for (uint8_t i = static_cast<uint8_t>(gpio::Pin_Number::PIN_10);
                i <= static_cast<uint8_t>(gpio::Pin_Number::PIN_15); ++i) {
            ExtiManager::get_instance().handleCallback(static_cast<gpio::Pin_Number>(i));
        }
    }

} // extern "C"
