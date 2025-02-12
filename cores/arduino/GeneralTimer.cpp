
#include "Arduino.h"
#include "GeneralTimer.h"

constexpr uint16_t MAX_PERIOD = 0xFFFFU;
constexpr uint8_t maxNumTimers_ = TIMER_COUNT;

timer::TIMER& GeneralTimer::get_timer_instance(timer::TIMER_Base Base) {
    static timer::TIMER* timer_instances[TIMER_COUNT] = {nullptr};
    size_t index = static_cast<size_t>(Base);

    if (timer_instances[index] == nullptr) {
        auto result = timer::TIMER::get_instance(Base);
        if (result.error() != timer::TIMER_Error_Type::OK) {
#ifdef CORE_DEBUG
            core_debug("ERROR: Invalid timer instance!");
#endif
            __builtin_trap();
        }
        timer_instances[index] = &result.value();
    }
    return *timer_instances[index];
}

GeneralTimer& GeneralTimer::get_instance(timer::TIMER_Base Base) {
    switch (Base) {
        case timer::TIMER_Base::TIMER0_BASE: {
            static GeneralTimer GTimer0(Base);
            return GTimer0;
        }
        case timer::TIMER_Base::TIMER1_BASE: {
            static GeneralTimer GTimer1(Base);
            return GTimer1;
        }
        case timer::TIMER_Base::TIMER2_BASE: {
            static GeneralTimer GTimer2(Base);
            return GTimer2;
        }
        case timer::TIMER_Base::TIMER3_BASE: {
            static GeneralTimer GTimer3(Base);
            return GTimer3;
        }
        case timer::TIMER_Base::TIMER4_BASE: {
            static GeneralTimer GTimer4(Base);
            return GTimer4;
        }
        case timer::TIMER_Base::TIMER5_BASE: {
            static GeneralTimer GTimer5(Base);
            return GTimer5;
        }
        case timer::TIMER_Base::TIMER6_BASE: {
            static GeneralTimer GTimer6(Base);
            return GTimer6;
        }
        case timer::TIMER_Base::TIMER7_BASE: {
            static GeneralTimer GTimer7(Base);
            return GTimer7;
        }
        case timer::TIMER_Base::INVALID:
        default:
#ifdef CORE_DEBUG
            core_debug("Invalid TIMER instance!");
            __builtin_trap();
#endif
            static GeneralTimer dummy(Base);
            return dummy;
    }
}

std::array<timer_to_irq, maxNumTimers_> GeneralTimer::timer_up_irq {{
        {0U, TIMER0_UP_IRQn}, {1U, TIMER1_IRQn}, {2U, TIMER2_IRQn}, {3U, TIMER3_IRQn},
        {4U, TIMER4_IRQn}, {5U, TIMER5_IRQn}, {6U, TIMER6_IRQn}, {7U, TIMER7_UP_IRQn}
    }};

std::array<timer_to_irq, maxNumTimers_> GeneralTimer::timer_ch_irq {{
        {0U, TIMER0_Channel_IRQn}, {1U, TIMER1_IRQn}, {2U, TIMER2_IRQn}, {3U, TIMER3_IRQn},
        {4U, TIMER4_IRQn}, {5U, TIMER5_IRQn}, {6U, TIMER6_IRQn}, {7U, TIMER7_Channel_IRQn}
    }};

GeneralTimer::GeneralTimer(timer::TIMER_Base Base) :
    base_(Base),
    timer_(get_timer_instance(Base)),
    config_(timer::default_config),
    capture_config_(timer::default_capture),
    compare_config_(timer::default_compare),
    callbacks_{0, nullptr, {nullptr}},
           preemptPriority_(12U),
           subPriority_(0U)
{
    // Initialize channel modes array
    for (int i = 0; i < TIMER_CHANNELS; i++) {
        channel_modes_[i] = InputOutputMode::INVALID;
    }

    // Initialize companion channels array
    for (int i = 0; i < TIMER_CHANNELS; i++) {
        companionChannel[i] = false;
    }

    IRQn_Type upIrq = timerToUpIrq(static_cast<size_t>(Base));
    IRQn_Type ccIrq = timerToChIrq(static_cast<size_t>(Base));

    if (upIrq != INVALID_IRQ && ccIrq != INVALID_IRQ) {
        NVIC_SetPriority(upIrq, preemptPriority_);
        NVIC_EnableIRQ(upIrq);
        if (ccIrq != upIrq) {
            NVIC_SetPriority(ccIrq, preemptPriority_);
            NVIC_EnableIRQ(ccIrq);
        }
    }
}

/**
 * @brief Initializes the timer with the given configuration.
 *
 * This function is used to initialize the timer with the given configuration.
 * It is usually called once at the beginning of the program, before using the
 * timer.
 *
 * @param config The configuration for the timer.
 */
void GeneralTimer::begin(timer::TIMER_Config config) {
    timer_.init(config);
}

/**
 * @brief Starts the timer.
 *
 * This function is used to start the timer. If the callback for the update
 * interrupt is set, the update interrupt is enabled and the timer is enabled.
 * Then, the function iterates through each channel and starts any active
 * channels by calling startTimerChannel.
 *
 * NOTE: The timer will not run if the update interrupt callback is not set.
 */
void GeneralTimer::start() {
    // Setup update interrupt if callback exists
    if (callbacks_.up_callback) {
        timer_.clear_interrupt_flag(timer::Interrupt_Flags::INTR_FLAG_UP);
        timer_.set_interrupt_enable(timer::Interrupt_Type::INTR_UPIE, true);
        timer_.enable();
    }

    for (int i = 0; i < TIMER_CHANNELS; i++) {
        startTimerChannel(static_cast<timer::Timer_Channel>(i));
    }
}

/**
 * @brief Stops the timer.
 *
 * This function is used to stop the timer. It clears all interrupts and
 * disables the timer and all channel interrupts. It is usually called before
 * the end of the program to ensure that the timer is disabled and that the
 * interrupts are cleared.
 */
void GeneralTimer::stop() {
    // Clear interrupt flags
    timer_.clear_all_interrupt_flags();
    // Disable interrupts and timer
    timer_.disable_all_interrupts();
    timer_.disable();
}

/**
 * @brief Generates a software event to trigger the update event.
 *
 * This function generates a software event to trigger the update event.
 * It is used to trigger the update interrupt if the timer is already running.
 */
void GeneralTimer::refresh() {
    timer_.generate_software_event(timer::Event_Source::EVENT_SRC_UPG);
}

void GeneralTimer::startTimerChannel(timer::Timer_Channel channel) {
    // Clear flag and enable interrupt
    if (callbacks_.channel_callbacks[static_cast<size_t>(channel)]) {
        timer::Interrupt_Flags flag = convertToInterruptFlag(channel);
        if (flag != timer::Interrupt_Flags::INVALID) {
            timer_.clear_interrupt_flag(flag);
        }
        timer::Interrupt_Type type = convertToInterrupt(channel);
        if (type != timer::Interrupt_Type::INVALID) {
            timer_.set_interrupt_enable(type, true);
        }
    }

    InputOutputMode mode = channel_modes_[static_cast<size_t>(channel)];

    switch (mode) {
        case InputOutputMode::COMPARE:
            timer_.set_channel_output_enable(channel, true);
            timer_.enable();
            {
                // enable companion channel
                timer::Timer_Channel companion = getCompanionChannel(channel);
                timer_.set_compliment_output_enable(companion, true);
                if (callbacks_.channel_callbacks[static_cast<size_t>(channel)]) {
                    timer_.clear_interrupt_flag(convertToInterruptFlag(companion));
                    timer_.set_interrupt_enable(convertToInterrupt(companion), true);
                }
            }
            break;
        case InputOutputMode::RISING:
        case InputOutputMode::FALLING:
            timer_.set_channel_output_enable(channel, true);
            timer_.enable();
            break;
        case InputOutputMode::TIMING:
        case InputOutputMode::OUTPUT:
        case InputOutputMode::CLEAR:
        case InputOutputMode::TOGGLE:
        case InputOutputMode::FORCE_LOW:
        case InputOutputMode::FORCE_HIGH:
        case InputOutputMode::PWM0:
        case InputOutputMode::PWM1:
            if (companionChannel[static_cast<size_t>(channel)]) {
                timer_.set_compliment_output_enable(channel, true);
            } else {
                timer_.set_channel_output_enable(channel, true);
            }
            timer_.set_primary_output_enable(true);
            timer_.enable();
            break;
        case InputOutputMode::INVALID:
        default:
            break;
    }
}

/**
 * @brief Disable the specified timer channel.
 *
 * This function disables the specified timer channel and clears any pending interrupts.
 *
 * @param channel The timer channel to disable. Valid values are Timer_Channel::CH0, Timer_Channel::CH1, Timer_Channel::CH2, or Timer_Channel::CH3.
 */
void GeneralTimer::stopTimerChannel(timer::Timer_Channel channel) {
    timer::Interrupt_Flags flag = convertToInterruptFlag(channel);
    timer::Interrupt_Type type = convertToInterrupt(channel);

    if (flag != timer::Interrupt_Flags::INVALID && type != timer::Interrupt_Type::INVALID) {
        timer_.clear_interrupt_flag(flag);
        timer_.set_interrupt_enable(type, false);
        timer_.set_channel_output_enable(channel, false);
        InputOutputMode mode = channel_modes_[static_cast<size_t>(channel)];
        if (mode == InputOutputMode::COMPARE) {
            timer::Timer_Channel companion = getCompanionChannel(channel);
            if (companion != timer::Timer_Channel::INVALID) {
                timer::Interrupt_Flags companion_flag = convertToInterruptFlag(companion);
                timer::Interrupt_Type companion_type = convertToInterrupt(companion);
                if (companion_flag != timer::Interrupt_Flags::INVALID && companion_type != timer::Interrupt_Type::INVALID) {
                    timer_.clear_interrupt_flag(companion_flag);
                    timer_.set_interrupt_enable(companion_type, false);
                }
            }
        }
    }
}

/**
 * @brief Sets the prescaler value for the timer.
 *
 * This function sets the prescaler value for the timer by updating the
 * prescaler register. The prescaler value is a 16-bit unsigned integer.
 * The reload event is generated when the counter reaches the auto-reload
 * value or when the counter is cleared.
 *
 * @param[in] prescaler The 16-bit prescaler value to set.
 */
void GeneralTimer::setPrescaler(uint16_t prescaler) {
    timer_.set_prescaler_reload(prescaler - 1U, timer::PSC_Reload::RELOAD_NOW);
}

/**
 * @brief Gets the current prescaler value of the timer.
 *
 * This function retrieves the current prescaler value of the timer by reading
 * the prescaler register and returns the value as a 16-bit unsigned integer.
 *
 * @return The current prescaler value of the timer.
 */
uint16_t GeneralTimer::getPrescaler() {
    return timer_.get_prescaler() + 1U;
}

/**
 * @brief Sets the rollover value for the timer.
 *
 * This function sets the rollover value for the timer based on the provided
 * value and format. The rollover value is the value that the timer counts up
 * to before it resets to 0. The format parameter determines the unit of the
 * rollover value. The unit can be microseconds, hertz, or timer ticks.
 *
 * @param[in] value The value to set as the rollover value.
 * @param[in] format The unit of the value parameter.
 */
void GeneralTimer::setRolloverValue(uint32_t value, TimerFormat format) {
    uint32_t tickValue = 0U;
    uint16_t prescalerValue = 0U;
    uint32_t cycle = 0U;

    switch (format) {
        case TimerFormat::US:
            cycle = value * (getTimerClockFrequency() / 1'000'000U);
            prescalerValue = (cycle / 0x10000U) + 1U;
            timer_.set_prescaler_reload(prescalerValue - 1U, timer::PSC_Reload::RELOAD_UPDATE);
            tickValue = cycle / prescalerValue;
            break;
        case TimerFormat::HERTZ:
            cycle = getTimerClockFrequency() / value;
            prescalerValue = (cycle / 0x10000U) + 1U;
            timer_.set_prescaler_reload(prescalerValue - 1U, timer::PSC_Reload::RELOAD_UPDATE);
            tickValue = cycle / prescalerValue;
            break;
        case TimerFormat::TICK:
        default:
            tickValue = value;
            break;
    }

    timer_.set_auto_reload((tickValue > 0U) ? tickValue - 1U : 0U);
}

/**
 * @brief Gets the rollover value for the timer.
 *
 * This function gets the rollover value for the timer based on the provided
 * format. The rollover value is the value that the timer counts up to before
 * it resets to 0. The format parameter determines the unit of the rollover
 * value. The unit can be microseconds, hertz, or timer ticks.
 *
 * @param[in] format The unit of the value to return.
 * @return The rollover value in the specified format.
 */
uint32_t GeneralTimer::getRolloverValue(TimerFormat format) {
    uint16_t value = timer_.get_auto_reload();
    uint16_t prescalerValue = timer_.get_prescaler() + 1U;

    switch (format) {
        case TimerFormat::US:
            return (((value + 1U) * prescalerValue * 1'000'000.0) / getTimerClockFrequency());
        case TimerFormat::HERTZ:
            return (getTimerClockFrequency() / ((value + 1U) * prescalerValue));
        case TimerFormat::TICK:
        default:
            return value + 1U;
    }

    return value + 1U;
}

/**
 * @brief Sets the current counter value of the timer.
 *
 * This function sets the current counter value of the timer based on the
 * provided value and format. The counter value is the value that the timer
 * counts up from. The format parameter determines the unit of the counter
 * value. The unit can be microseconds, hertz, or timer ticks.
 *
 * @param[in] count The value to set as the counter value.
 * @param[in] format The unit of the value parameter.
 */
void GeneralTimer::setCounter(uint16_t count, TimerFormat format) {
    uint16_t counterValue = 0U;
    uint16_t prescalerValue = timer_.get_prescaler() + 1U;

    switch (format) {
        case TimerFormat::US:
            counterValue = ((count * (getTimerClockFrequency() / 1'000'000U)) / prescalerValue);
            break;
        case TimerFormat::HERTZ:
            counterValue = (getTimerClockFrequency() / (count * prescalerValue));
            break;
        case TimerFormat::TICK:
        default:
            counterValue = count;
            break;
    }

    timer_.set_counter_value(counterValue);
}

/**
 * @brief Gets the current counter value of the timer.
 *
 * This function gets the current counter value of the timer based on the
 * provided format. The counter value is the value that the timer counts up
 * from. The format parameter determines the unit of the counter value. The
 * unit can be microseconds, hertz, or timer ticks.
 *
 * @param[in] format The unit of the value to return.
 * @return The current counter value in the specified format.
 */
uint32_t GeneralTimer::getCounter(TimerFormat format) {
    uint32_t counterValue = timer_.get_counter();
    uint16_t prescalerValue = timer_.get_prescaler() + 1U;

    switch (format) {
        case TimerFormat::US:
            return ((counterValue * prescalerValue * 1'000'000.0) / getTimerClockFrequency());
        case TimerFormat::HERTZ:
            return (getTimerClockFrequency() / (counterValue * prescalerValue));
        case TimerFormat::TICK:
        default:
            return counterValue;
    }

    return counterValue;
}

/**
 * @brief Configures the mode of the specified timer channel.
 *
 * This function configures the mode of the specified timer channel. The mode
 * can be one of the following: TIMING, OUTPUT, CLEAR, TOGGLE, FORCE_LOW,
 * FORCE_HIGH, PWM0, PWM1, RISING, FALLING, or COMPARE. The mode setting
 * determines how the timer channel behaves.
 *
 * @param[in] channel The timer channel to configure. Valid values are
 *                     Timer_Channel::CH0, Timer_Channel::CH1, Timer_Channel::CH2, or
 *                     Timer_Channel::CH3.
 * @param[in] mode The mode to set for the specified timer channel.
 * @param[in] pin The pin number associated with the specified timer channel.
 */
void GeneralTimer::setChannelMode(timer::Timer_Channel channel, InputOutputMode mode, pin_size_t pin) {
    if (static_cast<uint8_t>(channel) > static_cast<uint8_t>(TIMER_CHANNELS)) {
        return;
    }

    uint8_t companionNumber = 0U;
    channel_modes_[static_cast<size_t>(channel)] = mode;

    timer::TIMER_Input_Capture captureConfig = {
        .digital_filter = 0U,
        .polarity = timer::Polarity_Select::HIGH_RISING,
        .source_select = timer::Input_Capture_Select::IO_INPUT_CI0FE0,
        .prescaler = timer::Input_Capture_Prescaler::DIV1
    };

    timer::TIMER_Output_Compare compareConfig = {
        .state = timer::Output_Compare_State::OC_DISABLE,
        .companion_state = timer::Output_Compare_State::OC_DISABLE,
        .polarity = timer::Polarity_Select::HIGH_RISING,
        .companion_polarity = timer::Polarity_Select::HIGH_RISING,
        .idle_state = timer::Idle_State::IDLE_LOW,
        .companion_idle_state = timer::Idle_State::IDLE_LOW
    };

    // Main switch case for handling different modes
    switch (mode) {
        case InputOutputMode::TIMING:
            timer_.set_output_mode(channel, timer::Output_Compare_Mode::OC_TIMING_MODE);
            timer_.output_compare_init(channel, compareConfig);
            break;
        case InputOutputMode::OUTPUT:
            timer_.set_output_mode(channel, timer::Output_Compare_Mode::OC_OUTPUT_MODE);
            timer_.output_compare_init(channel, compareConfig);
            break;
        case InputOutputMode::CLEAR:
            timer_.set_output_mode(channel, timer::Output_Compare_Mode::OC_CLEAR_MODE);
            timer_.output_compare_init(channel, compareConfig);
            break;
        case InputOutputMode::TOGGLE:
            timer_.set_output_mode(channel, timer::Output_Compare_Mode::OC_TOGGLE_MODE);
            timer_.output_compare_init(channel, compareConfig);
            break;
        case InputOutputMode::FORCE_LOW:
            timer_.set_output_mode(channel, timer::Output_Compare_Mode::OC_FORCE_LOW_MODE);
            timer_.output_compare_init(channel, compareConfig);
            break;
        case InputOutputMode::FORCE_HIGH:
            timer_.set_output_mode(channel, timer::Output_Compare_Mode::OC_FORCE_HIGH_MODE);
            timer_.output_compare_init(channel, compareConfig);
            break;
        case InputOutputMode::PWM0:
            timer_.set_output_mode(channel, timer::Output_Compare_Mode::OC_PWM_MODE0);
            timer_.output_compare_init(channel, compareConfig);
            timer_.set_output_shadow(channel, timer::Output_Compare_Shadow::OC_SHADOW_ENABLE);
            timer_.set_output_fast(channel, timer::Output_Compare_Fast::OC_FAST_DISABLE);
            break;
        case InputOutputMode::PWM1:
            timer_.set_output_mode(channel, timer::Output_Compare_Mode::OC_PWM_MODE1);
            timer_.output_compare_init(channel, compareConfig);
            timer_.set_output_shadow(channel, timer::Output_Compare_Shadow::OC_SHADOW_ENABLE);
            timer_.set_output_fast(channel, timer::Output_Compare_Fast::OC_FAST_DISABLE);
            break;
        case InputOutputMode::RISING:
            captureConfig.polarity = timer::Polarity_Select::HIGH_RISING;
            timer_.input_capture_init(channel, captureConfig);
            break;
        case InputOutputMode::FALLING:
            captureConfig.polarity = timer::Polarity_Select::LOW_FALLING;
            timer_.input_capture_init(channel, captureConfig);
            break;
        // TODO:
        //  Can we even support this?
        //  Leaving commented for now
        //case InputOutputMode::DUAL:
        //  captureConfig.polarity = timer::Polarity_Select::DUAL_EDGE;
        //  if (captureConfig) {
        //      timer_.input_capture_init(channel, captureConfig);
        //  }
        //  break;
        case InputOutputMode::COMPARE:
            captureConfig.polarity = timer::Polarity_Select::HIGH_RISING;
            captureConfig.source_select = timer::Input_Capture_Select::IO_INPUT_CI0FE0;
            timer_.input_capture_init(channel, captureConfig);

            // Configure companion channel
            companionNumber = static_cast<uint8_t>(getCompanionChannel(channel));
            channel_modes_[static_cast<size_t>(companionNumber)] = mode;

            captureConfig.polarity = timer::Polarity_Select::LOW_FALLING;
            captureConfig.source_select = timer::Input_Capture_Select::IO_INPUT_CI1FE0;
            timer_.input_capture_init(getCompanionChannel(channel), captureConfig);
            break;
        case InputOutputMode::INVALID:
        default:
            break;
    }

    // Store configuration
    capture_config_ = captureConfig;
    compare_config_ = compareConfig;

    if (pin != NO_PIN) {
        if (getChannelFromPin(pin) == channel) {
            pinOpsPinout(TIMER_PinOps, pin);
            if ((mode == InputOutputMode::RISING) ||
                    (mode == InputOutputMode::FALLING) ||
                    (mode == InputOutputMode::COMPARE)) {
                pinMode(pin, INPUT);
            }
        } else {
            return; // Pin and channel mismatch
        }
        companionChannel[static_cast<uint8_t>(channel)] = (getCompanionChannelFromPin(pin) != timer::Timer_Channel::CH0);
    }
}

/**
 * @brief Gets the current output compare mode of the specified timer channel.
 *
 * @param channel The timer channel to query. Valid values are Timer_Channel::CH0, Timer_Channel::CH1, Timer_Channel::CH2, or Timer_Channel::CH3.
 * @return The current output compare mode of the specified timer channel.
 */
InputOutputMode GeneralTimer::getChannelMode(timer::Timer_Channel channel) {
    if (channel >= timer::Timer_Channel::CH0 && channel < timer::Timer_Channel::INVALID) {
        return channel_modes_[static_cast<size_t>(channel)];
    }
    return InputOutputMode::INVALID;
}

/**
 * @brief Enables or disables the auto-reload shadow feature.
 *
 * If the auto-reload shadow feature is enabled, the timer auto-reload value is
 * loaded from the shadow register when the counter reaches zero. If the feature
 * is disabled, the timer auto-reload value is loaded from the CAR register
 * directly.
 *
 * @param enable Set to true to enable the auto-reload shadow feature or false
 *               to disable it.
 */
void GeneralTimer::setAutoReloadEnable(bool enable) {
    timer_.set_auto_reload_shadow_enable(enable);
}

/**
 * @brief Sets the capture compare value for a given timer channel.
 *
 * This function sets the capture compare value for the specified timer channel.
 * The capture compare value is the value that is compared against the timer
 * counter value. The compare value is always an unsigned 32-bit integer.
 *
 * The compare value can be specified in different formats. The format is
 * specified by the CCFormat enumeration. The formats are:
 *   - US: microseconds
 *   - HERTZ: frequency in Hertz
 *   - PERCENT: percentage of the counter period
 *   - B1-B16: bit resolution formats
 *   - TICK: counter value in timer ticks
 *
 * If the compare value is set to a value that is greater than the auto-reload
 * value, the compare value is clamped to the auto-reload value minus one.
 *
 * @param channel The timer channel to set the capture compare value for.
 *                Valid values are Timer_Channel::CH0, Timer_Channel::CH1,
 *                Timer_Channel::CH2, or Timer_Channel::CH3.
 * @param value The capture compare value to set.
 * @param format The format of the compare value. Defaults to CCFormat::TICK.
 */
void GeneralTimer::setCaptureCompare(timer::Timer_Channel channel, uint32_t value, CCFormat format) {
    if (channel == timer::Timer_Channel::INVALID) {
        return;
    }

    uint32_t prescalerValue = static_cast<uint32_t>(timer_.get_prescaler() + 1U);
    uint32_t ccValue = 0U;

    switch (format) {
        case CCFormat::US:
            ccValue = ((value * (getTimerClockFrequency() / 1'000'000U)) / prescalerValue);
            break;
        case CCFormat::HERTZ:
            ccValue = getTimerClockFrequency() / (value * prescalerValue);
            break;
        case CCFormat::PERCENT:
            ccValue = ((timer_.get_auto_reload() + 1U) * value) / 100U;
            break;
        case CCFormat::B1: case CCFormat::B2: case CCFormat::B3: case CCFormat::B4:
        case CCFormat::B5: case CCFormat::B6: case CCFormat::B7: case CCFormat::B8:
        case CCFormat::B9: case CCFormat::B10: case CCFormat::B11: case CCFormat::B12:
        case CCFormat::B13: case CCFormat::B14: case CCFormat::B15: case CCFormat::B16:
            ccValue = ((timer_.get_auto_reload() + 1U) * value) / ((1U << static_cast<size_t>(format)) - 1U);
            break;
        case CCFormat::TICK:
        default:
            ccValue = value;
            break;
    }

    if ((timer_.get_auto_reload() == MAX_PERIOD) && (ccValue == MAX_PERIOD + 1U)) {
        ccValue = MAX_PERIOD;
    }

    timer_.set_capture_compare(channel, ccValue);
}

/**
 * @brief Returns the capture compare value for a given timer channel.
 *
 * This function returns the capture compare value for the specified timer
 * channel. The capture compare value is the value that is compared against the
 * timer counter value. The compare value is always an unsigned 32-bit integer.
 *
 * The compare value can be specified in different formats. The format is
 * specified by the CCFormat enumeration. The formats are:
 *   - US: microseconds
 *   - HERTZ: frequency in Hertz
 *   - PERCENT: percentage of the counter period
 *   - B1-B16: bit resolution formats
 *   - TICK: counter value in timer ticks
 *
 * If the format is not specified, the function returns the compare value in
 * timer tick format.
 *
 * @param channel The timer channel to get the capture compare value for.
 *                Valid values are Timer_Channel::CH0, Timer_Channel::CH1,
 *                Timer_Channel::CH2, or Timer_Channel::CH3.
 * @param format The format of the compare value. Defaults to CCFormat::TICK.
 * @return The capture compare value in the specified format.
 */
uint32_t GeneralTimer::getCaptureCompare(timer::Timer_Channel channel, CCFormat format) {
    if (channel == timer::Timer_Channel::INVALID || format == CCFormat::INVALID) {
        return 0U;
    }

    uint32_t ccValue = timer_.get_capture_compare(channel);

    // Fast path for TICK format
    if (format == CCFormat::TICK) {
        return ccValue;
    }

    // Handle bit resolution formats efficiently (B1-B16)
    uint8_t formatValue = static_cast<size_t>(format);
    if (formatValue >= static_cast<size_t>(CCFormat::B1) &&
            formatValue <= static_cast<size_t>(CCFormat::B16)) {
        uint16_t autoReload = timer_.get_auto_reload();
        return (ccValue * ((1U << formatValue) - 1U)) / autoReload;
    }

    uint32_t prescaler = timer_.get_prescaler() + 1U;

    // Handle remaining formats
    switch (format) {
        case CCFormat::US:
            return (ccValue * prescaler * 1'000'000.0) / (getTimerClockFrequency());
        case CCFormat::HERTZ:
            return (getTimerClockFrequency() / (ccValue * prescaler));
        case CCFormat::PERCENT:
            return (ccValue * 100U) / timer_.get_auto_reload();
        case CCFormat::TICK:    // Already handled above
        case CCFormat::INVALID: // Already handled above
        case CCFormat::B1: case CCFormat::B2: case CCFormat::B3: case CCFormat::B4:
        case CCFormat::B5: case CCFormat::B6: case CCFormat::B7: case CCFormat::B8:
        case CCFormat::B9: case CCFormat::B10: case CCFormat::B11: case CCFormat::B12:
        case CCFormat::B13: case CCFormat::B14: case CCFormat::B15: case CCFormat::B16:
            // Already handled in bit resolution section
            return ccValue; // Fallback, should never reach here
    }

    return ccValue; // Fallback, should never reach here
}

/**
 * @brief Initializes or updates PWM output on a specified pin.
 *
 * @param channel The timer channel to output PWM signal
 * @param pin Pin number to output PWM signal
 * @param frequency PWM frequency in Hz
 * @param dutycycle PWM dutycycle in percent (0-100)
 * @param UPCallback Timer update callback (optional)
 * @param CCCallback Timer channel callback (optional)
 *
 * @note If pin is not PWM capable or invalid, function returns without effect
 */
void GeneralTimer::setPWM(timer::Timer_Channel channel, pin_size_t pin, uint32_t frequency, uint32_t dutycycle, TimerCallback UPCallback, TimerCallback CCCallback) {
    setChannelMode(channel, InputOutputMode::PWM0, pin);
    setRolloverValue(frequency, TimerFormat::HERTZ);
    setCaptureCompare(channel, dutycycle, CCFormat::PERCENT);
    if (UPCallback) { attachInterrupt(UPCallback); }
    if (CCCallback) { attachInterrupt(CCCallback, channel); }
    start();
}

/**
 * @brief Sets the interrupt priority of the timer peripheral.
 *
 * This function sets the interrupt priority of the timer peripheral.
 * The priority is specified by two parameters: prepriority and subpriority.
 * The prepriority specifies the priority of the interrupt in the NVIC,
 * and the subpriority specifies the priority of the interrupt within the
 * timer peripheral. The subpriority is used to prioritize interrupts within
 * the peripheral. If two interrupts have the same prepriority, the one with
 * the higher subpriority takes precedence.
 *
 * @param preemptPriority The prepriority of the interrupt.
 * @param subPriority The subpriority of the interrupt.
 */
void GeneralTimer::setInterruptPriority(uint8_t preemptPriority, uint8_t subPriority) {
    IRQn_Type upIrq = timerToUpIrq(static_cast<size_t>(base_));
    IRQn_Type ccIrq = timerToChIrq(static_cast<size_t>(base_));

    if (upIrq == INVALID_IRQ || ccIrq == INVALID_IRQ) return;

    NVIC_SetPriority(upIrq, preemptPriority);

    if (ccIrq != upIrq) {
        NVIC_SetPriority(ccIrq, preemptPriority);
    }

    preemptPriority_ = preemptPriority;
    subPriority_ = subPriority;
}

/**
 * @brief Attaches an interrupt callback function to the timer's update interrupt.
 *
 * This function sets the interrupt callback function for the timer's update
 * interrupt. The callback function will be called when the update interrupt
 * occurs. If the callback is null, the interrupt is disabled.
 *
 * @param callback The callback function to attach to the update interrupt.
 */
void GeneralTimer::attachInterrupt(TimerCallback callback) {
    if (callbacks_.up_callback) {
        callbacks_.up_callback = callback;
        callbacks_.active_callbacks |= (callback ? 0x01U : 0x00U);
    } else {
        callbacks_.up_callback = callback;
        callbacks_.active_callbacks |= (callback ? 0x01U : 0x00U);
        if (callback) {
            timer_.clear_interrupt_flag(timer::Interrupt_Flags::INTR_FLAG_UP);
            timer_.set_interrupt_enable(timer::Interrupt_Type::INTR_UPIE, true);
        }
    }
}

/**
 * @brief Attaches an interrupt callback function to a specific timer channel.
 *
 * This function sets the interrupt callback function for a specific timer channel.
 * The callback function will be called when the channel interrupt occurs. If the
 * callback is null, the interrupt is disabled. The function also sets the
 * interrupt enable bit for the channel if the callback is non-null.
 *
 * @param callback The callback function to attach to the channel interrupt.
 * @param channel The timer channel to attach the interrupt to (CH0-CH3).
 * @note If channel is invalid or out of range, function returns without action.
 */
void GeneralTimer::attachInterrupt(TimerCallback callback, timer::Timer_Channel channel) {
    if ((static_cast<size_t>(channel) < 0) || (static_cast<size_t>(channel) > TIMER_CHANNELS)) return;
    if (callbacks_.channel_callbacks[static_cast<size_t>(channel)]) {
        callbacks_.channel_callbacks[static_cast<size_t>(channel)] = callback;
        callbacks_.active_callbacks |= (callback ? (1U << (static_cast<size_t>(channel) + 1U)) : 0x00U);
    } else {
        callbacks_.channel_callbacks[static_cast<size_t>(channel)] = callback;
        callbacks_.active_callbacks |= (callback ? (1U << (static_cast<size_t>(channel) + 1U)) : 0x00U);
        if (callback) {
            timer_.clear_interrupt_flag(convertToInterruptFlag(channel));
            timer_.set_interrupt_enable(convertToInterrupt(channel), true);
        }
    }
}

/**
 * @brief Detaches the update interrupt callback from the timer.
 *
 * This function disables the update interrupt and clears the associated callback.
 */
void GeneralTimer::detachInterrupt() {
    IRQn_Type upIrq = timerToUpIrq(static_cast<size_t>(base_));
    if (upIrq == INVALID_IRQ) return;
    timer_.set_interrupt_enable(timer::Interrupt_Type::INTR_UPIE, false);
    callbacks_.up_callback = nullptr;
}

/**
 * @brief Detaches the interrupt callback from the specified timer channel.
 *
 * This function disables the interrupt and clears the associated callback for the
 * specified timer channel. If the channel is invalid, the function returns without
 * action.
 *
 * @param channel The timer channel to detach the interrupt from (CH0-CH3).
 */
void GeneralTimer::detachInterrupt(timer::Timer_Channel channel) {
    if (static_cast<size_t>(channel) >= static_cast<size_t>(TIMER_CHANNELS)) return;
    IRQn_Type ccIrq = timerToChIrq(static_cast<size_t>(base_));
    if (ccIrq == INVALID_IRQ) return;
    timer_.set_interrupt_enable(convertToInterrupt(channel), false);
    callbacks_.channel_callbacks[static_cast<size_t>(channel)] = nullptr;
}

/**
 * @brief Checks if an interrupt callback is registered for the timer update interrupt.
 * @return true if an interrupt callback exists, false if no callback
 */
bool GeneralTimer::hasInterrupt() {
    return callbacks_.up_callback != nullptr;
}

/**
 * @brief Checks if an interrupt callback is registered for the specified timer channel.
 * @param channel The timer channel to check for an interrupt callback (CH0-CH3).
 * @return true if an interrupt callback exists, false if no callback
 */
bool GeneralTimer::hasInterrupt(timer::Timer_Channel channel) {
    if ((static_cast<size_t>(channel) < 0) || (static_cast<size_t>(channel) > static_cast<size_t>(TIMER_CHANNELS))) {
        return false;
    }
    return callbacks_.channel_callbacks[static_cast<size_t>(channel)] != nullptr;
}

/**
 * @brief Returns the clock frequency of the timer.
 *
 * This function returns the clock frequency of the timer in Hz. The clock
 * frequency is determined by the timer's clock source and APB prescaler. If the
 * timer's clock source is invalid, the function returns 0.
 *
 * @return The clock frequency of the timer in Hz.
 */
uint32_t GeneralTimer::getTimerClockFrequency() {
    rcu::Clock_Frequency timer_source = rcu::Clock_Frequency::CK_APB1;
    rcu::APB_Prescaler prescaler = rcu::APB_Prescaler::INVALID;

    switch (base_) {
        case timer::TIMER_Base::TIMER0_BASE:
        case timer::TIMER_Base::TIMER7_BASE:
            timer_source = rcu::Clock_Frequency::CK_APB2;
            prescaler = RCU_I.get_apb2_prescaler();
            break;
        case timer::TIMER_Base::TIMER1_BASE:
        case timer::TIMER_Base::TIMER2_BASE:
        case timer::TIMER_Base::TIMER3_BASE:
        case timer::TIMER_Base::TIMER4_BASE:
        case timer::TIMER_Base::TIMER5_BASE:
        case timer::TIMER_Base::TIMER6_BASE:
            timer_source = rcu::Clock_Frequency::CK_APB1;
            prescaler = RCU_I.get_apb1_prescaler();
            break;
        case timer::TIMER_Base::INVALID:
        default:
            return 0U;
    }

    if (prescaler == rcu::APB_Prescaler::INVALID) return 0U;

    return (prescaler == rcu::APB_Prescaler::CKAHB_DIV1) ?
           RCU_I.get_clock_frequency(timer_source) :
           RCU_I.get_clock_frequency(timer_source) * 2U;
}

/**
 * @brief Converts a timer channel to its corresponding status flag
 *
 * This function converts a timer channel to its corresponding status flag.
 * The status flag is used to check the status of the timer channel.
 *
 * @param channel Timer channel to convert
 * @return Corresponding status flag for the given channel
 */
inline timer::Status_Flags GeneralTimer::convertToFlag(timer::Timer_Channel channel) {
    switch (channel) {
        case timer::Timer_Channel::CH0: return timer::Status_Flags::FLAG_CH0;
        case timer::Timer_Channel::CH1: return timer::Status_Flags::FLAG_CH1;
        case timer::Timer_Channel::CH2: return timer::Status_Flags::FLAG_CH2;
        case timer::Timer_Channel::CH3: return timer::Status_Flags::FLAG_CH3;
        case timer::Timer_Channel::INVALID:
        default: return timer::Status_Flags::INVALID;
    }
}

/**
 * @brief Converts a timer channel to its corresponding interrupt flag
 *
 * This function converts a timer channel to its corresponding interrupt flag.
 * The interrupt flag is used to check the status of the timer channel interrupt
 * and to clear the interrupt flag.
 *
 * @param channel Timer channel to convert
 * @return Corresponding interrupt flag for the given channel
 */
inline timer::Interrupt_Flags GeneralTimer::convertToInterruptFlag(timer::Timer_Channel channel) {
    switch (channel) {
        case timer::Timer_Channel::CH0: return timer::Interrupt_Flags::INTR_FLAG_CH0;
        case timer::Timer_Channel::CH1: return timer::Interrupt_Flags::INTR_FLAG_CH1;
        case timer::Timer_Channel::CH2: return timer::Interrupt_Flags::INTR_FLAG_CH2;
        case timer::Timer_Channel::CH3: return timer::Interrupt_Flags::INTR_FLAG_CH3;
        case timer::Timer_Channel::INVALID:
        default: return timer::Interrupt_Flags::INVALID;
    }
}

/**
 * @brief Converts a timer channel to its corresponding interrupt type
 *
 * This function converts a timer channel to its corresponding interrupt type.
 * The interrupt type is used to enable or disable the timer channel interrupt
 * and to clear the interrupt flag.
 *
 * @param channel Timer channel to convert
 * @return Corresponding interrupt type for the given channel
 */
inline timer::Interrupt_Type GeneralTimer::convertToInterrupt(timer::Timer_Channel channel) {
    switch (channel) {
        case timer::Timer_Channel::CH0: return timer::Interrupt_Type::INTR_CH0IE;
        case timer::Timer_Channel::CH1: return timer::Interrupt_Type::INTR_CH1IE;
        case timer::Timer_Channel::CH2: return timer::Interrupt_Type::INTR_CH2IE;
        case timer::Timer_Channel::CH3: return timer::Interrupt_Type::INTR_CH3IE;
        case timer::Timer_Channel::INVALID:
        default: return timer::Interrupt_Type::INVALID;
    }
}

/**
 * @brief Returns the companion timer channel associated with a given timer channel
 *
 * The companion channel is the channel that is paired with the given channel
 * for complementary outputs. If the given channel is invalid, the function
 * returns `timer::Timer_Channel::INVALID`.
 *
 * @param channel The timer channel to get the companion for
 * @return The companion timer channel for the given channel
 */
inline timer::Timer_Channel GeneralTimer::getCompanionChannel(timer::Timer_Channel channel) {
    switch (channel) {
        case timer::Timer_Channel::CH0: return timer::Timer_Channel::CH1;
        case timer::Timer_Channel::CH1: return timer::Timer_Channel::CH0;
        case timer::Timer_Channel::CH2: return timer::Timer_Channel::CH3;
        case timer::Timer_Channel::CH3: return timer::Timer_Channel::CH2;
        case timer::Timer_Channel::INVALID:
        default: return timer::Timer_Channel::INVALID;
    }
}

/**
 * @brief Converts a pin number to its corresponding timer channel
 *
 * This function takes a pin number and returns the corresponding timer channel
 * associated with that pin. If the pin is invalid, the function returns
 * `timer::Timer_Channel::INVALID`.
 *
 * @param pin The pin number to convert
 * @return The corresponding timer channel for the given pin
 */
inline timer::Timer_Channel GeneralTimer::getChannelFromPin(pin_size_t pin) {
    auto instanceBase = getPinOpsPeripheralBase<TIMERPinOps, timer::TIMER_Base>(TIMER_PinOps, pin);
    if (instanceBase == timer::TIMER_Base::INVALID) {
        return timer::Timer_Channel::INVALID;
    }

    uint32_t packedPinOps = getPackedPinOps(TIMER_PinOps, instanceBase, pin);
    if (packedPinOps == invalidValue) {
        return timer::Timer_Channel::INVALID;
    }
    uint8_t channel_num = getPackedPinChannel(packedPinOps);
    return static_cast<timer::Timer_Channel>(channel_num);
}

/**
 * @brief Converts a pin number to its corresponding timer channel companion
 *
 * This function takes a pin number and returns the corresponding timer channel
 * companion associated with that pin. If the pin is invalid, the function
 * returns `timer::Timer_Channel::INVALID`.
 *
 * @param pin The pin number to convert
 * @return The corresponding timer channel companion for the given pin
 */
inline timer::Timer_Channel GeneralTimer::getCompanionChannelFromPin(pin_size_t pin) {
    auto instanceBase = getPinOpsPeripheralBase<TIMERPinOps, timer::TIMER_Base>(TIMER_PinOps, pin);
    if (instanceBase == timer::TIMER_Base::INVALID) {
        return timer::Timer_Channel::INVALID;
    }

    uint32_t packedPinOps = getPackedPinOps(TIMER_PinOps, instanceBase, pin);
    if (packedPinOps == invalidValue) {
        return timer::Timer_Channel::INVALID;
    }
    uint8_t channel_num = getPackedPinChOn(packedPinOps);
    return static_cast<timer::Timer_Channel>(channel_num);
}

/**
 * @brief Handles timer interrupts for update and channel interrupts
 *
 * This function is the interrupt service routine (ISR) for timer interrupts.
 * It checks the active interrupt flags and calls the corresponding callback
 * functions for each active interrupt. The `callbacks_.active_callbacks` bitfield
 * is used to quickly determine which interrupts are active and need to be
 * processed.
 *
 * The function first checks the update interrupt flag and calls the update
 * callback function if it is active. Then, it processes the channel interrupts
 * only if any are active. The channel interrupts are processed in order of
 * increasing channel number (CH0, CH1, CH2, CH3).
 */
void GeneralTimer::timerInterruptHandler() {
    uint8_t active = callbacks_.active_callbacks;

    // Fast path for update interrupt
    if (__builtin_expect((active & 0x01U), 0)) {
        if (timer_.get_interrupt_flag(timer::Interrupt_Flags::INTR_FLAG_UP)) {
            timer_.clear_interrupt_flag(timer::Interrupt_Flags::INTR_FLAG_UP);
            callbacks_.up_callback();
        }
    }

    // Process channel interrupts only if any are active
    if (__builtin_expect((active & 0x1EU), 0)) {
        active >>= 1U;  // Remove UP bit

        // Unroll loop for Cortex-M efficiency
        if (active & 0x01U) {
            this->processChannelInterrupt<timer::Timer_Channel::CH0>();
        }
        if (active & 0x02U) {
            this->processChannelInterrupt<timer::Timer_Channel::CH1>();
        }
        if (active & 0x04U) {
            this->processChannelInterrupt<timer::Timer_Channel::CH2>();
        }
        if (active & 0x08U) {
            this->processChannelInterrupt<timer::Timer_Channel::CH3>();
        }
    }
}


// Interrupt handlers
extern "C" {

    void TIMER0_UP_IRQHandler(void) {
        auto& instance = GeneralTimer::get_instance(timer::TIMER_Base::TIMER0_BASE);
        instance.timerInterruptHandler();
    }
    void TIMER0_Channel_IRQHandler(void) {
        auto& instance = GeneralTimer::get_instance(timer::TIMER_Base::TIMER0_BASE);
        instance.timerInterruptHandler();
    }
    void TIMER1_IRQHandler(void) {
        auto& instance = GeneralTimer::get_instance(timer::TIMER_Base::TIMER1_BASE);
        instance.timerInterruptHandler();
    }
    void TIMER2_IRQHandler(void) {
        auto& instance = GeneralTimer::get_instance(timer::TIMER_Base::TIMER2_BASE);
        instance.timerInterruptHandler();
    }
    void TIMER3_IRQHandler(void) {
        auto& instance = GeneralTimer::get_instance(timer::TIMER_Base::TIMER3_BASE);
        instance.timerInterruptHandler();
    }
    void TIMER4_IRQHandler(void) {
        auto& instance = GeneralTimer::get_instance(timer::TIMER_Base::TIMER4_BASE);
        instance.timerInterruptHandler();
    }
    void TIMER5_IRQHandler(void) {
        auto& instance = GeneralTimer::get_instance(timer::TIMER_Base::TIMER5_BASE);
        instance.timerInterruptHandler();
    }
    void TIMER6_IRQHandler(void) {
        auto& instance = GeneralTimer::get_instance(timer::TIMER_Base::TIMER6_BASE);
        instance.timerInterruptHandler();
    }
    void TIMER7_UP_IRQHandler(void) {
        auto& instance = GeneralTimer::get_instance(timer::TIMER_Base::TIMER7_BASE);
        instance.timerInterruptHandler();
    }
    void TIMER7_Channel_IRQHandler(void) {
        auto& instance = GeneralTimer::get_instance(timer::TIMER_Base::TIMER7_BASE);
        instance.timerInterruptHandler();
    }

} // extern "C"
