// gd32f30x General purpose Timer driver for Arduino Core
// Copyright (c) 2025 B. Mouritsen <bnmguy@gmail.com>
// All rights reserved.

#pragma once

#include <array>

#include "Arduino.h"
#include "CoreHandler.h"
#include "PinOpsMap.hpp"
#include "PinOps.hpp"

enum class TimerFormat : uint8_t { TICK, US, HERTZ };
enum class CCFormat : uint8_t { B1 = 1, B2, B3, B4, B5, B6, B7, B8, B9, B10, B11, B12, B13, B14, B15, B16, TICK = 0x80, US, HERTZ, PERCENT, INVALID };
enum class InputOutputMode : uint8_t { TIMING, OUTPUT, CLEAR, TOGGLE, FORCE_LOW, FORCE_HIGH, PWM0, PWM1, RISING, FALLING, COMPARE, INVALID };

#define TIMER_CHANNELS  4
#define TIMER_COUNT     8

struct timer_to_irq {
    uint8_t timer_index;
    IRQn_Type irq_type;
};

class GeneralTimer {
public:
    static auto get_instance(timer::TIMER_Base Base) -> GeneralTimer&;

    using TimerCallback = void (*)();

    struct CallbackState {
        uint8_t active_callbacks;
        TimerCallback up_callback;
        std::array<TimerCallback, TIMER_CHANNELS> channel_callbacks;
    };

    void begin(timer::TIMER_Config config);
    void start();
    void stop();
    void refresh();

    // Channel
    void startTimerChannel(timer::Timer_Channel channel);
    void startTimerChannel(uint8_t channel) {
        startTimerChannel(convertToChannel(channel));
    }
    void stopTimerChannel(timer::Timer_Channel channel);
    void stopTimerChannel(uint8_t channel) {
        stopTimerChannel(convertToChannel(channel));
    }
    void setChannelMode(timer::Timer_Channel channel, InputOutputMode mode, pin_size_t pin);
    inline void setChannelMode(uint8_t channel, InputOutputMode mode, pin_size_t pin) {
        setChannelMode(convertToChannel(channel), mode, pin);
    }
    auto getChannelMode(timer::Timer_Channel channel) -> InputOutputMode;
    inline auto getChannelMode(uint8_t channel) -> InputOutputMode {
        return getChannelMode(convertToChannel(channel));
    }

    // Prescaler
    void setPrescaler(uint16_t prescaler);
    auto getPrescaler() -> uint16_t;

    // Rollover
    void setRolloverValue(uint32_t value, TimerFormat format = TimerFormat::TICK);
    auto getRolloverValue(TimerFormat format = TimerFormat::TICK) -> uint32_t;

    // Counter
    void setCounter(uint16_t count, TimerFormat format = TimerFormat::TICK);
    auto getCounter(TimerFormat format = TimerFormat::TICK) -> uint32_t;

    // Auto-reload
    void setAutoReloadEnable(bool enable);

    // Capture/compare
    void setCaptureCompare(timer::Timer_Channel channel, uint32_t value, CCFormat format = CCFormat::TICK);
    inline void setCaptureCompare(uint8_t channel, uint32_t value, CCFormat format = CCFormat::TICK) {
        setCaptureCompare(convertToChannel(channel), value, format);
    }
    auto getCaptureCompare(timer::Timer_Channel channel, CCFormat format = CCFormat::TICK) -> uint32_t;
    inline auto getCaptureCompare(uint8_t channel, CCFormat format = CCFormat::TICK) -> uint32_t {
        return getCaptureCompare(convertToChannel(channel), format);
    }

    // PWM
    void setPWM(timer::Timer_Channel channel, pin_size_t pin, uint32_t frequency,
                uint32_t dutycycle, TimerCallback UpCallback = nullptr,
                TimerCallback ChCallback = nullptr);
    inline void setPWM(uint8_t channel, pin_size_t pin, uint32_t frequency,
                       uint32_t dutycycle, TimerCallback UpCallback = nullptr,
                       TimerCallback ChCallback = nullptr) {
        setPWM(convertToChannel(channel), pin, frequency, dutycycle, UpCallback, ChCallback);
    }

    // Interrupts
    void setInterruptPriority(uint8_t preemptPriority, uint8_t subPriority);
    void attachInterrupt(TimerCallback callback);
    void attachInterrupt(TimerCallback callback, timer::Timer_Channel channel);
    inline void attachInterrupt(TimerCallback callback, uint8_t channel) {
        attachInterrupt(callback, convertToChannel(channel));
    }
    void detachInterrupt();
    void detachInterrupt(timer::Timer_Channel channel);
    inline void detachInterrupt(uint8_t channel) {
        detachInterrupt(convertToChannel(channel));
    }
    auto hasInterrupt() -> bool;
    auto hasInterrupt(timer::Timer_Channel channel) -> bool;
    inline auto hasInterrupt(uint8_t channel) -> bool {
        return hasInterrupt(convertToChannel(channel));
    }

    void timerInterruptHandler();

    inline void UpCallback() {
        if (callbacks_.active_callbacks & 0x01) {
            callbacks_.up_callback();
        }
    }

    inline void ChCallback(timer::Timer_Channel channel) {
        uint8_t ch = static_cast<uint8_t>(channel);
        if (callbacks_.active_callbacks & (1 << (ch + 1))) {
            callbacks_.channel_callbacks[ch]();
        }
    }

    inline void ChCallback(uint8_t channel) {
        ChCallback(convertToChannel(channel));
    }

    inline auto getTimerUpIRQ() -> IRQn_Type { return timerToUpIrq(); }
    inline auto getTimerChIRQ() -> IRQn_Type { return timerToChIrq(); }

    auto getTimerClockFrequency() -> uint32_t;

protected:
    explicit GeneralTimer(timer::TIMER_Base Base);

    timer::TIMER_Base base_;
    timer::TIMER& timer_;
    timer::TIMER_Config config_;
    timer::TIMER_Input_Capture capture_config_;
    timer::TIMER_Output_Compare compare_config_;
    CallbackState callbacks_;
    uint8_t preemptPriority_;
    uint8_t subPriority_;
    std::array<InputOutputMode, TIMER_CHANNELS> channel_modes_;
    std::array<bool, TIMER_CHANNELS> companionChannel;

    static auto get_timer_instance(timer::TIMER_Base Base) -> timer::TIMER&;

    inline auto getChannelFromPin(pin_size_t pin) -> timer::Timer_Channel {
        auto instanceBase = getPinOpsPeripheralBase<TIMERPinOps, timer::TIMER_Base>(TIMER_PinOps, pin);
        if (instanceBase == timer::TIMER_Base::INVALID) {
            return timer::Timer_Channel::INVALID;
        }

        auto packedPinOps = getPackedPinOps(TIMER_PinOps, instanceBase, pin);
        if (packedPinOps == invalidValue) {
            return timer::Timer_Channel::INVALID;
        }
        auto channel_num = getPackedPinChannel(packedPinOps);
        return static_cast<timer::Timer_Channel>(channel_num);
    }

    inline auto getCompanionChannelFromPin(pin_size_t pin) -> timer::Timer_Channel {
        auto instanceBase = getPinOpsPeripheralBase<TIMERPinOps, timer::TIMER_Base>(TIMER_PinOps, pin);
        if (instanceBase == timer::TIMER_Base::INVALID) {
            return timer::Timer_Channel::INVALID;
        }

        auto packedPinOps = getPackedPinOps(TIMER_PinOps, instanceBase, pin);
        if (packedPinOps == invalidValue) {
            return timer::Timer_Channel::INVALID;
        }
        auto channel_num = getPackedPinChOn(packedPinOps);
        return static_cast<timer::Timer_Channel>(channel_num);
    }

    inline auto getCompanionChannel(timer::Timer_Channel channel) -> timer::Timer_Channel {
        switch (channel) {
            case timer::Timer_Channel::CH0: return timer::Timer_Channel::CH1;
            case timer::Timer_Channel::CH1: return timer::Timer_Channel::CH0;
            case timer::Timer_Channel::CH2: return timer::Timer_Channel::CH3;
            case timer::Timer_Channel::CH3: return timer::Timer_Channel::CH2;
            case timer::Timer_Channel::INVALID:
            default: return timer::Timer_Channel::INVALID;
        }
    }

    inline auto convertToFlag(timer::Timer_Channel channel) -> timer::Status_Flags {
        switch (channel) {
            case timer::Timer_Channel::CH0: return timer::Status_Flags::FLAG_CH0;
            case timer::Timer_Channel::CH1: return timer::Status_Flags::FLAG_CH1;
            case timer::Timer_Channel::CH2: return timer::Status_Flags::FLAG_CH2;
            case timer::Timer_Channel::CH3: return timer::Status_Flags::FLAG_CH3;
            case timer::Timer_Channel::INVALID:
            default: return timer::Status_Flags::INVALID;
        }
    }

    inline auto convertToInterruptFlag(timer::Timer_Channel channel) -> timer::Interrupt_Flags {
        switch (channel) {
            case timer::Timer_Channel::CH0: return timer::Interrupt_Flags::INTR_FLAG_CH0;
            case timer::Timer_Channel::CH1: return timer::Interrupt_Flags::INTR_FLAG_CH1;
            case timer::Timer_Channel::CH2: return timer::Interrupt_Flags::INTR_FLAG_CH2;
            case timer::Timer_Channel::CH3: return timer::Interrupt_Flags::INTR_FLAG_CH3;
            case timer::Timer_Channel::INVALID:
            default: return timer::Interrupt_Flags::INVALID;
        }
    }

    inline auto convertToInterrupt(timer::Timer_Channel channel) -> timer::Interrupt_Type {
        switch (channel) {
            case timer::Timer_Channel::CH0: return timer::Interrupt_Type::INTR_CH0IE;
            case timer::Timer_Channel::CH1: return timer::Interrupt_Type::INTR_CH1IE;
            case timer::Timer_Channel::CH2: return timer::Interrupt_Type::INTR_CH2IE;
            case timer::Timer_Channel::CH3: return timer::Interrupt_Type::INTR_CH3IE;
            case timer::Timer_Channel::INVALID:
            default: return timer::Interrupt_Type::INVALID;
        }
    }

    inline timer::Timer_Channel convertToChannel(uint8_t channel) { return static_cast<timer::Timer_Channel>(channel); }

private:
    inline static constexpr std::array<timer_to_irq, TIMER_COUNT> timer_up_irq {{
        {0U, TIMER0_UP_IRQn}, {1U, TIMER1_IRQn}, {2U, TIMER2_IRQn}, {3U, TIMER3_IRQn},
        {4U, TIMER4_IRQn}, {5U, TIMER5_IRQn}, {6U, TIMER6_IRQn}, {7U, TIMER7_UP_IRQn}
    }};

    inline static constexpr std::array<timer_to_irq, TIMER_COUNT> timer_ch_irq {{
        {0U, TIMER0_Channel_IRQn}, {1U, TIMER1_IRQn}, {2U, TIMER2_IRQn}, {3U, TIMER3_IRQn},
        {4U, TIMER4_IRQn}, {5U, TIMER5_IRQn}, {6U, TIMER6_IRQn}, {7U, TIMER7_Channel_IRQn}
    }};

    inline auto timerToUpIrq() -> IRQn_Type {
        uint8_t timerIndex = static_cast<uint8_t>(base_);
        for (const auto& index : timer_up_irq) {
            if (index.timer_index == timerIndex) {
                return index.irq_type;
            }
        }
        // Return invalid
        return INVALID_IRQ;
    }

    inline auto timerToChIrq() -> IRQn_Type {
        uint8_t timerIndex = static_cast<uint8_t>(base_);
        for (const auto& index : timer_ch_irq) {
            if (index.timer_index == timerIndex) {
                return index.irq_type;
            }
        }
        // Return invalid
        return INVALID_IRQ;
    }

    template<timer::Timer_Channel channel>
    inline void processChannelInterrupt() {
        auto flag = convertToFlag(channel);
        if (timer_.get_flag(flag)) {
            timer_.clear_interrupt_flag(convertToInterruptFlag(channel));
            callbacks_.channel_callbacks[static_cast<size_t>(channel)]();
        }
    }
};
