// gd32f30x General purpose Timer driver for Arduino Core
// Copyright (c) 2025 B. Mourit <bnmguy@gmail.com>
// All rights reserved.

#pragma once

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
    static GeneralTimer& get_instance(timer::TIMER_Base Base);

    using TimerCallback = void (*)(void);

    struct CallbackState {
        uint8_t active_callbacks;
        TimerCallback up_callback;
        TimerCallback channel_callbacks[TIMER_CHANNELS];
    };

    void begin(timer::TIMER_Config config);
    void start();
    void stop();
    void refresh();
    void startTimerChannel(timer::Timer_Channel channel);
    void startTimerChannel(uint8_t channel) {
        startTimerChannel(convertToChannel(channel));
    }
    void stopTimerChannel(timer::Timer_Channel channel);
    void stopTimerChannel(uint8_t channel) {
        stopTimerChannel(convertToChannel(channel));
    }
    void setPrescaler(uint16_t prescaler);
    uint16_t getPrescaler();
    // Rollover (overflow/auto reload)
    void setRolloverValue(uint32_t value, TimerFormat format = TimerFormat::TICK);
    uint32_t getRolloverValue(TimerFormat format = TimerFormat::TICK);
    void setCounter(uint16_t count, TimerFormat format = TimerFormat::TICK);
    uint32_t getCounter(TimerFormat format = TimerFormat::TICK);
    void setChannelMode(timer::Timer_Channel channel, InputOutputMode mode, pin_size_t pin);
    inline void setChannelMode(uint8_t channel, InputOutputMode mode, pin_size_t pin) {
        setChannelMode(convertToChannel(channel), mode, pin);
    }
    InputOutputMode getChannelMode(timer::Timer_Channel channel);
    inline InputOutputMode getChannelMode(uint8_t channel) {
        return getChannelMode(convertToChannel(channel));
    }
    void setAutoReloadEnable(bool enable);
    void setCaptureCompare(timer::Timer_Channel channel, uint32_t value, CCFormat format = CCFormat::TICK);
    inline void setCaptureCompare(uint8_t channel, uint32_t value, CCFormat format = CCFormat::TICK) {
        setCaptureCompare(convertToChannel(channel), value, format);
    }
    uint32_t getCaptureCompare(timer::Timer_Channel channel, CCFormat format = CCFormat::TICK);
    inline uint32_t getCaptureCompare(uint8_t channel, CCFormat format = CCFormat::TICK) {
        return getCaptureCompare(convertToChannel(channel), format);
    }
    void setPWM(timer::Timer_Channel channel, pin_size_t pin, uint32_t frequency,
                uint32_t dutycycle, TimerCallback UPCallback = nullptr,
                TimerCallback CCCallback = nullptr);
    inline void setPWM(uint8_t channel, pin_size_t pin, uint32_t frequency,
                uint32_t dutycycle, TimerCallback UPCallback = nullptr,
                TimerCallback CCCallback = nullptr) {
        setPWM(convertToChannel(channel), pin, frequency, dutycycle, UPCallback, CCCallback);
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
    bool hasInterrupt();
    bool hasInterrupt(timer::Timer_Channel channel);
    inline bool hasInterrupt(uint8_t channel) {
        return hasInterrupt(convertToChannel(channel));
    }

    void timerInterruptHandler();

    void UPCallback() {
        if (callbacks_.active_callbacks & 0x01) {
            callbacks_.up_callback();
        }
    }
    void CCCallback(timer::Timer_Channel channel) {
        uint8_t ch = static_cast<size_t>(channel);
        if (callbacks_.active_callbacks & (1 << (ch + 1))) {
            callbacks_.channel_callbacks[ch]();
        }
    }
    void CCCallback(uint8_t channel) {
        CCCallback(convertToChannel(channel));
    }

    uint32_t getTimerClockFrequency();

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
    InputOutputMode channel_modes_[TIMER_CHANNELS];
    bool companionChannel[TIMER_CHANNELS];

    static timer::TIMER& get_timer_instance(timer::TIMER_Base Base);

    inline IRQn_Type getTimerUpIRQ();
    inline IRQn_Type getTimerCCIRQ();
    inline timer::Timer_Channel getChannelFromPin(pin_size_t pin);
    inline timer::Timer_Channel getCompanionChannelFromPin(pin_size_t pin);
    inline timer::Timer_Channel getCompanionChannel(timer::Timer_Channel channel);
    inline timer::Status_Flags convertToFlag(timer::Timer_Channel channel);
    inline timer::Interrupt_Flags convertToInterruptFlag(timer::Timer_Channel channel);
    inline timer::Interrupt_Type convertToInterrupt(timer::Timer_Channel channel);
    inline timer::Timer_Channel convertToChannel(uint8_t channel) {
        return static_cast<timer::Timer_Channel>(channel);
    }

private:
    static std::array<timer_to_irq, TIMER_COUNT> timer_up_irq;
    static std::array<timer_to_irq, TIMER_COUNT> timer_ch_irq;

    inline IRQn_Type timerToUpIrq(uint8_t timerIndex) {
        for (const auto& index : timer_up_irq) {
            if (index.timer_index == timerIndex) {
                return index.irq_type;
            }
        }
        // Return invalid
        return INVALID_IRQ;
    }

    inline IRQn_Type timerToChIrq(uint8_t timerIndex) {
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
        timer::Status_Flags flag = convertToFlag(channel);
        if (timer_.get_flag(flag)) {
            timer_.clear_interrupt_flag(convertToInterruptFlag(channel));
            callbacks_.channel_callbacks[static_cast<size_t>(channel)]();
        }
    }
};
