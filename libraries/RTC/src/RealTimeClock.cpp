
#include "Arduino.h"
#include "RealTimeClock.h"

constexpr uint8_t irqPriority = RTC_IRQ_PRIORITY;
constexpr uint8_t irqSubPriority = RTC_IRQ_SUBPRIORITY;

constexpr const uint32_t SecondsPerMinute = 60U;
constexpr const uint32_t SecondsPerHour = 3600U;
constexpr const uint32_t SecondsPerDay = 86400U;

RealTimeClock& RealTimeClock::get_instance() {
    static RealTimeClock instance;
    return instance;
}

RealTimeClock::RealTimeClock() :
       rtc_(rtc::RTC::get_instance()),
       time_(),
       Callbacks_{nullptr, nullptr, nullptr}
{
    RCU_I.set_rtc_source(rcu::RTC_Source::RTCSRC_LXTAL);
    time_ = { 1970U, 1U, 1U, 0U, 0U, 0U, };
    init();
}

void RealTimeClock::init() {
#ifdef KILL_RTC_BACKUP_DOMAIN_ON_RESTART
    backup_domain_kill();
#endif

    // Wait for sync
    rtc_.sync_register_wait();

    // Clear flags
    rtc_.clear_flag(rtc::Status_Flags::FLAG_OVIF);
    rtc_.clear_flag(rtc::Status_Flags::FLAG_ALRMIF);
    rtc_.clear_flag(rtc::Status_Flags::FLAG_SCIF);

    // Wait for write op
    rtc_.lwoff_wait();

    uint32_t priority_group = NVIC_GetPriorityGrouping();
    NVIC_SetPriority(RTC_Alarm_IRQn, NVIC_EncodePriority(priority_group, irqPriority, irqSubPriority));
    NVIC_EnableIRQ(RTC_Alarm_IRQn);

    //backup_domain_enable();
}

void RealTimeClock::setTime(Time_Set* time) {
    uint32_t counter = createTimestamp(time) - SecondsPerHour * 8;
    rtc_.lwoff_wait();
    rtc_.set_counter(counter);
    rtc_.lwoff_wait();
}

void RealTimeClock::getTime(Time_Set *time) {
    time->year = time_.year;
    time->month = time_.month;
    time->day = time_.day;

    uint32_t timestamp = rtc_.get_counter() + SecondsPerHour * 8;
    uint32_t day = timestamp % SecondsPerDay;

    time->hour = day / 3600;
    time->minute = (day % 3600) / 60;
    time->second = day % 60;

    uint32_t length = timestamp / SecondsPerDay;

    while (length >= getYearLength(time->year)) {
        length -= getYearLength(time->year);
        time->year++;
    }

    while (length >= getMonthLength(isLeapYear(time->year), time->month)) {
        length -= getMonthLength(isLeapYear(time->year), time->month);
        time->month++;
    }
    time->day += length;
}

void RealTimeClock::setSeconds(uint32_t seconds) {
    rtc_.lwoff_wait();
    rtc_.set_counter(seconds);
    rtc_.lwoff_wait();
}

uint32_t RealTimeClock::getSeconds() {
    return rtc_.get_counter();
}

void RealTimeClock::setAlarm(uint32_t offset, Alarm_Format format) {
    uint32_t seconds = 0;

    switch (format) {
    case Alarm_Format::ALARM_S:
        seconds = offset;
        break;
    case Alarm_Format::ALARM_M:
        seconds = offset * SecondsPerMinute;
        break;
    case Alarm_Format::ALARM_H:
        seconds = offset * SecondsPerHour;
        break;
    default:
        break;
    }

    rtc_.lwoff_wait();
    rtc_.set_alarm(rtc_.get_counter() + seconds);
    rtc_.lwoff_wait();
}

uint32_t RealTimeClock::createTimestamp(Time_Set* time) {
    uint16_t year = time->year;
    uint8_t month = time->month;
    uint8_t day = time->day;
    uint32_t length = 0;

    while (--year != 1969) {
        if (isLeapYear(year)) {
            length += 366;
        } else {
            length += 365;
        }
    }

    while (--month != 0) {
        length += getMonthLength(isLeapYear(time->year), month);
    }

    length = length + day - 1;
    uint32_t timestamp = length * SecondsPerDay + (time->hour * 3600 + time->minute * 60 + time->second);

    return timestamp;
}

void RealTimeClock::attachInterrupt(RTCCallback callback, Interrupt_Type type) {
    rtc::Interrupt_Type interrupt;

    switch (type) {
    case Interrupt_Type::INTR_SECOND:
        this->Callbacks_[0] = callback;
        interrupt = rtc::Interrupt_Type::INTR_SCIE;
        break;
    case Interrupt_Type::INTR_ALARM:
        this->Callbacks_[1] = callback;
        interrupt = rtc::Interrupt_Type::INTR_ALRMIE;
        exti::EXTI& EXTI_I = exti::EXTI::get_instance();
        EXTI_I.init(exti::EXTI_Line::EXTI17, exti::EXTI_Mode::EXTI_INTERRUPT, exti::EXTI_Trigger::TRIG_RISING);
        break;
    case Interrupt_Type::INTR_OVERFLOW:
        this->Callbacks_[2] = callback;
        interrupt = rtc::Interrupt_Type::INTR_OVIE;
        break;
    default:
        break;
    }
    rtc_.set_interrupt_enable(interrupt, true);
}

void RealTimeClock::detachInterrupt(Interrupt_Type type) {
    rtc::Interrupt_Type interrupt;

    switch (type) {
    case Interrupt_Type::INTR_SECOND:
        interrupt = rtc::Interrupt_Type::INTR_SCIE;
        this->Callbacks_[0] = nullptr;
        break;
    case Interrupt_Type::INTR_ALARM:
        interrupt = rtc::Interrupt_Type::INTR_ALRMIE;
        this->Callbacks_[1] = nullptr;
        break;
    case Interrupt_Type::INTR_OVERFLOW:
        interrupt = rtc::Interrupt_Type::INTR_OVIE;
        this->Callbacks_[2] = nullptr;
        break;
    default:
        break;
    }

    rtc_.set_interrupt_enable(interrupt, false);
}

void RealTimeClock::interruptHandler(Interrupt_Type type) {
    switch (type) {
    case Interrupt_Type::INTR_SECOND:
        if (this->Callbacks_[0] != nullptr) {
            this->Callbacks_[0]();
        }
        break;
    case Interrupt_Type::INTR_ALARM:
        if (this->Callbacks_[1] != nullptr) {
            this->Callbacks_[1]();
        }
        break;
    case Interrupt_Type::INTR_OVERFLOW:
        if (this->Callbacks_[2] != nullptr) {
            this->Callbacks_[2]();
        }
        break;
    default:
        break;
    }
}


extern "C" {

    void RTC_IRQHandler(void) {
        RealTimeClock& RTC_Handle = RealTimeClock::get_instance();
        auto& rtcInstance = rtc::RTC::get_instance();
        if (rtcInstance.get_flag(rtc::Status_Flags::FLAG_SCIF)) {
            rtcInstance.clear_flag(rtc::Status_Flags::FLAG_SCIF);
            RTC_Handle.interruptHandler(Interrupt_Type::INTR_SECOND);
        }

        if (rtcInstance.get_flag(rtc::Status_Flags::FLAG_OVIF)) {
            rtcInstance.clear_flag(rtc::Status_Flags::FLAG_OVIF);
            RTC_Handle.interruptHandler(Interrupt_Type::INTR_OVERFLOW);
        }
    }

    void RTC_Alarm_IRQHandler(void) {
        RealTimeClock& RTC_Handle = RealTimeClock::get_instance();
        auto& rtcInstance = rtc::RTC::get_instance();
        if (rtcInstance.get_flag(rtc::Status_Flags::FLAG_ALRMIF)) {
            rtcInstance.clear_flag(rtc::Status_Flags::FLAG_ALRMIF);
            exti::EXTI::get_instance().clear_flag(exti::Status_Flags::FLAG_EXTI17);
            RTC_Handle.interruptHandler(Interrupt_Type::INTR_ALARM);
        }
    }

} // extern "C"
