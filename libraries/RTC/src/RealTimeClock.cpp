
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

/**
 * @brief Initializes the RTC peripheral.
 *
 * This function is called once during construction and need not be called by
 * the user.
 *
 * It sets up the RTC peripheral to use the Low Speed External Crystal (LSE)
 * as the clock source, and clears any pending interrupts. It also sets the
 * RTC alarm interrupt priority and enables the interrupt.
 */
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
}

/**
 * @brief Sets the current time using the provided timestamp.
 *
 * This function is thread-safe as it waits for any pending writes to the RTC
 * to complete before modifying the counter value.
 *
 * @param[in] time The timestamp to set.
 */
void RealTimeClock::setTime(const Time_Set& time) {
    const uint32_t counter = createTimestamp(time) - SecondsPerHour * 8;
    rtc_.lwoff_wait();
    rtc_.set_counter(counter);
    rtc_.lwoff_wait();
}

/**
 * @brief Retrieves the current time from the RTC.
 *
 * This function populates the provided Time_Set structure with the current
 * year, month, day, hour, minute, and second values obtained from the RTC
 * counter. The time is adjusted for the timezone offset, and calculations
 * are performed to determine the correct date and time components.
 *
 * @param[out] time A Time_Set structure that will be updated with the current
 *                  date and time information from the RTC.
 */

void RealTimeClock::getTime(Time_Set& time) {
    // Base date initialization
    time = time_;

    const uint32_t timestamp = rtc_.get_counter() + SecondsPerHour * 8;
    const uint32_t daySeconds = timestamp % SecondsPerDay;

    // Calculate time components
    time.hour = daySeconds / SecondsPerHour;
    time.minute = (daySeconds % SecondsPerHour) / SecondsPerMinute;
    time.second = daySeconds % SecondsPerMinute;

    // Calculate date components
    uint32_t remainingDays = timestamp / SecondsPerDay;

    while (remainingDays >= getYearLength(time.year)) {
        remainingDays -= getYearLength(time.year);
        time.year++;
    }

    while (remainingDays >= getMonthLength(isLeapYear(time.year), time.month)) {
        remainingDays -= getMonthLength(isLeapYear(time.year), time.month);
        time.month++;
    }
    time.day += remainingDays;
}

/**
 * @brief Sets the RTC counter to the specified number of seconds since midnight.
 *
 * This function is useful for setting the RTC to a specific time of day, or for
 * setting the RTC to a specific time zone offset. The function is synchronized
 * with the RTC to ensure that the counter is not modified while the function is
 * executing.
 *
 * @param[in] seconds The number of seconds since midnight to set as the current
 *                    time.
 */
void RealTimeClock::setSeconds(uint32_t seconds) {
    rtc_.lwoff_wait();
    rtc_.set_counter(seconds);
    rtc_.lwoff_wait();
}

/**
 * @brief Retrieves the current time in seconds since midnight.
 *
 * This function returns the current value of the RTC counter, representing
 * the number of seconds elapsed since midnight. It is useful for obtaining
 * the precise current time for time-sensitive operations.
 *
 * @return The current time in seconds since midnight.
 */

uint32_t RealTimeClock::getSeconds() {
    return rtc_.get_counter();
}

/**
 * @brief Sets the RTC alarm to trigger after the specified hours, minutes, and seconds.
 *
 * This function sets the RTC alarm to trigger after the specified hours, minutes,
 * and seconds has passed. The function is synchronized with the RTC to ensure that
 * the alarm is not modified while the function is executing.
 *
 * @param hours The number of hours before the alarm should trigger.
 * @param minutes The number of minutes before the alarm should trigger.
 * @param seconds The number of seconds before the alarm should trigger.
 */
void RealTimeClock::setTimerAlarm(uint32_t hours, uint32_t minutes, uint32_t seconds) {
    uint32_t totalSeconds = (hours * SecondsPerHour) + 
                           (minutes * SecondsPerMinute) + 
                           seconds;

    rtc_.lwoff_wait();
    rtc_.set_alarm(rtc_.get_counter() + totalSeconds);
    rtc_.lwoff_wait();
}

/**
 * @brief Sets the RTC alarm to trigger after a specified offset.
 *
 * This function sets the RTC alarm to trigger after a specified offset from the
 * current time. The offset can be specified in different formats, as specified
 * by the Alarm_Format enumeration. The possible formats are:
 *   - ALARM_S: seconds
 *   - ALARM_M: minutes
 *   - ALARM_H: hours
 *
 * The function is synchronized with the RTC to ensure that the alarm is not
 * modified while the function is executing.
 *
 * @param offset The offset from the current time, in the specified format.
 * @param format The format of the offset, as specified by the Alarm_Format
 *               enumeration.
 */
void RealTimeClock::setTimerAlarm(uint32_t offset, Alarm_Format format) {
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

/**
 * @brief Sets the RTC alarm to trigger at the specified time every day.
 *
 * This function sets the RTC alarm to trigger at the specified time every day.
 * The time is specified in 24-hour format, with hours, minutes, and seconds
 * parameters. If the specified time is earlier than the current time, the alarm
 * is set for the next day.
 *
 * @param hour The hour component of the time to set the alarm for.
 * @param minute The minute component of the time to set the alarm for.
 * @param second The second component of the time to set the alarm for.
 */
void RealTimeClock::setDailyAlarm(uint8_t hour, uint8_t minute, uint8_t second) {
    Time_Set currentTime;
    getTime(currentTime);

    // Calculate target time in seconds since midnight
    uint32_t targetSeconds = (hour * SecondsPerHour) + 
                            (minute * SecondsPerMinute) + 
                            second;

    // Calculate current time in seconds since midnight
    uint32_t currentSeconds = (currentTime.hour * SecondsPerHour) + 
                             (currentTime.minute * SecondsPerMinute) + 
                             currentTime.second;

    // If target time is earlier today, set for tomorrow
    uint32_t alarmOffset = (targetSeconds <= currentSeconds) 
        ? (SecondsPerDay - currentSeconds) + targetSeconds
        : targetSeconds - currentSeconds;

    rtc_.lwoff_wait();
    rtc_.set_alarm(rtc_.get_counter() + alarmOffset);
    rtc_.lwoff_wait();
}

/**
 * @brief Sets the RTC alarm to trigger at the specified date and time.
 *
 * This function sets the RTC alarm to trigger at the specified date and time.
 * The date and time are specified in their respective components. If the
 * specified date and time is earlier than the current time, the alarm is set
 * for the same date and time of the next year.
 *
 * @param month The month component of the date and time to set the alarm for.
 * @param day The day component of the date and time to set the alarm for.
 * @param hour The hour component of the date and time to set the alarm for.
 * @param minute The minute component of the date and time to set the alarm for.
 * @param second The second component of the date and time to set the alarm for.
 */
void RealTimeClock::setCalendarAlarm(uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
    Time_Set currentTime;
    getTime(currentTime);

    Time_Set targetTime = currentTime;
    targetTime.month = month;
    targetTime.day = day;
    targetTime.hour = hour;
    targetTime.minute = minute;
    targetTime.second = second;

    // If target date is earlier this year, set for next year
    if (month < currentTime.month || 
        (month == currentTime.month && day < currentTime.day)) {
        targetTime.year++;
    }

    uint32_t targetTimestamp = createTimestamp(targetTime);
    uint32_t currentTimestamp = createTimestamp(currentTime);
    uint32_t alarmOffset = targetTimestamp - currentTimestamp;

    rtc_.lwoff_wait();
    rtc_.set_alarm(rtc_.get_counter() + alarmOffset);
    rtc_.lwoff_wait();
}

/**
 * @brief Converts a Time_Set object to a timestamp in seconds since the
 *        epoch (January 1, 1970, 00:00:00 UTC).
 *
 * This function takes a Time_Set object and converts it to a timestamp in
 * seconds since the epoch. The timestamp is calculated by summing the
 * contributions of the year, month, day, hour, minute, and second components.
 *
 * @param[in] time The Time_Set object to convert.
 * @return The timestamp in seconds since the epoch.
 */
uint32_t RealTimeClock::createTimestamp(const Time_Set& time) {
    uint32_t length = 0;

    // Calculkate years contribution
    for (uint16_t year = 1970; year < time.year; ++year) {
        length += isLeapYear(year) ? 366 : 365;
    }

    // Calculate months contribution
    for (uint8_t month = 1; month < time.month; ++month) {
        length += getMonthLength(isLeapYear(time.year), month);
    }

    // Add days and time components
    length += time.day - 1;

    return (length * SecondsPerDay) +
           (time.hour * SecondsPerHour) +
           (time.minute * SecondsPerMinute) +
           time.second;
}

/**
 * @brief Attaches an interrupt callback to a specific interrupt type.
 *
 * This function takes a callback function and an interrupt type and enables
 * the interrupt. The interrupt type can be one of the following:
 *
 * - `Interrupt_Type::INTR_SECOND`: Interrupts every second.
 * - `Interrupt_Type::INTR_ALARM`: Interrupts when the alarm time is reached.
 * - `Interrupt_Type::INTR_OVERFLOW`: Interrupts when the RTC counter overflows.
 *
 * The callback function is called when the interrupt occurs.
 *
 * @param[in] callback The callback function to attach to the interrupt.
 * @param[in] type The type of interrupt to attach the callback to.
 */
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

/**
 * @brief Detaches an interrupt callback from a specific interrupt type.
 *
 * This function takes an interrupt type and detaches the callback function
 * associated with that interrupt type. The interrupt type can be one of the
 * following:
 *
 * - `Interrupt_Type::INTR_SECOND`: Interrupts every second.
 * - `Interrupt_Type::INTR_ALARM`: Interrupts when the alarm time is reached.
 * - `Interrupt_Type::INTR_OVERFLOW`: Interrupts when the RTC counter overflows.
 *
 * @param[in] type The type of interrupt to detach the callback from.
 */
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

/**
 * @brief Interrupt handler function for handling interrupts generated by the
 *        RTC.
 *
 * This function is called by the interrupt handler when an interrupt is
 * generated by the RTC. It determines the type of interrupt and calls the
 * appropriate callback function.
 *
 * @param[in] type The type of interrupt that occurred.
 */
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
