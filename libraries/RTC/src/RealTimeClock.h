#pragma once

#include "Arduino.h"

#ifndef RTC_IRQ_PRIORITY
    #define RTC_IRQ_PRIORITY      2
#endif
#ifndef RTC_IRQ_SUBPRIORITY
    #define RTC_IRQ_SUBPRIORITY   0
#endif

enum class Alarm_Format : uint8_t {
    ALARM_S,
    ALARM_M,
    ALARM_H,
};

enum class Interrupt_Type : uint8_t {
    INTR_SECOND,
    INTR_ALARM,
    INTR_OVERFLOW,
};

struct Time_Set {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

class RealTimeClock {
public:
    static RealTimeClock& get_instance();

    using RTCCallback = void (*)(void);

    void init();
    void setTime(const Time_Set& time);
    void getTime(Time_Set& time);
    void setSeconds(uint32_t seconds);
    uint32_t getSeconds();
    // Alarms
    void setTimerAlarm(uint32_t hours, uint32_t minutes, uint32_t seconds);
    void setTimerAlarm(uint32_t offset, Alarm_Format format);
    void setDailyAlarm(uint8_t hour, uint8_t minute, uint8_t second);
    void setCalendarAlarm(uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
    // Timestamp
    uint32_t createTimestamp(const Time_Set& time);
    // Interrupts
    void attachInterrupt(RTCCallback callback, Interrupt_Type type);
    void detachInterrupt(Interrupt_Type type);
    void interruptHandler(Interrupt_Type type);

private:
    RealTimeClock();

    rtc::RTC& rtc_;
    Time_Set time_;
    RTCCallback Callbacks_[3];

    inline uint8_t getMonthLength(uint8_t leapYear, uint8_t month) {
        uint8_t length = 30U;
        if (month == 2U) {       // February
            length = (28U + leapYear);
        } else {
            if (month > 7U) {    // August to december
                month--;
            }
            if (month & 1U) {
                length = 31U;
            }
        }
        return length;
    }

    inline bool isLeapYear(uint16_t year) {
        return (!((year) % 400) || (((year) & 100) && !((year) % 4)));
    }

    inline uint32_t getYearLength(uint16_t year) {
        return (isLeapYear(year) ? 366U : 365U);
    }
};
