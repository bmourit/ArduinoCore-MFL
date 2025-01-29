#include <Arduino.h>
#include <UsartSerial.hpp>
#include <RealTimeClock.h>

RealTimeClock& RTC_I = RealTimeClock::get_instance();

// set current time: year, month, date, hour, minute, second
Time_Set currentTime = { 2025, 1, 6, 9, 0, 0 }; 

void setup() {
    pinMode(LED2, OUTPUT);
    Serial.begin(115200);
    RTC_I.setTime(&currentTime);
    RTC_I.attachInterrupt(secondCallback, Interrupt_Type::INTR_SECOND);
}

// the loop function runs over and over again forever
void loop() {
    delay(1000);
    //print current time every second
    Serial.print(currentTime.year, DEC);
    Serial.print(' ');
    Serial.print(currentTime.month, DEC);
    Serial.print(' ');
    Serial.print(currentTime.day, DEC);
    Serial.print(' ');
    Serial.print(currentTime.hour, DEC);
    Serial.print(' ');
    Serial.print(currentTime.minute, DEC);
    Serial.print(' ');
    Serial.println(currentTime.second, DEC);
}

// second interrupt callback function
void secondCallback(void) {
    RTC_I.getTime(&currentTime);
    digitalToggle(LED2);
}
