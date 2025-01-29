/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>
 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
 Modified and adapted for MFL Jan 2025,
 by B. Mouritsen
*/

#include <Arduino.h>
#include <Servo.h>

// NOTE: Change these to match your board
#define SERVO_PIN   9
#define POT_PIN     1

// create servo object to control a servo
Servo myservo;

uint16_t potValue;
uint16_t servoAngle;

void setup() {
    // attaches the servo on pin 9 to the servo object
    myservo.attach(SERVO_PIN);
}

void loop() {
    // reads the value of the potentiometer (value between 0 and 1023)
    potValue = analogRead(POT_PIN);
    // scale it to use it with the servo (value between 0 and 180)
    servoAngle = map(potValue, 0, 1023, 0, 180);
    // sets the servo position according to the scaled value
    myservo.write(servoAngle);
    // waits for the servo to get there
    delay(15);
}
