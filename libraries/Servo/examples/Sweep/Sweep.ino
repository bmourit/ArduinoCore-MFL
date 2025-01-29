/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.
 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
 Modified and adapted for MFL January 2025
 by B. Mouritsen
*/

#include <Arduino.h>
#include <Servo.h>

// NOTE: Change this to match your board
#define SERVO_PIN   9

Servo myservo;

// Variable to store the servo position
int pos = 0;

void setup() {
    // Attaches the servo on pin 9 to the servo object
    myservo.attach(SERVO_PIN);
}

void loop() {
    for(pos = 0; pos <= 180; pos += 1) {  // goes from 0 degrees to 180 degrees
        myservo.write(pos);               // tell servo to go to position in variable 'pos'
        delay(15);                        // waits 15ms for the servo to reach the position
    }
    for(pos = 180; pos >= 0; pos -= 1) {  // goes from 180 degrees to 0 degrees
        myservo.write(pos);               // tell servo to go to position in variable 'pos'
        delay(15);                        // waits 15ms for the servo to reach the position
    }
}
