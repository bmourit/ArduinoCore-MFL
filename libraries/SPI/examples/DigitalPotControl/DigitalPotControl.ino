/*
  Digital Pot Control

  This example controls an Analog Devices AD5206 digital potentiometer.
  The AD5206 has 6 potentiometer channels. Each channel's pins are labeled
  A - connect this to voltage
  W - this is the pot's wiper, which changes when you set it
  B - connect this to ground.

 The AD5206 is SPI-compatible,and to command it, you send two bytes,
 one with the channel number (0 - 5) and one with the resistance value for the
 channel (0 - 255).

 The circuit:
  * All A pins  of AD5206 connected to +5V
  * All B pins of AD5206 connected to ground
  * An LED and a 220-ohm resisor in series connected from each W pin to ground
  * CS - to digital pin 10  (SS pin)
  * SDI - to digital pin 11 (MOSI pin)
  * CLK - to digital pin 13 (SCK pin)

 created 10 Aug 2010
 by Tom Igoe

 Thanks to Heather Dewey-Hagborg for the original tutorial, 2005

 Modified and adapted for MFL January 2025
 by B. Mouritsen
*/

#include <Arduino.h>
#include <SPI.h>

// Set pin 10 as the slave select for the digital pot
const pin_size_t sselPin = 10;

/**
 * @brief Initialization function
 *
 * This function is called once, when the program starts. It does two things:
 * 1. Sets the slave select pin as an output
 * 2. Initializes the SPI library
 */
void setup() {
    // Set the slaveSelectPin mode to output
    pinMode(sselPin, OUTPUT);
    // Initialize SPI
    SPI.begin();
}

void loop() {
    // Go through the six channels of the digital pot
    for (int channel = 0; channel < 6; channel++) {
        // Change the resistance on this channel from min to max
        for (int level = 0; level < 255; level++) {
            digitalPotWrite(channel, level);
            delay(10);
        }
        // Wait 1 second at the top
        delay(100);
        // Change the resistance on this channel from max to min
        for (int level = 0; level < 255; level++) {
            digitalPotWrite(channel, 255 - level);
            delay(10);
        }
    }
}

void digitalPotWrite(int address, int value) {
    // Take the SS pin low to select the chip
    digitalWrite(sselPin, LOW);
    // Send in the address and value via SPI
    SPI.transfer(address);
    SPI.transfer(value);
    // Take the SS pin high to deselect the chip
    digitalWrite(sselPin, HIGH);
}
