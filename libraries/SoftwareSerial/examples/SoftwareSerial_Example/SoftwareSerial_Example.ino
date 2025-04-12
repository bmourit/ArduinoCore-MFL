#include <SoftwareSerial.h>

SoftwareSerial mySerial(PA10, PA9); // RX(10), TX(9)

void setup() {
    mySerial.begin(115200);
    mySerial.print("hello arduino");
    mySerial.print("The startup code appears to work");
}

// the loop function runs over and over again forever
void loop() {

}
