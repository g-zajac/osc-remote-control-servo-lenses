#define SERIAL_DEBUGING     // comment it out to disable serial debuging, for production i.e.

#include <Arduino.h>

void setup() {
  #ifdef SERIAL_DEBUGING
    Serial.begin(9600);
  #endif
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
}


void loop() {
  #ifdef SERIAL_DEBUGING
    Serial.println("serial test");
  #endif
  delay(1000);
}
