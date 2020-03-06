#define SERIAL_DEBUGING     // comment it out to disable serial debuging, for production i.e.

#include <Arduino.h>

// PCA9685 16-channel PWM servo driver
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// default address 0x40, default board setting
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*
MG996R
dead band: 0.050ms
period 10ms / 50Hz
1ms - 2ms
*/

#define MIN_PULSE_WIDTH  550 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define MAX_PULSE_WIDTH  2540 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

int angleToPulse(int angle){
   int pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);   // map angle of 0 to 180 to Servo min and Servo max
   int analog_value = int(float(pulse_wide) / 1000000 * SERVO_FREQ * 4096);
   Serial.print("Angle: "); Serial.print(angle);
   Serial.print(" pulse: "); Serial.println(analog_value);
   return analog_value;
}


void moveMotorToPosition(uint8_t motor, int position_in_degrees){
  pwm.setPWM(motor, 0, angleToPulse(position_in_degrees));
}



void setup() {
  #ifdef SERIAL_DEBUGING
    Serial.begin(9600);
  #endif

  //initialize servo board
  pwm.begin();
  // pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
}


void loop() {
  Serial.println("set servo @ 0");
  moveMotorToPosition(0, 0);
  moveMotorToPosition(1, 0);
  moveMotorToPosition(2, 0);
  delay(5000);

  Serial.println("set servo @ 180");
  moveMotorToPosition(0, 180);
  moveMotorToPosition(1, 180);
  moveMotorToPosition(2, 180);
  delay(10000);
}
