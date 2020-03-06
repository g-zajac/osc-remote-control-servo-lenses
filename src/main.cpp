#define SERIAL_DEBUGING     // comment it out to disable serial debuging, for production i.e.

#include <Arduino.h>
#include <UIPEthernet.h> // Used for Ethernet

// PCA9685 16-channel PWM servo driver
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <Encoder.h>

// default address 0x40, default board setting
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*
MG996R
dead band: 0.050ms
period 10ms / 50Hz
1ms - 2ms
*/

#define MIN_PULSE_WIDTH  550 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define MAX_PULSE_WIDTH  2600 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


// Ethernet MAC address - must be unique on your network - MAC Reads T4A001 in hex (unique in your network)
byte mac[] = { 0x54, 0x34, 0x41, 0x30, 0x30, 0x31 };
IPAddress IP(10,0,10,132);






#ifdef SERIAL_DEBUGING
void printIPAddress()
  {
    Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
    Serial.print("IP Address        : ");
    Serial.println(Ethernet.localIP());
    Serial.print("Subnet Mask       : ");
    Serial.println(Ethernet.subnetMask());
    Serial.print("Default Gateway IP: ");
    Serial.println(Ethernet.gatewayIP());
    Serial.print("DNS Server IP     : ");
    Serial.println(Ethernet.dnsServerIP());
    Serial.println();
    Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
  }
#endif

int angleToPulse(int angle){
  int pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);   // map angle of 0 to 180 to Servo min and Servo max
  int analog_value = int(float(pulse_wide) / 1000000 * SERVO_FREQ * 4096);
  #ifdef SERIAL_DEBUGING
    Serial.print("Angle: "); Serial.print(angle);
    Serial.print(" pulse: "); Serial.println(analog_value);
  #endif
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

/* DHCP
  // start the Ethernet connection:
  if (Ethernet.begin(mac) == 0) {
      Serial.println("Failed to configure Ethernet using DHCP");
      // no point in carrying on, so do nothing forevermore:
      for (;;);
    }
    // print your local IP address:
    printIPAddress();
*/

// Static
  Ethernet.begin(mac, IP);
  #ifdef SERIAL_DEBUGING
    printIPAddress();
  #endif

  // initialize digital pin LED_BUILTIN as an output.
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
}

void loop() {

  switch (Ethernet.maintain())
  {
    case 1:
      //renewed fail
      #ifdef SERIAL_DEBUGING
        Serial.println("Error: renewed fail");
      #endif
      break;

    case 2:
      //renewed success
      #ifdef SERIAL_DEBUGING
        Serial.println("Renewed success");
        //print your local IP address:
        printIPAddress();
      #endif
      break;

    case 3:
      //rebind fail
      #ifdef SERIAL_DEBUGING
        Serial.println("Error: rebind fail");
      #endif
      break;

    case 4:
      //rebind success
      #ifdef SERIAL_DEBUGING
        Serial.println("Rebind success");
        //print your local IP address:
        printIPAddress();
      #endif
      break;

    default:
    //nothing happened
    break;

   }

  // Serial.println("set servo @ 0");
  // moveMotorToPosition(0, 0);
  // moveMotorToPosition(1, 0);
  // moveMotorToPosition(2, 0);
  // delay(5000);
  //
  // Serial.println("set servo @ 180");
  // moveMotorToPosition(0, 180);
  // moveMotorToPosition(1, 180);
  // moveMotorToPosition(2, 180);
  // delay(10000);
}
