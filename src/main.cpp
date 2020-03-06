#define SERIAL_DEBUGING     // comment it out to disable serial debuging, for production i.e.
#define SERIAL_SPEED 115200

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

/*
Rotary encoder wireing
[connected to arduino pin]
[5](1)   A  1         4 -  - brown (4)[16]
[G](2)   C  2         5 -  -
[6](3)   B  3    X    6 - Switch -  (5)[15]
                          7 -
                          8 - Common GND -  (6)[GND]
*/
Encoder knob(6, 5);
int knob_position  = -999;
uint8_t knob_scaling_factor = 12;   //higher number, more impulses per 0-180, one knob rotation has 24 jumps
uint8_t knob_scaled;
unsigned long previousMillis = 0;
const long interval = 250;

//------------------------------ Functions -------------------------------------

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
    // Serial.print("Angle: "); Serial.print(angle);
    // Serial.print(" pulse: "); Serial.println(analog_value);
  #endif
  return analog_value;
}

void moveMotorToPosition(uint8_t motor, int position_in_degrees){
  pwm.setPWM(motor, 0, angleToPulse(position_in_degrees));
}

void updateEncoderPosition(){

  long knob_new_position;
  knob_new_position = (knob.read() / 4);

  if (knob_new_position != knob_position){
    #ifdef SERIAL_DEBUGING
      Serial.print("new = "); Serial.print(knob_new_position);
      Serial.print(", prev = "); Serial.print(knob_position);
    #endif
    knob_position = knob_new_position;
    // set limists
    if (knob_position < 0) {
      knob_position = 0;
      knob.write(0);
    }

    // encoder has 24 jumps / rotation
    knob_scaled = map(knob_position, 0, knob_scaling_factor, 0, 180);

    if (knob_scaled > 180){
      knob_position = knob_scaling_factor;
      knob.write(knob_scaling_factor * 4);
    }

    #ifdef SERIAL_DEBUGING
      Serial.print("  |  knob position "); Serial.print(knob_position);
      Serial.print(" -> scaled by "); Serial.print(knob_scaling_factor);
      Serial.print(" to "); Serial.println(knob_scaled);
    #endif

    moveMotorToPosition(0, knob_scaled);

    //TODO sync position with current OSC position value
    //TODO add manual OSC flag
  };
}

// -----------------------------------------------------------------------------

void setup() {
  #ifdef SERIAL_DEBUGING
    Serial.begin(SERIAL_SPEED);
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

  updateEncoderPosition();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // #ifdef SERIAL_DEBUGING
    //   Serial.print("encoder position = "); Serial.println(knob_position);
    // #endif
  }





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
}
