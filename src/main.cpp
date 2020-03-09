#define FIRMWARE_VERSION 106
#define SERIAL_DEBUGING     // comment it out to disable serial debuging, for production i.e.
#define SERIAL_SPEED 115200
#define WEB_SERVER

//-------- PIN MAPPING ---------------
#define ENCODER_A 5
#define ENCODER_B 6
#define KNOB_BUTTON_PIN 4

#define ENCODER_LED_R 7
#define ENCODER_LED_G 8
#define ENCODER_LED_B 9

#define PIXEL_PIN 3
/*
I2C
Teensy LC SDA 18, SCL 19
*/
//---------------------------------

#include <Arduino.h>

#include <UIPEthernet.h> // Used for Ethernet

#include <OSCBundle.h>
#include <OSCMessage.h>

// PCA9685 16-channel PWM servo driver
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <Encoder.h>
#include <Bounce2.h>

#include <Adafruit_NeoPixel.h>

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
Encoder knob(ENCODER_A, ENCODER_B);
int knob_position  = -999;
//******************************************************************************
const uint8_t knob_scaling_factor = 12;  // number of encoder ticks per 0-180 servo movment
//******************************************************************************
uint8_t knob_scaled;
unsigned long previousMillis = 0;
const long interval = 500;
long uptime = 0;

Bounce debouncer = Bounce(); // Instantiate a Bounce object
uint8_t selected_servo = 0;

// define RGB led pins     R G B
const uint8_t rgb_led_pins[] = {ENCODER_LED_R, ENCODER_LED_G, ENCODER_LED_B};

// Networking / UDP Setup
EthernetUDP Udp;

// destination address
IPAddress targetIP(10, 0, 10, 101);
const unsigned int targetPort = 9999;
const unsigned int inPort = 8888;

#ifdef WEB_SERVER
  EthernetServer server(80);                       //server port
#endif

// array of servos position: {servo1, servo2, servo3}
uint8_t servo_position[] = {0, 0, 0};

// --- neopixel ------
#define NUMPIXELS 1

Adafruit_NeoPixel pixels(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);


//------------------------------ Functions -------------------------------------

void refresh_button_led(uint8_t active_servo){
  for (uint8_t i = 0; i < 3; i++){
    if (i == active_servo) digitalWrite(rgb_led_pins[i], LOW);
    else digitalWrite(rgb_led_pins[i], HIGH);
  }
}

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
  // #ifdef SERIAL_DEBUGING
  //   Serial.print("Angle: "); Serial.print(angle);
  //   Serial.print(" pulse: "); Serial.println(analog_value);
  // #endif
  return analog_value;
}

void moveMotorToPosition(uint8_t motor, int position_in_degrees){
  pwm.setPWM(motor, 0, angleToPulse(position_in_degrees));
  servo_position[motor] = position_in_degrees;
}

void readEncoderPosition(){

  long knob_new_position;
  knob_new_position = (knob.read() / 4);

  if (knob_new_position != knob_position){
    // check current motor position
    //
    #ifdef SERIAL_DEBUGING
      Serial.print("* new pos = "); Serial.print(knob_new_position);
      Serial.print(", prev pos = "); Serial.print(knob_position);
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
      Serial.print("  |  knob pos: "); Serial.print(knob_position);
      Serial.print(" -> scaled by "); Serial.print(knob_scaling_factor);
      Serial.print(" to "); Serial.print(knob_scaled);
      Serial.print(" sent to motor "); Serial.println(selected_servo);
    #endif

    moveMotorToPosition(selected_servo, knob_scaled);

    //TODO sync position with current OSC position value
    //TODO add manual OSC flag
  };
}

void servo1_OSCHandler(OSCMessage &msg, int addrOffset) {
  int inValue = msg.getFloat(0);
  #ifdef SERIAL_DEBUGING
    Serial.print("osc servo 1 update: ");
    Serial.println(inValue);
  #endif
  moveMotorToPosition(0, inValue);
}

void servo2_OSCHandler(OSCMessage &msg, int addrOffset) {
  int inValue = msg.getFloat(0);
  #ifdef SERIAL_DEBUGING
    Serial.print("osc servo 2 update: ");
    Serial.println(inValue);
  #endif
  moveMotorToPosition(1, inValue);
}

void servo3_OSCHandler(OSCMessage &msg, int addrOffset) {
  int inValue = msg.getFloat(0);
  #ifdef SERIAL_DEBUGING
    Serial.print("osc servo 3 update: ");
    Serial.println(inValue);
  #endif
  moveMotorToPosition(2, inValue);
}

void localise_OSCHandler(OSCMessage &msg, int addrOffset) {
  int inValue = msg.getFloat(0);
  #ifdef SERIAL_DEBUGING
    Serial.print("localise! ");
    Serial.println(inValue);
  #endif
  if (inValue == 100){
    pixels.setPixelColor(0, pixels.Color(150, 150, 150));
    pixels.setBrightness(50);
    pixels.show();
  } else {
    pixels.clear();
    pixels.show();
  }
}

void receiveOSCsingle(){
  // read incoming udp packets
  OSCMessage msgIn;
  int size;
  int success;

  if( (size = Udp.parsePacket())>0)
  {

    //while((size = Udp.available()) > 0)
    while(size--)
      msgIn.fill(Udp.read());

    // route messages
    if(!msgIn.hasError()) {
      msgIn.route("/servo/1", servo1_OSCHandler);
      msgIn.route("/servo/2", servo2_OSCHandler);
      msgIn.route("/servo/3", servo3_OSCHandler);
      msgIn.route("/device1/localise", localise_OSCHandler);

    }

    //finish reading this packet:
    Udp.flush();
    //restart UDP connection to receive packets from other clients
    Udp.stop();
    success = Udp.begin(inPort);
  }
}

void checkKnobButton(){
  debouncer.update();
  if ( debouncer.rose()){
    selected_servo ++;
    if (selected_servo == 3) selected_servo = 0;

    refresh_button_led(selected_servo);

    #ifdef SERIAL_DEBUGING
     Serial.print("button pressed, current servo: "); Serial.println(selected_servo);
    #endif
  }
}

void maintainEthernetConnection(){
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

void sendOSCbundle(){
  OSCBundle bndl;
  bndl.add("/device1/ver").add(FIRMWARE_VERSION);
  uptime = (int)(millis()/1000);
  bndl.add("/device1/uptime").add(uptime);

  bndl.add("/device1/servo1/position").add(servo_position[0]);
  bndl.add("/device1/servo2/position").add(servo_position[1]);
  bndl.add("/device1/servo3/position").add(servo_position[2]);

  Udp.beginPacket(targetIP, targetPort);
  bndl.send(Udp); // send the bytes to the SLIP stream
  Udp.endPacket(); // mark the end of the OSC Packet
  bndl.empty(); // empty the bundle to free room for a new one

  //finish reading this packet:
  Udp.flush();

  //restart UDP connection to receive packets from other clients
  Udp.stop();
  Udp.begin(inPort);
}

// -----------------------------------------------------------------------------

void setup() {
  #ifdef SERIAL_DEBUGING
    Serial.begin(SERIAL_SPEED);
  #endif

  pixels.begin();
  pixels.clear();

  // neopixel test
  pixels.setPixelColor(0, pixels.Color(0, 150, 0));
  pixels.setBrightness(10);
  pixels.show();

  //set RGB led off
  for (uint8_t i = 0; i < 3; i++){
    pinMode(rgb_led_pins[i], OUTPUT);
    digitalWrite(rgb_led_pins[i], HIGH);
  }

  refresh_button_led(selected_servo);

  // add pull down resistor 10k
  pinMode(KNOB_BUTTON_PIN, INPUT);
  debouncer.attach(KNOB_BUTTON_PIN); // Attach the debouncer to a pin with pull down, switch connected to +3V3
  debouncer.interval(10); // Use a debounce interval of 25 milliseconds

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

  //TODO add check connected status if
  Udp.begin(inPort);

#ifdef WEB_SERVER
  server.begin();                       			   // start to listen for clients
#endif

}


void loop() {

  checkKnobButton();
  readEncoderPosition();

  maintainEthernetConnection();

  // TODO only if connected
  receiveOSCsingle();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sendOSCbundle();
  }

  #ifdef WEB_SERVER
  EthernetClient client = server.available();
  if (client) {
  Serial.println("new client");
  // an http request ends with a blank line
  boolean currentLineIsBlank = true;
  while (client.connected()) {
    if (client.available()) {
      char c = client.read();
      Serial.write(c);
      if (c == '\n' && currentLineIsBlank) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("Connection: close");
        client.println("Refresh: 5");
        client.println();
        client.println("<!DOCTYPE HTML>");
        client.println("<html>");
        client.print("<h1>Analogue Values</h1>");

        client.println("</html>");
        break;
      }
      if (c == '\n') {
        currentLineIsBlank = true;
      } else if (c != '\r') {
        currentLineIsBlank = false;
      }
    }
  }
  delay(1);
  // close the connection:
  client.stop();
  Serial.println("client disconnected");
  }
  #endif

}
