#define FIRMWARE_VERSION 116
#define DEVICE_ID 131         // NOTE number? IP address i.e 101, 102, 103, 104... isadora 100
#define SERIAL_DEBUGING     // comment it out to disable serial debuging, for production i.e.
#define SERIAL_SPEED 115200
#define WEB_SERVER          // comment out to disable webserver
#define NEOPIXEL            // comment out to disable status led (neopixel)

//-------- PIN MAPPING ---------------
#define ENCODER_A 6
#define ENCODER_B 5
#define KNOB_BUTTON_PIN 4

#define ENCODER_LED_R 7
#define ENCODER_LED_G 8
#define ENCODER_LED_B 9

#define PIXEL_PIN 3
#define BRIGHTNESS 10

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

#ifdef NEOPIXEL
  #include <Adafruit_NeoPixel.h>
#endif

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
IPAddress IP(10,0,10,DEVICE_ID);

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
//*****************************************************************************************
const uint8_t knob_scaling_factor = 12;  // number of encoder ticks per 0-180 servo movment
//*****************************************************************************************
uint8_t knob_scaled;
unsigned long previousMillis = 0;
const long interval = 500;
long uptime = 0;

Bounce debouncer = Bounce(); // Initiate a Bounce object
uint8_t selected_servo = 0;

// define RGB led pins     R G B
const uint8_t rgb_led_pins[] = {ENCODER_LED_R, ENCODER_LED_G, ENCODER_LED_B};

// Networking / UDP Setup
EthernetUDP Udp;

// destination address
IPAddress targetIP(10, 0, 10, 101);   // Isadora machine IP address
const unsigned int targetPort = 9999;
const unsigned int inPort = 8888;

#ifdef WEB_SERVER
  EthernetServer server(80);                       //server port
#endif

// array of servos position: {servo1, servo2, servo3}
uint8_t servo_position[] = {0, 0, 0};

  // --- neopixel ------
#ifdef NEOPIXEL
  #define NUMPIXELS 1
  Adafruit_NeoPixel pixels(NUMPIXELS, PIXEL_PIN, NEO_RGB + NEO_KHZ800);
  // int previousPixelColor[] = {0,0,0};
#endif

#ifdef WEB_SERVER
  // ---- web server ---
  String readString;
#endif

//------------------------------ Functions -------------------------------------
int uptimeInSecs(){
  return (int)(millis()/1000);
}

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
  #ifdef NEOPIXEL
    pixels.setPixelColor(0, pixels.Color(255, 255, 0));
    pixels.show();
  #endif
  int inValue = msg.getFloat(0);
  #ifdef SERIAL_DEBUGING
    Serial.print("osc servo 1 update: ");
    Serial.println(inValue);
  #endif
  moveMotorToPosition(0, inValue);
  #ifdef NEOPIXEL
    pixels.clear();
    pixels.show();
  #endif
}

void servo2_OSCHandler(OSCMessage &msg, int addrOffset) {
  #ifdef NEOPIXEL
    pixels.setPixelColor(0, pixels.Color(255, 255, 0));
    pixels.show();
  #endif
  int inValue = msg.getFloat(0);
  #ifdef SERIAL_DEBUGING
    Serial.print("osc servo 2 update: ");
    Serial.println(inValue);
  #endif
  moveMotorToPosition(1, inValue);
  #ifdef NEOPIXEL
    pixels.clear();
    pixels.show();
  #endif
}

void servo3_OSCHandler(OSCMessage &msg, int addrOffset) {
  #ifdef NEOPIXEL
    pixels.setPixelColor(0, pixels.Color(255, 255, 0));
    pixels.show();
  #endif
  int inValue = msg.getFloat(0);
  #ifdef SERIAL_DEBUGING
    Serial.print("osc servo 3 update: ");
    Serial.println(inValue);
  #endif
  moveMotorToPosition(2, inValue);
  #ifdef NEOPIXEL
    pixels.clear();
    pixels.show();
  #endif
}

void localise_OSCHandler(OSCMessage &msg, int addrOffset) {
  int inValue = msg.getFloat(0);
  #ifdef SERIAL_DEBUGING
    Serial.print("localise! ");
    Serial.println(inValue);
  #endif
  if (inValue == 100){
    #ifdef NEOPIXEL
      pixels.setPixelColor(0, pixels.Color(255, 255, 255));
      pixels.show();
    #endif
  } else {
    #ifdef NEOPIXEL
      pixels.clear();
      pixels.show();
    #endif
  }
}

void receiveOSCsingle(){
  // read incoming udp packets
  OSCMessage msgIn;
  int size;

  if( (size = Udp.parsePacket())>0)
  {

    //while((size = Udp.available()) > 0)
    while(size--)
      msgIn.fill(Udp.read());

    // route messages
    if(!msgIn.hasError()) {
      msgIn.route("/device1/servo/1", servo1_OSCHandler);
      msgIn.route("/device1/servo/2", servo2_OSCHandler);
      msgIn.route("/device1/servo/3", servo3_OSCHandler);
      msgIn.route("/device1/localise", localise_OSCHandler);
    }

    //finish reading this packet:
    Udp.flush();
    //restart UDP connection to receive packets from other clients
    Udp.stop();
    Udp.begin(inPort);
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

void sendOSCbundle(){
  OSCBundle bndl;
  bndl.add("/device1/ver").add(FIRMWARE_VERSION);
  bndl.add("/device1/uptime").add(uptimeInSecs());

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

  #ifdef NEOPIXEL
    pixels.begin();
    delay(100);
    // neopixel test
    pixels.setBrightness(BRIGHTNESS);
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    pixels.show();
  #endif
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

  // TODO only if connected
  receiveOSCsingle();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sendOSCbundle();
  }

  #ifdef WEB_SERVER
  // Create a client connection
  EthernetClient client = server.available();
  if (client) {
    while (client.connected()) {
      if (client.available()) {
        #ifdef NEOPIXEL
          pixels.setPixelColor(0, pixels.Color(0, 0, 255));
          pixels.show();
        #endif

        char c = client.read();

        //read char by char HTTP request
        if (readString.length() < 100) {
          //store characters to string
          readString += c;
          //Serial.print(c);
         }

         //if HTTP request has ended
         if (c == '\n') {
           Serial.println(readString); //print to serial monitor for debuging

           client.println("HTTP/1.1 200 OK"); //send new page
           client.println("Content-Type: text/html");
           // client.println("<meta http-equiv=\"refresh\" content=>'0;url=http://arduino.cc/'");
           client.println("Refresh: 3;URL='//10.0.10.131/'>");
           client.println("Connection: close");
           // client.println("Refresh: 3");
           client.println();
           client.println("<!DOCTYPE HTML>");
           client.println("<HTML>");
           client.println("<HEAD>");
           client.println("<TITLE>Camera Lens Controler</TITLE>");
           client.println("</HEAD>");
           client.println("<BODY>");
           client.println("<H1>SSP Camera Lens controler</H1>");
           client.println("<hr />");
           client.println("<br />");
           client.println("<h3><a href=\"/?buttonIDclicked\"\">Device ID:</a>");
           client.print(DEVICE_ID); client.println("</h3>");
           client.println("<br />");
           client.print("Firmware version: ");
           client.println(FIRMWARE_VERSION);
           client.println("<br />");
           client.println("IP address: ");
           client.println(Ethernet.localIP());
           client.println("<br />");
           client.print("uptime: ");
           client.print(uptimeInSecs());
           client.println(" secs");
           client.println("<br />");

           client.print("knob position: "); client.print(knob_position);
           client.print(" -> scaled by "); client.print(knob_scaling_factor);
           client.print(" to "); client.println(knob_scaled);
           client.println("<br />");

           client.println("<ul>");
           for (uint8_t i = 0; i < 3; i++){
             client.println("<li>");
             client.print("servo "); client.println(i);
             client.println(" @ ");
             client.print(servo_position[i]);
             if (i == selected_servo){
               switch (i){
                 case 0:
                 client.print("<span style='color:red;'> <- selected</span>");
                 break;
                 case 1:
                 client.print("<span style='color:green;'> <- selected</span>");
                 break;
                 case 2:
                 client.print("<span style='color:blue;'> <- selected</span>");
                 break;
               }
             }
             client.println("</li>");
           }
           client.println("</ul");
           client.println("<br />");


           client.println("<br />");
           client.println("<a href=\"/?button0clicked\"\"><button class='button' type='button'>Set Servos @ 0</button></a>");
           client.println("<a href=\"/?button90clicked\"\"><button class='button' type='button'>Set Servos @ 90</button></a>");
           client.println("<a href=\"/?button180clicked\"\"><button class='button' type='button'>Set Servos @ 180</button></a>");
           client.println("<br />");



           client.println("<br />");
           client.println("</BODY>");
           client.println("</HTML>");

           client.println("<style type='text/css'>");
             client.println("body {background-color: #222222; color: #fefefe; font-family:  Helvetica, Arial, sans-serif; font-weight: lighter;}");
             client.println("h1 {color: #104bab}");
             client.println("h3 {color: #ff5620}");

             client.println(".button {background-color: #222222; color: white; border: 1px solid #104bab; border-radius: 2px; padding: 15px 32px; margin: 4px 2px; font-size: 16px; cursor: pointer;}");

           client.println("</style>");

           delay(1);
           //stopping client
           client.stop();
           //controls the Arduino if you press the buttons

           if (readString.indexOf("?button0clicked") >0 ){
             #ifdef SERIAL_DEBUGING
               Serial.println("Web button pressed, setting servos @ 0");
             #endif
             moveMotorToPosition(0,0);
             moveMotorToPosition(1,0);
             moveMotorToPosition(2,0);
           }
           if (readString.indexOf("?button90clicked") >0){
             #ifdef SERIAL_DEBUGING
               Serial.println("Web button pressed, setting servos @ 90");
             #endif
             moveMotorToPosition(0,90);
             moveMotorToPosition(1,90);
             moveMotorToPosition(2,90);
           }
           if (readString.indexOf("?button180clicked") >0){
             #ifdef SERIAL_DEBUGING
               Serial.println("Web button pressed, setting servos @ 90");
             #endif
             moveMotorToPosition(0,180);
             moveMotorToPosition(1,180);
             moveMotorToPosition(2,180);
           }
           if (readString.indexOf("?buttonIDclicked") >0){
             #ifdef SERIAL_DEBUGING
               Serial.println("Web button pressed, identifing unit with LED");
             #endif
             #ifdef NEOPIXEL
               pixels.setPixelColor(0, pixels.Color(150, 150, 150));
               pixels.setBrightness(50);
               pixels.show();
               delay(500);
               pixels.clear();
               pixels.show();
             #endif
           }
            //clearing string for next read
            readString="";
         }
          #ifdef NEOPIXEL
           pixels.clear();
           pixels.show();
          #endif
       }
    }
  }                      			   // start to listen for clients
  #endif

}
