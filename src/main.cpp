#define FIRMWARE_VERSION 226

// device_id, numer used a position in array to get last octet of MAC and static IP
// prototype 0, unit 1, unit 2... unit 7.

// ****************
#define DEVICE_ID 0
// ****************

// Enable/Disable modules
#define SERIAL_DEBUGING
#define NEOPIXEL
#define WEB_SERVER

//-------------------------------- pins definition -----------------------------
// Focus
#define MOTOR1DIR_PIN 15
#define MOTOR1STEP_PIN 14
#define MOTOR1FAULT 16

// Aperture
#define MOTOR2DIR_PIN 22
#define MOTOR2STEP_PIN 21
#define MOTOR2FAULT 20
// Zoom
#define MOTOR3DIR_PIN 5
#define MOTOR3STEP_PIN 6
#define MOTOR3FAULT 7

#define ENCODER_N 3 //Number limit of the encoder
#define INT_PIN 17 // Definition of the encoder interrupt pin

#define PIXEL_PIN 2
#define NUMPIXELS 1

//-------------------------------- settings ------------------------------------
#define SERIAL_SPEED 115200

// encoders settings
#define potStep 10
#define potMax 1000


//------------------------------------------------------------------------------
#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>

#include <OSCMessage.h>

#include <Wire.h>
#include <i2cEncoderLibV2.h>

#include <AccelStepper.h>

#ifdef NEOPIXEL
  #include <Adafruit_NeoPixel.h>
#endif

//------------------------------ Stepper motors --------------------------------
// Bipolar motor, converted 28BYJ-48 with DRV8834 driver
// 28BYJ-48 motor runs in full step mode, each step corresponds to a rotation of 11.25°.
// That means there are 32 steps per revolution (360°/11.25° = 32). What this means is that
// there are actually 32*63.68395 steps per revolution = 2037.8864 ~ 2038 steps!

// Define a stepper and the pins it will use
// AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

//set up the accelStepper intance
//the "1" tells it we are using a driver
AccelStepper stepper1(AccelStepper::DRIVER, MOTOR1STEP_PIN, MOTOR1DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR2STEP_PIN, MOTOR2DIR_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, MOTOR3STEP_PIN, MOTOR3DIR_PIN);

//------------------------------ I2C encoders ----------------------------------
// Connections:
// - -> GND
// + -> 5V
// SDA -> A4
// SCL -> A5
// INT -> 3 temporary for tests

//Class initialization with the I2C addresses
i2cEncoderLibV2 RGBEncoder[ENCODER_N] = { i2cEncoderLibV2(0x01),
                                          i2cEncoderLibV2(0x02),
                                          i2cEncoderLibV2(0x03),
                                        };
uint8_t encoder_status, i;

//---------------------------- MAC & IP list ----------------------------------
// Change #define DEVICE_ID to a number from 0 to 7 on top of the code to
// assign MAC and IP for device, they mus be unique within the netowrk

byte MAC_ARRAY[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
int IP_ARRAY[] = {240, 241, 242, 243, 244, 245, 246, 247};
//-----------------------------------------------------------------------------

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, MAC_ARRAY[DEVICE_ID]
};
IPAddress ip(10, 0, 10, IP_ARRAY[DEVICE_ID]);

bool isLANconnected = false;
// bool isUDPconnected = false;

//----------------------------- Setup for OSC ----------------------------------
EthernetUDP Udp;

// OSC destination address, 255 broadcast
IPAddress targetIP(10, 0, 10, 101);   // Isadora machine IP address
const unsigned int destPort = 9999;          // remote port to receive OSC
const unsigned int localPort = 8888;        // local port to listen for OSC packets

unsigned long previousMillis = 0;
const long interval = 1000;
long uptime = 0;

char osc_prefix[16];                  // device OSC prefix message, i.e /camera1


#ifdef WEB_SERVER
  EthernetServer server(80);
  String readString;
  int potsPositionsArray[] = {0,0,0};
#endif

#ifdef NEOPIXEL
  Adafruit_NeoPixel pixels(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif

//***************************** Functions *************************************

void moveMotorToPosition(uint8_t motor, int position){
  switch(motor) {
    case 1:
      #ifdef SERIAL_DEBUGING
        Serial.print("moving motor 1 to position:");
        Serial.println(position);
      #endif
      stepper1.moveTo(position);
      break;
    case 2:
      #ifdef SERIAL_DEBUGING
        Serial.print("moving motor 2 to position:");
        Serial.println(position);
      #endif
      stepper2.moveTo(position);
      break;
    case 3:
      #ifdef SERIAL_DEBUGING
        Serial.print("moving motor 3 to position:");
        Serial.println(position);
      #endif
      stepper3.moveTo(position);
      break;
  }
}

//Callback when the encoder is rotated
void encoder_rotated(i2cEncoderLibV2* obj) {
  if (obj->readStatus(i2cEncoderLibV2::RINC))
    #ifdef SERIAL_DEBUGING
      Serial.print("Increment ");
    #endif
  else
    #ifdef SERIAL_DEBUGING
      Serial.print("Decrement ");
    #endif
    int motorID = (obj->id) + 1;
    int position =obj->readCounterInt();
    #ifdef SERIAL_DEBUGING
      Serial.print(motorID);
      Serial.print(": ");
      Serial.println(position);
    #endif

  obj->writeRGBCode(0x00FF00);

  moveMotorToPosition(motorID, position);

  // #ifdef WEB_SERVER
  //   potsPositionsArray[motorID] = position;
  // #endif
}

void encoder_click(i2cEncoderLibV2* obj) {
  Serial.print("Push: ");
  Serial.println(obj->id);
  obj->writeRGBCode(0x0000FF);
}

void encoder_thresholds(i2cEncoderLibV2* obj) {
  if (obj->readStatus(i2cEncoderLibV2::RMAX))
    Serial.print("Max: ");
  else
    Serial.print("Min: ");
  Serial.println(obj->id);
  obj->writeRGBCode(0xFF0000);
}

void encoder_fade(i2cEncoderLibV2* obj) {
  obj->writeRGBCode(0x000000);
}

int uptimeInSecs(){
  return (int)(millis()/1000);
}

void servo1_OSCHandler(OSCMessage &msg, int addrOffset) {
  // TODO replace with one function for all OSC with motor number?
  int inValue = msg.getFloat(0);
  #ifdef SERIAL_DEBUGING
    Serial.print("osc servo 1 update: ");
    Serial.println(inValue);
  #endif

  RGBEncoder[0].writeCounter((int32_t) inValue); //Reset of the CVAL register
  moveMotorToPosition(1, inValue);
}

void servo2_OSCHandler(OSCMessage &msg, int addrOffset) {
  int inValue = msg.getFloat(0);
  #ifdef SERIAL_DEBUGING
    Serial.print("osc servo 2 update: ");
    Serial.println(inValue);
  #endif

  RGBEncoder[1].writeCounter((int32_t) inValue); //Reset of the CVAL register
  moveMotorToPosition(2, inValue);
}

void servo3_OSCHandler(OSCMessage &msg, int addrOffset) {
  // TODO check isadora sending int?
  int inValue = msg.getFloat(0);
  #ifdef SERIAL_DEBUGING
    Serial.print("osc servo 3 update: ");
    Serial.println(inValue);
  #endif

  RGBEncoder[2].writeCounter((int32_t) inValue); //Reset of the CVAL register
  moveMotorToPosition(3, inValue);
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
      // TODO add dynamic device number based on setting
      msgIn.route("/camera1/servo/1", servo1_OSCHandler);
      msgIn.route("/camera1/servo/2", servo2_OSCHandler);
      msgIn.route("/camera1/servo/3", servo3_OSCHandler);
      // msgIn.route("/device1/localise", localise_OSCHandler);

      #ifdef NEOPIXEL
      pixels.setPixelColor(0, pixels.Color(255, 0, 150));
      pixels.show();
      #endif
    }

    //finish reading this packet:
    Udp.flush();
    //restart UDP connection to receive packets from other clients
    Udp.stop();
    Udp.begin(localPort);
  }
}

void sendOSCmessage(char* name, int value){
  char message_osc_header[32];
  message_osc_header[0] = {0};
  strcat(message_osc_header, osc_prefix);
  strcat(message_osc_header, name);
  OSCMessage message(message_osc_header);
  message.add(value);
  Udp.beginPacket(targetIP, destPort);
  message.send(Udp);
  Udp.endPacket();
  message.empty();
}

void sendOSCreport(){
  #ifdef SERIAL_DEBUGING
    Serial.print("Sending OSC raport ");
  #endif
  sendOSCmessage("/ver", FIRMWARE_VERSION);
  sendOSCmessage("/uptime", uptimeInSecs());
  sendOSCmessage("/motor1/position", stepper1.currentPosition());
  sendOSCmessage("/motor2/position", stepper2.currentPosition());
  sendOSCmessage("/motor3/position", stepper3.currentPosition());
  #ifdef SERIAL_DEBUGING
    Serial.println(" *");
  #endif
}

bool checkEthernetConnection(){
  // // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    #ifdef SERIAL_DEBUGING
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    #endif

    #ifdef NEOPIXEL
      pixels.setPixelColor(0, pixels.Color(150, 0, 0));
      pixels.show();
    #endif

    return false;
  }
  else if (Ethernet.linkStatus() == LinkOFF) {
    #ifdef SERIAL_DEBUGING
      Serial.println("Ethernet cable is not connected.");
    #endif

    #ifdef NEOPIXEL
      pixels.setPixelColor(0, pixels.Color(150, 0, 0));
      pixels.show();
    #endif

    return false;
  }
  else if (Ethernet.linkStatus() == LinkON) {
    #ifdef SERIAL_DEBUGING
      Serial.println("Ethernet cable is connected.");
    #endif

    #ifdef NEOPIXEL
      pixels.setPixelColor(0, pixels.Color(0, 0, 150));
      pixels.show();
    #endif

    return true;
  }
}

void checkMotorFaults(){
  Serial.print("Motors faults reading: ");
  Serial.print(digitalRead(MOTOR1FAULT));
  Serial.print(digitalRead(MOTOR2FAULT));
  Serial.println(digitalRead(MOTOR3FAULT));
}

//******************************************************************************

void setup() {
  // neopixel
  #ifdef NEOPIXEL
    pixels.begin();
    pixels.setBrightness(20);
    pixels.clear();
    pixels.show();

    pixels.setPixelColor(0, pixels.Color(150, 150, 0));
    pixels.show();
  #endif

  #ifdef SERIAL_DEBUGING
    Serial.begin(SERIAL_SPEED);
  #endif

  #ifdef SERIAL_DEBUGING
    Serial.print("\r\nFirmware Ver: "); Serial.print(FIRMWARE_VERSION);
    Serial.println(" written by Grzegorz Zajac");
    Serial.println("Compiled: " __DATE__ ", " __TIME__ ", " __VERSION__);
    Serial.println();
  #endif

//-------------------------- Initializing encoders -----------------------------
  #ifdef SERIAL_DEBUGING
    Serial.println("initializing encoders");
  #endif
  uint8_t enc_cnt;

  pinMode(INT_PIN, INPUT);

  Wire.begin();
  //Reset of all the encoder
  for (enc_cnt = 0; enc_cnt < ENCODER_N; enc_cnt++) {
    RGBEncoder[enc_cnt].reset();
  }

  // Initialization of the encoders
  for (enc_cnt = 0; enc_cnt < ENCODER_N; enc_cnt++) {
    RGBEncoder[enc_cnt].begin(
      i2cEncoderLibV2::INT_DATA | i2cEncoderLibV2::WRAP_DISABLE
      | i2cEncoderLibV2::DIRE_RIGHT
      | i2cEncoderLibV2::IPUP_ENABLE
      | i2cEncoderLibV2::RMOD_X1
      | i2cEncoderLibV2::RGB_ENCODER);
    RGBEncoder[enc_cnt].writeCounter((int32_t) 0); //Reset of the CVAL register
    RGBEncoder[enc_cnt].writeMax((int32_t) potMax); //Set the maximum threshold to 50
    RGBEncoder[enc_cnt].writeMin((int32_t) 0); //Set the minimum threshold to 0
    RGBEncoder[enc_cnt].writeStep((int32_t) potStep); //The step at every encoder click is 1
    RGBEncoder[enc_cnt].writeRGBCode(0);
    RGBEncoder[enc_cnt].writeFadeRGB(3); //Fade enabled with 3ms step
    RGBEncoder[enc_cnt].writeAntibouncingPeriod(25); //250ms of debouncing
    RGBEncoder[enc_cnt].writeDoublePushPeriod(0); //Set the double push period to 500ms

    /* Configure the events */
    RGBEncoder[enc_cnt].onChange = encoder_rotated;
    RGBEncoder[enc_cnt].onButtonRelease = encoder_click;
    RGBEncoder[enc_cnt].onMinMax = encoder_thresholds;
    RGBEncoder[enc_cnt].onFadeProcess = encoder_fade;

    /* Enable the I2C Encoder V2 interrupts according to the previus attached callback */
    RGBEncoder[enc_cnt].autoconfigInterrupt();
    RGBEncoder[enc_cnt].id = enc_cnt;
  }

//-------------------------- Initializing steppers -----------------------------
  #ifdef SERIAL_DEBUGING
    Serial.println("initializing steppers");
  #endif

  pinMode(MOTOR1FAULT, INPUT);
  pinMode(MOTOR2FAULT, INPUT);
  pinMode(MOTOR3FAULT, INPUT);

  stepper1.setMaxSpeed(500);
  stepper1.setAcceleration(200);

  stepper2.setMaxSpeed(500);
  stepper2.setAcceleration(200);

  stepper3.setMaxSpeed(500);
  stepper3.setAcceleration(200);

//-------------------------- Initializing ethernet -----------------------------
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);    // begin reset the WIZ820io
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);  // de-select WIZ820io
  digitalWrite(9, HIGH);   // end reset pulse

  Ethernet.init(10);

  // start the Ethernet connection
  Ethernet.begin(mac, ip);

  //Create OSC message header with unit number
  osc_prefix[0] = {0};
  strcat(osc_prefix, "/camera");

  char buf[8];
  sprintf(buf, "%d", DEVICE_ID);
  strcat(osc_prefix, buf);



  #ifdef SERIAL_DEBUGING
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
    Serial.print("OSC prefix: ");
    Serial.println(osc_prefix);
    Serial.println();
    Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
  #endif

  //TODO add ifconnected condition
  Udp.begin(localPort);

  #ifdef WEB_SERVER
    server.begin();                       			   // start to listen for clients
  #endif

  #ifdef NEOPIXEL
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));
    pixels.show();
  #endif
}


void loop() {
  receiveOSCsingle();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    isLANconnected = checkEthernetConnection();
    if (isLANconnected){

      #ifdef NEOPIXEL
        pixels.setPixelColor(0, pixels.Color(0, 255, 150));
        pixels.show();
      #endif

      // sendOSCreport();

      #ifdef NEOPIXEL
        pixels.setPixelColor(0, pixels.Color(0, 0, 150));
        pixels.show();
      #endif
    }
    Serial.println(stepper1.currentPosition());
    Serial.println(stepper2.currentPosition());
    Serial.println(stepper3.currentPosition());

    checkMotorFaults();
  }

  // check pots
  uint8_t enc_cnt;
  if (digitalRead(INT_PIN) == LOW) {
    //Interrupt from the encoders, start to scan the encoder matrix
    for (enc_cnt = 0; enc_cnt < ENCODER_N; enc_cnt++) {
      if (digitalRead(INT_PIN) == HIGH) { //If the interrupt pin return high, exit from the encoder scan
        break;
      }
      RGBEncoder[enc_cnt].updateStatus();
    }
  }

  stepper1.run();
  stepper2.run();
  stepper3.run();


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
           client.println("Refresh: 3;URL='//10.0.10.240/'>");
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

           // client.print("knob 1 position: "); client.print(potsPositionsArray[0]);
           // client.print("knob 2 position: "); client.print(potsPositionsArray[1]);
           // client.print("knob 3 position: "); client.print(potsPositionsArray[2]);

           client.println("<br />");

           // client.println("<ul>");
           //   client.println("<li>");
           //   client.print("stepper 1 position: "); client.print(stepper1.currentPosition());
           //   client.println("</li>");
           //   client.println("<li>");
           //   client.print("stepper 2 position: "); client.print(stepper2.currentPosition());
           //   client.println("</li>");
           //   client.println("<li>");
           //   client.print("stepper 3 position: "); client.print(stepper3.currentPosition());
           //   client.println("</li>");
           // client.println("</ul");

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
             moveMotorToPosition(0,500);
             moveMotorToPosition(1,500);
             moveMotorToPosition(2,500);
           }
           if (readString.indexOf("?button180clicked") >0){
             #ifdef SERIAL_DEBUGING
               Serial.println("Web button pressed, setting servos @ 90");
             #endif
             moveMotorToPosition(0,1000);
             moveMotorToPosition(1,1000);
             moveMotorToPosition(2,1000);
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
