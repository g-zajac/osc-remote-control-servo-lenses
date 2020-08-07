#define FIRMWARE_VERSION 250

// device_id, numer used a position in array to get last octet of MAC and static IP
// prototype 0, unit 1, unit 2... unit 7.

// *********************
#define DEVICE_ID 0
// *********************

// Enable/Disable modules
#define SERIAL_DEBUGING
#define NEOPIXEL
// #define WEB_SERVER

//-------------------------------- pins definition -----------------------------
// Focus
#define MOTOR1DIR_PIN 20
#define MOTOR1STEP_PIN 16

// Aperture
#define MOTOR2DIR_PIN 22
#define MOTOR2STEP_PIN 21

// Zoom
#define MOTOR3DIR_PIN 15
#define MOTOR3STEP_PIN 14

#define ENCODER_N 3 //Number limit of the encoder
#define INT_PIN 17 // Definition of the encoder interrupt pin

#define PIXEL_PIN 6
#define NUMPIXELS 1

//-------------------------------- settings ------------------------------------
#define SERIAL_SPEED 115200

// encoders settings
#define potStep 1
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
// + -> 3V3V
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
IPAddress targetIP(10, 0, 10, 102);   // Isadora machine IP address
const unsigned int destPort = 9999;          // remote port to receive OSC
const unsigned int localPort = 8888;        // local port to listen for OSC packets

unsigned long previousMillis = 0;
const long interval = 1000;
long uptime = 0;

char osc_prefix[16];                  // device OSC prefix message, i.e /camera1


#ifdef WEB_SERVER
  EthernetServer server(80);
  String readString;
  // int potsPositionsArray[] = {0,0,0};
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
  // TODO convert float to int?
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

  #ifdef SERIAL_DEBUGING
    Serial.print("OSC header: ");
    Serial.println(message_osc_header);
  #endif

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
  // Reset of all the encoder
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

  stepper1.setMaxSpeed(500);
  stepper1.setAcceleration(200);

  stepper2.setMaxSpeed(500);
  stepper2.setAcceleration(200);

  stepper3.setMaxSpeed(5000);
  stepper3.setAcceleration(5000);

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

      sendOSCreport();

      #ifdef NEOPIXEL
        pixels.setPixelColor(0, pixels.Color(0, 0, 150));
        pixels.show();
      #endif
    }

    Serial.println(stepper1.currentPosition());
    Serial.println(stepper2.currentPosition());
    Serial.println(stepper3.currentPosition());

    // checkMotorFaults();
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
  EthernetClient client = server.available();
  if (client) {
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        if (c == 'n' && currentLineIsBlank) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("Connection: close");
        client.println("Refresh: 5");
        client.println();
        client.println("<!DOCTYPE HTML>");
        client.println("<html>");
        client.println("<title>Example</title>");
        client.print("<p>Hello World</p>");
        client.println("</html>");
        break;
      }
      if (c == 'n') {
        currentLineIsBlank = true;
      } else if (c != 'r') {
        currentLineIsBlank = false;
      }
    }
  }
    delay(1);
    client.stop();
  }                   			   // start to listen for clients
  #endif

}
