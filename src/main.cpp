#define FIRMWARE_VERSION 310

// device_id, numer used a position in array to get last octet of MAC and static IP
// prototype 0, unit 1, unit 2... unit 7.

// ******************************
// Device ID stored in EEPROM @ 0
// ******************************

// Enable/Disable modules
#define SERIAL_DEBUGING
#define NEOPIXEL
#define WEB_SERVER

//-------------------------------- pins definition -----------------------------

// Aperture
#define MOTOR1DIR_PIN 22
#define MOTOR1STEP_PIN 21

// Focus
#define MOTOR2DIR_PIN 20
#define MOTOR2STEP_PIN 16

// Zoom
#define MOTOR3DIR_PIN 15
#define MOTOR3STEP_PIN 14


#define ENCODER_N 3 //Number limit of the encoder
#define INT_PIN 17 // Definition of the encoder interrupt pin
#define POT_CHECK 4

#define PIXEL_PIN 6
#define NUMPIXELS 1

//-------------------------------- settings ------------------------------------
#define SERIAL_SPEED 115200

//------------------------------------------------------------------------------
#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetBonjour.h>

// TODO remove redundant
#include <OSCMessage.h>
#include <OSCBundle.h>

#include <Wire.h>
#include <i2cEncoderLibV2.h>

#include <AccelStepper.h>

#ifdef NEOPIXEL
  #include <Adafruit_NeoPixel.h>
#endif

#include <EEPROM.h>

//------------------------------ Stepper motors --------------------------------
// Define a stepper and the pins it will use
// AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

//set up the accelStepper intance
//the "1" tells it we are using a driver

AccelStepper stepper1(AccelStepper::DRIVER, MOTOR1STEP_PIN, MOTOR1DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR2STEP_PIN, MOTOR2DIR_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, MOTOR3STEP_PIN, MOTOR3DIR_PIN);

// AccelStepper pointers
AccelStepper *stepper[] = {&stepper1, &stepper2, &stepper3};

bool homeing[] = { false, false, false };

// default values, may be overwritten by OSC
int HOMEING_POSITION_APERTURE = -2048;
int HOMEING_POSITION_FOCUS = -2048;
int HOMEING_POSITION_ZOOM = -2048;

int HOMING_POSITIONS[] = { HOMEING_POSITION_APERTURE, HOMEING_POSITION_FOCUS, HOMEING_POSITION_ZOOM };

// default motors speed and acceleration
#define APERTURE_SPEED 500
#define APERTURE_ACCELERATION 1000

#define FOCUS_SPEED 500
#define FOCUS_ACCELERATION 1000

#define ZOOM_SPEED 500
#define ZOOM_ACCELERATION 1000


int button1value = 2048;
int button2value = 2048;
int button3value = 2048;

//------------------------------ I2C encoders ----------------------------------
// Connections:
// - -> GND
// + -> 3V3V
// SDA -> 18
// SCL -> 19
// INT -> 17

//Class initialization with the I2C addresses
// address vs function
// from top (cat5 socket)
// 0x01 - pot 1
// 0x02 - pot 2
// 0x03 - pot 3

// Indexing order motors and IDs
// 1 - Aperture
// 2 - Focus
// 3 - Zoom

i2cEncoderLibV2 RGBEncoder[ENCODER_N] = { i2cEncoderLibV2(0x02),
                                          i2cEncoderLibV2(0x01),
                                          i2cEncoderLibV2(0x03),
                                        };
uint8_t encoder_status, i;

bool remote_connected = false;
bool lock_remote = false;

bool toggle[] = { 1, 1, 1 };  // starting with coarse adjustment
float brightness = 1.0;

// encoders settings
// Focus, Aperture, Zoom
int potFineStep[] = { 1, 1, 1 };
int potCoarseStep[] = {10 ,10, 10};
int potMin[] = { 0, 0, 0};
int potMax[] = { 3400, 1700, 1580 };  // 6144 = 3 truns

//---------------------------- MAC & IP list ----------------------------------
// id stored in EEPROM, id points on array index and
// assign MAC and IP for device, they mus be unique within the netowrk

byte MAC_ARRAY[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
int IP_ARRAY[] = {200, 201, 202, 203, 204, 205, 206, 207};
//-----------------------------------------------------------------------------

// get the device ID from EEPROM
int device_id = EEPROM.read(0);

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, MAC_ARRAY[device_id]
};
IPAddress ip(10, 0, 10, IP_ARRAY[device_id]);

bool isLANconnected = false;
// bool isUDPconnected = false;

//----------------------------- Setup for OSC ----------------------------------
EthernetUDP Udp;

// OSC destination address, 255 broadcast
IPAddress targetIP(10, 0, 10, 101);   // Isadora machine IP address
const unsigned int destPort = 1234;          // remote port to receive OSC
const unsigned int localPort = 4321;        // local port to listen for OSC packets

unsigned long previousMillis = 0;
long interval = 100;
long uptime = 0;

char osc_prefix[16];                  // device OSC prefix message, i.e /camera1

#ifdef WEB_SERVER
  EthernetServer server(80);
  String readString;
  char web_address[16] = {0};                  // host name + .local - for web refreshin link
#endif

#ifdef NEOPIXEL
  Adafruit_NeoPixel pixels(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
#endif

//***************************** Functions *************************************

void moveMotorToPosition(uint8_t motor, int position){
      #ifdef SERIAL_DEBUGING
        Serial.print("moving motor "); Serial.print(motor); Serial.print(" to position "); Serial.println(position);
      #endif

      stepper[motor]->moveTo(-position);
}

int rgb2hex(int r, int g, int b, float br){
  r = (int) r * br;
  g = (int) g * br;
  b = (int) b * br;

  int color = ((long)r << 16) | ((long)g << 8 ) | (long)b;

  #ifdef SERIAL_DEBUGING
    Serial.println("COlour conversion function");
    Serial.print("R:" + String(r) + " G:" + String(g) + " B:" + String(b));
    Serial.print(" BR: "); Serial.println(br);
    Serial.print(" color: "); Serial.println(color, HEX);
  #endif

  return color;
}

// -------------------------- Encoder callbacks --------------------------------
//Callback when the encoder is rotated
void encoder_rotated(i2cEncoderLibV2* obj) {
  if (obj->readStatus(i2cEncoderLibV2::RINC))
    {
      #ifdef SERIAL_DEBUGING
        Serial.print("Encoder incremented ");
      #endif
    }
    else {
      #ifdef SERIAL_DEBUGING
        Serial.print("Encoder decremented ");
      #endif
    }

    int motorID = (obj->id);
    int position =obj->readCounterInt();
    #ifdef SERIAL_DEBUGING
      Serial.print(motorID);
      Serial.print(": ");
      Serial.println(position);
      Serial.print("global brightness: "); Serial.println(brightness);
    #endif

    obj->writeFadeRGB(3);
    if ( toggle[obj->id] ){
        // coarse adjustemnt in blue
        obj->writeRGBCode(rgb2hex(0, 0, 255, brightness));
    } else {
        // fine adjustment in green
        obj->writeRGBCode(rgb2hex(0, 255, 0, brightness));
    }


    moveMotorToPosition(motorID, position);
}

void encoder_click(i2cEncoderLibV2* obj) {
  int pushed = obj->id;
  toggle[pushed] = !toggle[pushed];

  obj->writeFadeRGB(3);
  if ( toggle[pushed] ){
      // coarse adjustemnt in blue
      obj->writeRGBCode(rgb2hex(0, 0, 255, brightness));
  } else {
      // fine adjustment in green
      obj->writeRGBCode(rgb2hex(0, 255, 0, brightness));
  }
  // update pot step (toggle = 1 -> coarse, 0 -> fine)
  if (toggle[pushed]) {
    RGBEncoder[pushed].writeStep((int32_t) potCoarseStep[pushed]);
  } else {
    RGBEncoder[pushed].writeStep((int32_t) potFineStep[pushed]);

  }

  #ifdef SERIAL_DEBUGING
    Serial.print("Toggle =  ");
    Serial.print(toggle[0]);
    Serial.print('\t');
    Serial.print(toggle[1]);
    Serial.print('\t');
    Serial.println(toggle[2]);
  #endif

}

void encoder_thresholds(i2cEncoderLibV2* obj) {
  if (obj->readStatus(i2cEncoderLibV2::RMAX))
    {
      #ifdef SERIAL_DEBUGING
        Serial.print("Max: ");
      #endif
    }
    else {
      #ifdef SERIAL_DEBUGING
        Serial.print("Min: ");
        Serial.println(obj->id);
      #endif
    }
    obj->writeRGBCode(rgb2hex(255, 0, 0, brightness));
}

void encoder_fade(i2cEncoderLibV2* obj) {
  obj->writeRGBCode(0x000000);
}
//TODO convert to human friendly texh HH:MM:SS?
int uptimeInSecs(){
  return (int)(millis()/1000);
}


//------------------------------ Stepper handlers ------------------------------
// osc receiver msg function
int receiveOSCvalue(OSCMessage &msg){

  char address[255];
  msg.getAddress(address, 0);

  int inValue;
  bool isFloat;

  if (msg.isInt(0)){
    isFloat = false;
    inValue = msg.getInt(0);
  } else if(msg.isFloat(0)){
    isFloat = true;
    inValue = msg.getFloat(0);
  }

  #ifdef SERIAL_DEBUGING
    Serial.println("");
    Serial.print("OSC message ");
    Serial.print(address);
    Serial.print(" received ");
    if(isFloat){
      Serial.print("float ");
    } else {
      Serial.print("integer ");
    }
    Serial.print("value: ");
    Serial.println(inValue);
  #endif

  return inValue;
}


void apertureMoveToOSChandler(OSCMessage &msg, int addrOffset) {
  int inValue = receiveOSCvalue(msg);

  if (remote_connected){
    RGBEncoder[0].writeCounter((int32_t) inValue); //Reset of the CVAL register
  }
  moveMotorToPosition(0, inValue);
  lock_remote = false;
}

void focusMoveToOSChandler(OSCMessage &msg, int addrOffset) {
  int inValue = receiveOSCvalue(msg);

  if (remote_connected){
      RGBEncoder[1].writeCounter((int32_t) inValue); //Reset of the CVAL register
  }
  moveMotorToPosition(1, inValue);
  lock_remote = false;
}

void zoomMoveToOSChandler(OSCMessage &msg, int addrOffset) {
  int inValue = receiveOSCvalue(msg);

  if (remote_connected){
    RGBEncoder[2].writeCounter((int32_t) inValue); //Reset of the CVAL register
  }
  moveMotorToPosition(2, inValue);
  lock_remote = false;
}

//------------------------------- LED handlers ---------------------------------
void apertureLedOSChandler(OSCMessage &msg, int addrOffset) {
  // TODO check isadora sending int?
  int r = msg.getInt(0);
  int g = msg.getInt(1);
  int b = msg.getInt(2);

  long rgb = 0;
  rgb = ((long)r << 16) | ((long)g << 8 ) | (long)b;

  RGBEncoder[0].writeFadeRGB(0);
  RGBEncoder[0].writeRGBCode(rgb);
  lock_remote = false;
}

void focusLedOSChandler(OSCMessage &msg, int addrOffset) {
  // TODO check isadora sending int?
  int r = msg.getInt(0);
  int g = msg.getInt(1);
  int b = msg.getInt(2);

  // TODO to DRY, replace rgb conversion with function
  long rgb = 0;
  rgb = ((long)r << 16) | ((long)g << 8 ) | (long)b;


  #ifdef SERIAL_DEBUGING
    Serial.println("R:" + String(r) + " G:" + String(g) + " B:" + String(b));
    Serial.print("focus rgb: "); Serial.print(rgb);
    Serial.println(" Hex: " + String(rgb, HEX));
  #endif

  RGBEncoder[1].writeFadeRGB(0);
  RGBEncoder[1].writeRGBCode(rgb);
  lock_remote = false;
}

void zoomLedOSChandler(OSCMessage &msg, int addrOffset) {
  // TODO check isadora sending int?
  int r = msg.getInt(0);
  int g = msg.getInt(1);
  int b = msg.getInt(2);

  long rgb = 0;
  rgb = ((long)r << 16) | ((long)g << 8 ) | (long)b;

  #ifdef SERIAL_DEBUGING
    Serial.println("R:" + String(r) + " G:" + String(g) + " B:" + String(b));
    Serial.print("zoom rgb: "); Serial.print(rgb);
    Serial.println("  Hex: " + String(rgb, HEX));
  #endif

  RGBEncoder[2].writeFadeRGB(0);
  RGBEncoder[2].writeRGBCode(rgb);
  lock_remote = false;
}

// ---------------------------- parameters handlers ----------------------------

void resetAperturePositionOSCHandler(OSCMessage &msg, int addrOffset) {
  int inValue = receiveOSCvalue(msg);

  #ifdef NEOPIXEL
    // TODO add global color
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
  #endif

  HOMING_POSITIONS[0] = inValue;

  lock_remote = false;
}

void resetFocusPositionOSCHandler(OSCMessage &msg, int addrOffset) {
  int inValue = receiveOSCvalue(msg);

  #ifdef NEOPIXEL
    // TODO add global color
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
  #endif

  HOMING_POSITIONS[1] = inValue;

  lock_remote = false;
}

void resetZoomPositionOSCHandler(OSCMessage &msg, int addrOffset) {
  int inValue = receiveOSCvalue(msg);

  #ifdef NEOPIXEL
    // TODO add global color
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
  #endif

  HOMING_POSITIONS[2] = inValue;

  lock_remote = false;
}

void setEncodersStepFineOSChandler(OSCMessage &msg, int addrOffset) {

  #ifdef SERIAL_DEBUGING
    Serial.println("Encoder fine steps: " + String(msg.getFloat(0)) + " " + String(msg.getFloat(1)) + " " +  String(msg.getFloat(2)));
  #endif

  for (int i=0; i<3; i++){
    potFineStep[i] = msg.getFloat(i);
    // update pot only if fine mode
    if(!toggle[i]){
      RGBEncoder[i].writeStep((int32_t) msg.getFloat(i));
    }
  }

  lock_remote = false;
}

void setEncodersStepCoarseOSChandler(OSCMessage &msg, int addrOffset) {

  #ifdef SERIAL_DEBUGING
    Serial.println("Encoder coarse steps: " + String(msg.getFloat(0)) + " " + String(msg.getFloat(1)) + " " +  String(msg.getFloat(2)));
  #endif

  for (int i=0; i<3; i++){
    potCoarseStep[i] = msg.getFloat(i);
    // update pot only if coarse mode
    if(toggle[i]){
      RGBEncoder[i].writeStep((int32_t) msg.getFloat(i));
    }
  }

  lock_remote = false;
}

void setEncodersMinOSChandler(OSCMessage &msg, int addrOffset) {
  // TODO check isadora sending int?
  int min1 = msg.getFloat(0);
  int min2 = msg.getFloat(1);
  int min3 = msg.getFloat(2);

  #ifdef SERIAL_DEBUGING
    Serial.println("Encoder coarse steps :" + String(min1) + " " + String(min2) + " " +  String(min3));
  #endif

  // RGBEncoder[enc_cnt].writeMax((int32_t) potMax[enc_cnt]); //Set the maximum threshold to
  RGBEncoder[0].writeMin((int32_t) min1); //Set the minimum threshold to
  RGBEncoder[1].writeMin((int32_t) min2); //Set the minimum threshold to
  RGBEncoder[2].writeMin((int32_t) min3); //Set the minimum threshold to

  lock_remote = false;
}

void setEncodersMaxOSChandler(OSCMessage &msg, int addrOffset) {
  // TODO check isadora sending int?
  int max1 = msg.getFloat(0);
  int max2 = msg.getFloat(1);
  int max3 = msg.getFloat(2);

  #ifdef SERIAL_DEBUGING
    Serial.println("Encoder coarse steps :" + String(max1) + " " + String(max2) + " " +  String(max3));
  #endif

  RGBEncoder[0].writeMax((int32_t) max1); //Set the maximum threshold to
  RGBEncoder[1].writeMax((int32_t) max2); //Set the maximum threshold to
  RGBEncoder[2].writeMax((int32_t) max3); //Set the maximum threshold to

  lock_remote = false;
}

// ------------------------------- other handlers ------------------------------


void resetMotorsPositions(){

  for (int i=0; i<3; i++){
    homeing[i] = true;
    moveMotorToPosition(i, HOMING_POSITIONS[i]);
  }
}

void resetOSChandler(OSCMessage &msg, int addrOffset) {
  int inValue = receiveOSCvalue(msg);

  #ifdef NEOPIXEL
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
  #endif

  resetMotorsPositions();
}

void brightnessHandler(OSCMessage &msg, int addrOffset) {
  brightness = msg.getFloat(0);

  #ifdef SERIAL_DEBUGING
    Serial.print("received brightness value: ");
    Serial.println(brightness);
  #endif

  lock_remote = false;
}

void setIntervalOSChandler(OSCMessage &msg, int addrOffset) {
  int inValue = receiveOSCvalue(msg);
  // NOTE hard limit for max frequency in ms
  if (inValue < 50) {interval = 50;}
  else {interval = inValue;}

  lock_remote = false;
}
//------------------------------------------------------------------------------

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

      lock_remote = true;

      // block osc messages when at least on motor is homeing
      if(!homeing[0] && !homeing[1] && !homeing[2]){
        msgIn.route("/resetPosition/aperture", resetAperturePositionOSCHandler);
        msgIn.route("/resetPosition/focus", resetFocusPositionOSCHandler);
        msgIn.route("/resetPosition/zoom", resetZoomPositionOSCHandler);

        msgIn.route("/aperture", apertureMoveToOSChandler);
        msgIn.route("/focus", focusMoveToOSChandler);
        msgIn.route("/zoom", zoomMoveToOSChandler);

        msgIn.route("/reset", resetOSChandler);
      }

      msgIn.route("/ledAperture", apertureLedOSChandler);
      msgIn.route("/ledFocus", focusLedOSChandler);
      msgIn.route("/ledZoom", zoomLedOSChandler);

      msgIn.route("/brightness", brightnessHandler);

      msgIn.route("/set/encoders/fine", setEncodersStepFineOSChandler);
      msgIn.route("/set/encoders/coarse", setEncodersStepCoarseOSChandler);

      msgIn.route("/set/encoders/min", setEncodersMinOSChandler);
      msgIn.route("/set/encoders/max", setEncodersMaxOSChandler);

      msgIn.route("/set/interval", setIntervalOSChandler);

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

  // #ifdef SERIAL_DEBUGING
  //   Serial.print("OSC header: ");
  //   Serial.println(message_osc_header);
  // #endif

  OSCMessage message(message_osc_header);
  message.add(value);
  Udp.beginPacket(targetIP, destPort);
  message.send(Udp);
  Udp.endPacket();
  message.empty();
}

void sendOSCreport(){
  // #ifdef SERIAL_DEBUGING
  //   Serial.print("Sending OSC raport ");
  // #endif
  // TODO fix sending -256 values when remote disconnected
  sendOSCmessage("/aperture", -stepper[0]->currentPosition());
  sendOSCmessage("/focus", -stepper[1]->currentPosition());
  sendOSCmessage("/zoom", -stepper[2]->currentPosition());
  sendOSCmessage("/uptime", uptimeInSecs());
  sendOSCmessage("/ver", FIRMWARE_VERSION);
  #ifdef SERIAL_DEBUGING
    Serial.print(" *");
  #endif
}

void sendOSCbundleReport(){
  //declare the bundle
  OSCBundle bndl;

  // char message_osc_header[32];
  // message_osc_header[0] = {0};
  // strcat(message_osc_header, osc_prefix);
  // strcat(message_osc_header, name);

  char message_osc_header_msg[32];
  message_osc_header_msg[0] = {0};
  strcat(message_osc_header_msg, osc_prefix);
  strcat(message_osc_header_msg, "/positions");
  bndl.add(message_osc_header_msg).add(-stepper[0]->currentPosition()).add(-stepper[1]->currentPosition()).add(-stepper[2]->currentPosition());

  message_osc_header_msg[0] = {0};
  strcat(message_osc_header_msg, osc_prefix);
  strcat(message_osc_header_msg, "/encoders/fine");
  bndl.add(message_osc_header_msg).add(potFineStep[0]).add(potFineStep[1]).add(potFineStep[2]);

  message_osc_header_msg[0] = {0};
  strcat(message_osc_header_msg, osc_prefix);
  strcat(message_osc_header_msg, "/encoders/coarse");
  bndl.add(message_osc_header_msg).add(potCoarseStep[0]).add(potCoarseStep[1]).add(potCoarseStep[2]);
  // bndl.add("/encoders/min").add();
  // bndl.add("/encoders/max").add();
  message_osc_header_msg[0] = {0};
  strcat(message_osc_header_msg, osc_prefix);
  strcat(message_osc_header_msg, "/interval");
  bndl.add(message_osc_header_msg).add(interval);

  message_osc_header_msg[0] = {0};
  strcat(message_osc_header_msg, osc_prefix);
  strcat(message_osc_header_msg, "/uptime");
  bndl.add(message_osc_header_msg).add(uptimeInSecs());

  message_osc_header_msg[0] = {0};
  strcat(message_osc_header_msg, osc_prefix);
  strcat(message_osc_header_msg, "/ver");
  bndl.add(message_osc_header_msg).add(FIRMWARE_VERSION);

  // bndl.add("/aperture").add(-stepper[0]->currentPosition());
  // bndl.add("/focus").add(-stepper[1]->currentPosition());
  // bndl.add("/zoom").add(-stepper[2]->currentPosition());
  // bndl.add("/digital/5").add((digitalRead(5)==HIGH)?"HIGH":"LOW");
  // bndl.add("/mouse/step").add((int32_t)analogRead(0)).add((int32_t)analogRead(1));
  // bndl.add("/units").add("pixels");

  Udp.beginPacket(targetIP, destPort);
  bndl.send(Udp); // send the bytes to the SLIP stream
  Udp.endPacket(); // mark the end of the OSC Packet
  bndl.empty(); // empty the bundle to free room for a new one
  #ifdef SERIAL_DEBUGING
    Serial.print(" *");
  #endif
}

bool checkEthernetConnection(){
  // // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    #ifdef SERIAL_DEBUGING
      // Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    #endif

    #ifdef NEOPIXEL
      pixels.setPixelColor(0, pixels.Color(150, 0, 0));
      pixels.show();
    #endif

    return false;
  }
  else if (Ethernet.linkStatus() == LinkOFF) {
    #ifdef SERIAL_DEBUGING
      // Serial.println("Ethernet cable is not connected.");
    #endif

    #ifdef NEOPIXEL
      pixels.setPixelColor(0, pixels.Color(150, 0, 0));
      pixels.show();
    #endif

    return false;
  }
  else if (Ethernet.linkStatus() == LinkON) {
    #ifdef SERIAL_DEBUGING
      // Serial.println("Ethernet cable is connected.");
    #endif

    // #ifdef NEOPIXEL
    //   pixels.setPixelColor(0, pixels.Color(0, 0, 150));
    //   pixels.show();
    // #endif

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

    // add delay when serial connected to catch first logs
    if (!Serial){ delay(3000); };
  #endif

  #ifdef SERIAL_DEBUGING
    Serial.print("\r\nFirmware Ver: "); Serial.print(FIRMWARE_VERSION);
    Serial.println(" written by Grzegorz Zajac");
    Serial.println("Compiled: " __DATE__ ", " __TIME__ ", " __VERSION__);
    Serial.print("Device ID "); Serial.println(device_id);
    Serial.println();
  #endif

  // Check if encoders are connected, only on startup, not hotpluging yet
  pinMode(POT_CHECK, INPUT_PULLUP); // LOW when remote is connected
  remote_connected = !digitalRead(POT_CHECK);

//-------------------------- Initializing encoders -----------------------------
  #ifdef SERIAL_DEBUGING
    Serial.println("initializing encoders");
  #endif

  uint8_t enc_cnt;
  pinMode(INT_PIN, INPUT);

  if (remote_connected){
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
      RGBEncoder[enc_cnt].writeMax((int32_t) potMax[enc_cnt]); //Set the maximum threshold to
      RGBEncoder[enc_cnt].writeMin((int32_t) potMin[enc_cnt]); //Set the minimum threshold to
      RGBEncoder[enc_cnt].writeStep((int32_t) potCoarseStep[enc_cnt]); //The step at every encoder click is 1
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
  }


//-------------------------- Initializing steppers -----------------------------
  #ifdef SERIAL_DEBUGING
    Serial.println("initializing steppers");
  #endif

  // exprimental settings, speed for manual adjustment quick response
  stepper[0]->setMaxSpeed(APERTURE_SPEED);
  stepper[0]->setAcceleration(APERTURE_ACCELERATION);

  stepper[1]->setMaxSpeed(FOCUS_SPEED);
  stepper[1]->setAcceleration(FOCUS_ACCELERATION);

  stepper[2]->setMaxSpeed(ZOOM_SPEED);
  stepper[2]->setAcceleration(ZOOM_ACCELERATION);

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

  char id[8];
  sprintf(id, "%d", device_id);
  strcat(osc_prefix, id);

  // Bonjour name
  char bonjour_name[8] = {0};                 // host name i.e camera1, used for address instead of IP -> used with .local i.e camera1.local
  strcat(bonjour_name, "camera");
  strcat(bonjour_name, id);

  #ifdef WEB_SERVER
    // decelared globally for accesing in loop in web site
    strcat(web_address, bonjour_name);
    strcat(web_address, ".local");
  #endif

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
    Serial.print("Bonjour name: ");
    Serial.println(bonjour_name);
    #ifdef WEB_SERVER
      Serial.print("Web refresh link: ");
      Serial.println(web_address);
    #endif
    Serial.println();
    Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
  #endif


  // Initializing EthernetBonjour
  EthernetBonjour.begin(bonjour_name);

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


//=================================== LOOP =====================================

void loop() {
  isLANconnected = checkEthernetConnection();

  if (isLANconnected){
    EthernetBonjour.run();
    receiveOSCsingle();
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (isLANconnected){

      #ifdef NEOPIXEL
        pixels.setPixelColor(0, pixels.Color(0, 255, 150));
        pixels.show();
      #endif

      // sendOSCreport();
      sendOSCbundleReport();

      #ifdef NEOPIXEL
        pixels.setPixelColor(0, pixels.Color(0, 0, 150));
        pixels.show();
      #endif
    }
  }

  // check pots
  uint8_t enc_cnt;

  if (remote_connected && !lock_remote){
    if (digitalRead(INT_PIN) == LOW) {
      //Interrupt from the encoders, start to scan the encoder matrix
      for (enc_cnt = 0; enc_cnt < ENCODER_N; enc_cnt++) {
        if (digitalRead(INT_PIN) == HIGH) { //If the interrupt pin return high, exit from the encoder scan
          break;
        }
        RGBEncoder[enc_cnt].updateStatus();
      }
    }
  }

  for (int i=0; i<3; i++){
    stepper[i]->run();

    if(homeing[i] && (-stepper[i]->currentPosition() == HOMING_POSITIONS[i]) ){
        stepper[i]->setCurrentPosition(0);
        RGBEncoder[i].writeCounter((int32_t) 0); //Reset of the CVAL register
        #ifdef SERIAL_DEBUGING
          Serial.print("stepper "); Serial.print(i); Serial.println(" is at home position");
        #endif

        homeing[i] = false;

        // if all restarted then unlock remote
        if ( !homeing[0] && !homeing[1] && !homeing[2] ){
          Serial.println("All motors reset to 0");
          lock_remote = false;
        }
    }

    // #ifdef NEOPIXEL
    //   pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    //   pixels.show();
    // #endif

    // TODO where to set homing to false, is it necessery?
  }

  #ifdef WEB_SERVER
  // TODO add dynamic IP

  // Create a client connection
  EthernetClient client = server.available();
  if (client) {
    while (client.connected()) {
      if (client.available()) {
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
           client.println("Connection: close");
           client.print("Refresh: 3;URL='//");
           client.print(web_address);
           client.println("/'");
           client.println();
           client.println("<!DOCTYPE HTML>");
           client.println("<HTML>");
           client.println("<HEAD>");
           client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
           client.println("<TITLE>Camera Lens Controller</TITLE>");
           client.println("</HEAD>");
           client.println("<BODY>");
           client.println("<H3>SSP Camera Lens controler</H3>");
           // client.print(device_id); client.println("</h4>");
           // NOTE toggle button?
           client.print("<a href=\"/?buttonIDclicked\"\"><button class=\"button\" type='button'>Identify Device ");
           client.print(device_id);
           client.println("</button></a>");
           client.print("<a href=\"/?buttonResetclicked\"\"><button class=\"button\" type='button'>Reset ");
           client.print(device_id);
           client.println("</button></a>");
           client.println("<br />");
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
           client.println("Remote connected: ");
           client.println(remote_connected);
           client.println("<br />");


           client.println("<ul>");
             client.println("<li>");
             client.print("Aperture position: "); client.println(-stepper[0]->currentPosition());
             client.println("</li>");
             client.println("<li>");
             client.print("Focus position: "); client.println(-stepper[1]->currentPosition());
             client.println("</li>");
             client.println("<li>");
             client.print("Zoom position: "); client.println(-stepper[2]->currentPosition());
             client.println("</li>");
           client.println("</ul>");

           client.println("<a href=\"/?buttonA0clicked\"\"><button class=\"button\" type='button'>Apperture @ 0</button></a>");
           client.println("<a href=\"/?buttonA1000clicked\"\"><button class=\"button\" type='button'>Apperture @");
           client.print(button1value);
           client.println("</button></a>");
           client.println("<br />");

           client.println("<a href=\"/?buttonF0clicked\"\"><button class=\"button\" type='button'>Focus @ 0</button></a>");
           client.println("<a href=\"/?buttonF1000clicked\"\"><button class=\"button\" type='button'>Focus @");
           client.print(button2value);
           client.println("</button></a>");
           client.println("<br />");

           client.println("<a href=\"/?buttonZ0clicked\"\"><button class=\"button\" type='button'>Zoom @ 0</button></a>");
           client.println("<a href=\"/?buttonZ1000clicked\"\"><button class=\"button\" type='button'>Zoom @");
           client.print(button3value);
           client.println("</button></a>");
           client.println("<br />");

           client.println("</BODY>");
           client.println("</HTML>");

           client.println("<style type='text/css'>");
             client.println("body {background-color: #222222; color: #fefefe;}");
             // client.println("h3 {color: #104bab}");
             client.println("h4 {color: rgb(255, 255, 255)}");
             client.println(".button { background-color: #104bab; color: white; border: none; border-radius: 4px; display: inline-block;");
             client.println("text-decoration: none; margin: 2px; padding: 14px 10px; width: 40%; cursor: pointer;}");
             // client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
           client.println("</style>");

           delay(1);
           //stopping client
           client.stop();
           // client.flush();
           //controls the Arduino if you press the buttons

           if (readString.indexOf("?buttonA0clicked") > 0 ){
             #ifdef SERIAL_DEBUGING
               Serial.println("Web button pressed, setting aperture to 0");
             #endif
             if (remote_connected){
               RGBEncoder[0].writeCounter((int32_t) 0); //Reset of the CVAL register
             }
             moveMotorToPosition(0, 0);
           }
           if (readString.indexOf("?buttonF0clicked") > 0){
             #ifdef SERIAL_DEBUGING
               Serial.println("Web button pressed, setting focus to 0");
             #endif
             if (remote_connected){
               RGBEncoder[1].writeCounter((int32_t) 0); //Reset of the CVAL register
             }
             moveMotorToPosition(1, 0);
           }
           if (readString.indexOf("?buttonZ0clicked") > 0){
             #ifdef SERIAL_DEBUGING
               Serial.println("Web button pressed, setting zoom to 0");
             #endif
             if (remote_connected){
               RGBEncoder[2].writeCounter((int32_t) 0); //Reset of the CVAL register
             }
             moveMotorToPosition(2, 0);
           }
           if (readString.indexOf("?buttonA1000clicked") > 0 ){
             #ifdef SERIAL_DEBUGING
               Serial.print("Web button pressed, setting focus to "); Serial.println(button1value);
             #endif
             if (remote_connected){
               RGBEncoder[0].writeCounter((int32_t) button1value); //Reset of the CVAL register
             }
             moveMotorToPosition(0, button1value);
           }
           if (readString.indexOf("?buttonF1000clicked") > 0){
             #ifdef SERIAL_DEBUGING
               Serial.print("Web button pressed, setting focus to "); Serial.println(button2value);
             #endif
             if (remote_connected){
               RGBEncoder[1].writeCounter((int32_t) button2value); //Reset of the CVAL register
             }
             moveMotorToPosition(1, button2value);
           }
           if (readString.indexOf("?buttonZ1000clicked") > 0){
             #ifdef SERIAL_DEBUGING
               Serial.print("Web button pressed, setting focus to "); Serial.println(button3value);
             #endif
             if (remote_connected){
               RGBEncoder[2].writeCounter((int32_t) button3value); //Reset of the CVAL register
             }
             moveMotorToPosition(2, button2value);
           }
           if (readString.indexOf("?buttonIDclicked") > 0){
             #ifdef SERIAL_DEBUGING
               Serial.println("Web button pressed, turning neopixel white");
             #endif
             #ifdef NEOPIXEL
               pixels.setPixelColor(0, pixels.Color(255, 255, 255));
               pixels.show();
             #endif
           }
           if (readString.indexOf("?buttonResetclicked") > 0){
             #ifdef SERIAL_DEBUGING
               Serial.println("reset button pressed, homeing motors");
             #endif
             resetMotorsPositions();
           }


            //clearing string for next read
            readString="";

         }
       }
    }
  }                      			   // start to listen for clients
  #endif

} // end of loop
