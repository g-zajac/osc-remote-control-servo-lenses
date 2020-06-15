#define FIRMWARE_VERSION 213

// device_id, numer used a position in array to get last octet of MAC and static IP
// prototype 0, unit 1, unit 2... unit 7.
#define DEVICE_ID 1

// Enable/Disable modules
#define SERIAL_DEBUGING
#define SERIAL_SPEED 115200

// pins definition
#define LED_PIN 2


//------------------------------------------------------------------------------
#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
// #include <OSCBundle.h>
#include <OSCMessage.h>

#include <Wire.h>
#include <i2cEncoderLibV2.h>

//------------------------------ I2C encoders ----------------------------------
// Connections:
// - -> GND
// + -> 5V
// SDA -> A4
// SCL -> A5
// INT -> 3 temporary for tests

const int IntPin = 3; /* Definition of the interrupt pin. You can change according to your board /*
//Class initialization with the I2C addresses*/
i2cEncoderLibV2 Encoder(0x01); /* A0 is soldered */

//---------------------------- MAC & IP list ----------------------------------
byte MAC_ARRAY[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
int IP_ARRAY[] = {240, 241, 242, 243, 244, 245, 246, 247};
//-----------------------------------------------------------------------------

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, MAC_ARRAY[DEVICE_ID]
};
IPAddress ip(10, 0, 10, IP_ARRAY[DEVICE_ID]);

// Networking / UDP Setup for OSC
EthernetUDP Udp;

// OSC destination address
IPAddress targetIP(10, 0, 10, 101);   // Isadora machine IP address
const unsigned int destPort = 9999;          // remote port to receive OSC
const unsigned int localPort = 8888;        // local port to listen for OSC packets

unsigned long previousMillis = 0;
const long interval = 500;
long uptime = 0;

char osc_prefix[16];                  // device OSC prefix message, i.e /camera1


//***************************** Functions *************************************
int uptimeInSecs(){
  return (int)(millis()/1000);
}

// TODO remove, redundant, no led indicator? link can be checked on ether socket
void checkConnectin(){
  if (Ethernet.linkStatus() == Unknown) {
    Serial.println("Link status unknown. Link status detection is only available with W5200 and W5500.");
    digitalWrite(LED_PIN, HIGH);
  }
  else if (Ethernet.linkStatus() == LinkON) {
    Serial.print("Link status: On, connected with IP: ");
    Serial.println(Ethernet.localIP());
    digitalWrite(LED_PIN, LOW);
  }
  else if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Link status: Off");
    digitalWrite(LED_PIN, HIGH);
  }
}

void servo1_OSCHandler(OSCMessage &msg, int addrOffset) {
  int inValue = msg.getFloat(0);
  #ifdef SERIAL_DEBUGING
    Serial.print("osc servo 1 update: ");
    Serial.println(inValue);
  #endif
}

void servo2_OSCHandler(OSCMessage &msg, int addrOffset) {
  int inValue = msg.getFloat(0);
  #ifdef SERIAL_DEBUGING
    Serial.print("osc servo 2 update: ");
    Serial.println(inValue);
  #endif
}

void servo3_OSCHandler(OSCMessage &msg, int addrOffset) {
  int inValue = msg.getFloat(0);
  #ifdef SERIAL_DEBUGING
    Serial.print("osc servo 3 update: ");
    Serial.println(inValue);
  #endif
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
      msgIn.route("/device1/servo/1", servo1_OSCHandler);
      msgIn.route("/device1/servo/2", servo2_OSCHandler);
      msgIn.route("/device1/servo/3", servo3_OSCHandler);
      // msgIn.route("/device1/localise", localise_OSCHandler);
    }

    //finish reading this packet:
    Udp.flush();
    //restart UDP connection to receive packets from other clients
    Udp.stop();
    Udp.begin(localPort);
  }
}

void sendOSCmessage(char* name, int value){
  char message_osc_header[16];
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
  sendOSCmessage("/ver", FIRMWARE_VERSION);
  sendOSCmessage("/uptime", uptimeInSecs());
}

//******************************************************************************

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  //NOTE on boot the led inidicate power, once connects with ethernet goes off

  #ifdef SERIAL_DEBUGING
    Serial.begin(SERIAL_SPEED);
    while (!Serial) {
      ; //TODO remove for production, debuging only, wait for serial port to connect. Needed for native USB port only
    }
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
  pinMode(IntPin, INPUT);
  Wire.begin();

  Encoder.reset();
  Encoder.begin(
    i2cEncoderLibV2::INT_DATA | i2cEncoderLibV2::WRAP_DISABLE
    | i2cEncoderLibV2::DIRE_LEFT | i2cEncoderLibV2::IPUP_ENABLE
    | i2cEncoderLibV2::RMOD_X1 | i2cEncoderLibV2::RGB_ENCODER);

  // Use this in case of standard encoder!
  //  Encoder.begin(i2cEncoderLibV2::INT_DATA | i2cEncoderLibV2::WRAP_DISABLE | i2cEncoderLibV2::DIRE_LEFT | i2cEncoderLibV2::IPUP_ENABLE | i2cEncoderLibV2::RMOD_X1 | i2cEncoderLibV2::STD_ENCODER);

  // try also this!
  //  Encoder.begin(i2cEncoderLibV2::INT_DATA | i2cEncoderLibV2::WRAP_ENABLE | i2cEncoderLibV2::DIRE_LEFT | i2cEncoderLibV2::IPUP_ENABLE | i2cEncoderLibV2::RMOD_X1 | i2cEncoderLibV2::RGB_ENCODER);

  Encoder.writeCounter((int32_t) 0); /* Reset the counter value */
  Encoder.writeMax((int32_t) 10); /* Set the maximum threshold*/
  Encoder.writeMin((int32_t) - 10); /* Set the minimum threshold */
  Encoder.writeStep((int32_t) 1); /* Set the step to 1*/
  Encoder.writeInterruptConfig(0xff); /* Enable all the interrupt */
  Encoder.writeAntibouncingPeriod(20); /* Set an anti-bouncing of 200ms */
  Encoder.writeDoublePushPeriod(50); /*Set a period for the double push of 500ms */

//-------------------------- Initializing ethernet -----------------------------
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);    // begin reset the WIZ820io
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);  // de-select WIZ820io
  digitalWrite(9, HIGH);   // end reset pulse

  Ethernet.init(10);

  // Initialize the Ethernet server library
  // with the IP address and port you want to use
  // (port 80 is default for HTTP):
  // EthernetServer server(80);

  // start the Ethernet connection
  Ethernet.begin(mac, ip);

  // // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    #ifdef SERIAL_DEBUGING
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    #endif
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    #ifdef SERIAL_DEBUGING
      Serial.println("Ethernet cable is not connected.");
    #endif
    digitalWrite(LED_PIN, HIGH);
  }

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

  // start the server
  // server.begin();
  // Serial.print("server is at ");
  // Serial.println(Ethernet.localIP());

  //TODO add check connected status if
  Udp.begin(localPort);
}

void loop() {
  receiveOSCsingle();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sendOSCreport();
  }


  if (digitalRead(IntPin) == LOW) {
    if (Encoder.updateStatus()) {
      if (Encoder.readStatus(i2cEncoderLibV2::RINC)) {
        Serial.print("Increment: ");
        Serial.println(Encoder.readCounterByte());

        /* Write here your code */

      }
      if (Encoder.readStatus(i2cEncoderLibV2::RDEC)) {
        Serial.print("Decrement: ");
        Serial.println(Encoder.readCounterByte());

        /* Write here your code */

      }

      if (Encoder.readStatus(i2cEncoderLibV2::RMAX)) {
        Serial.print("Maximum threshold: ");
        Serial.println(Encoder.readCounterByte());

        /* Write here your code */

      }

      if (Encoder.readStatus(i2cEncoderLibV2::RMIN)) {
        Serial.print("Minimum threshold: ");
        Serial.println(Encoder.readCounterByte());

        /* Write here your code */

      }

      if (Encoder.readStatus(i2cEncoderLibV2::PUSHR)) {
        Serial.println("Push button Released");

        /* Write here your code */

      }

      if (Encoder.readStatus(i2cEncoderLibV2::PUSHP)) {
        Serial.println("Push button Pressed");

        /* Write here your code */

      }

      if (Encoder.readStatus(i2cEncoderLibV2::PUSHD)) {
        Serial.println("Double push!");

        /* Write here your code */

      }
    }
  }

}
