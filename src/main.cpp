// #define DEBUG_HARD_SERIAL

#include <Arduino.h>
#include <UIPEthernet.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <Servo.h>
#include <Encoder.h>
// #include <Adafruit_NeoPixel.h>

// Networking / UDP Setup
EthernetUDP Udp;

// destination address, Isadora machine, set last octet to 255 for broadcat - send to all
IPAddress targetIP(10, 0, 10, 101);
const unsigned int targetPort = 9999;
const unsigned int inPort = 8888;

// NOTE change in production with multiple units
byte mac[] = { 0x54, 0x34, 0x41, 0x30, 0x30, 0x31 };

// refresh, update timing to send osc ping, position msg
unsigned long previousMillis = 0;
const long interval = 1000;

const int servoPin = 2;
// Create a servo object
Servo Servo1;
// TODO add global var servo_position

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
int knob_position  = 0;

// #define NEOPIXEL_PIN 11
// #define NUMPIXELS 1

// #include <Adafruit_NeoPixel.h>
// #ifdef __AVR__
//   #include <avr/power.h>
// #endif

// Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// # define LED_PIN 13
//NOTE colison with ethernet?

// void printIPAddress()
// {
//   #ifdef DEBUG_HARD_SERIAL
//   Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-\n");
//   Serial.print("IP Address        : ");
//   Serial.println(Ethernet.localIP());
//   Serial.print("Subnet Mask       : ");
//   Serial.println(Ethernet.subnetMask());
//   Serial.print("Default Gateway IP: ");
//   Serial.println(Ethernet.gatewayIP());
//   Serial.print("DNS Server IP     : ");
//   Serial.println(Ethernet.dnsServerIP());
//   Serial.println();
//   #endif
// }

void servoOSCHandler(OSCMessage &msg, int addrOffset) {
  int inValue = msg.getFloat(0);
  #ifdef DEBUG_HARD_SERIAL
    Serial.print("osc msg value: ");
    Serial.println(inValue);
  #endif
  Servo1.write(inValue);
    // NOTE send OSC feedback with new position?

}

// the setup function runs once when you press reset or power the board
void setup() {
  #ifdef DEBUG_HARD_SERIAL
    Serial.begin(9600);
  #endif

  // #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  //   clock_prescale_set(clock_div_1);
  // #endif

  // pixels.begin();
  // pixels.clear();

  // initialize digital pin LED_PIN as an output.
  // pinMode(LED_PIN, OUTPUT);
  // digitalWrite(LED_PIN, HIGH);
  // delay(1000);
  // digitalWrite(LED_PIN, LOW);
  // delay(1000);
  // digitalWrite(LED_PIN, HIGH);

  // pixels.setPixelColor(0, pixels.Color(50, 0, 0));
  // pixels.show();

  Servo1.attach(servoPin);

  // start the Ethernet connection:
  if (Ethernet.begin(mac) == 0) {
    #ifdef DEBUG_HARD_SERIAL
      Serial.println("Failed to configure Ethernet using DHCP");
    #endif
    // no point in carrying on, so do nothing forevermore:
    for (;;);
    }
    // print your local IP address:
    // printIPAddress();
    #ifdef DEBUG_HARD_SERIAL
      Serial.println("Connected");
    #endif

    Udp.begin(inPort);

    // pixels.setPixelColor(0, pixels.Color(0, 0, 30));
    // pixels.show();
}


void loop() {

  // ethernet
  switch (Ethernet.maintain())
   {
     case 1:
       //renewed fail
       #ifdef DEBUG_HARD_SERIAL
        Serial.println("Error: renewed fail");
      #endif
      // digitalWrite(LED_PIN, LOW);
      // pixels.setPixelColor(0, pixels.Color(50, 0, 0));
      // pixels.show();
      break;

     case 2:
      //renewed success
      #ifdef DEBUG_HARD_SERIAL
        Serial.println("Renewed success");
      #endif
      // digitalWrite(LED_PIN, HIGH);
      // pixels.setPixelColor(0, pixels.Color(0, 0, 30));
      // pixels.show();
      //print your local IP address:
      // printIPAddress();
      break;

     case 3:
      //rebind fail
      #ifdef DEBUG_HARD_SERIAL
        Serial.println("Error: rebind fail");
      #endif
      // digitalWrite(LED_PIN, LOW);
      // pixels.setPixelColor(0, pixels.Color(50, 0, 0));
      // pixels.show();
      break;

     case 4:
      //rebind success
      #ifdef DEBUG_HARD_SERIAL
        Serial.println("Rebind success");
      #endif
      // digitalWrite(LED_PIN, HIGH);
      // pixels.setPixelColor(0, pixels.Color(0, 0, 30));
      // pixels.show();
      //print your local IP address:
      // printIPAddress();
      break;

     default:
       //nothing happened
       break;

   }

   // OSC
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
       msgIn.route("/servo", servoOSCHandler);
     }

     //finish reading this packet:
     Udp.flush();

     //restart UDP connection to receive packets from other clients
     Udp.stop();
     success = Udp.begin(inPort);

     // pixels.setPixelColor(0, pixels.Color(0, 0, 30));
     // pixels.show();
   }



   unsigned long currentMillis = millis();
   if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // pixels.setPixelColor(0, pixels.Color(0, 50, 0));
    // pixels.show();

    int uptime = (int)(millis()/1000);

    #ifdef DEBUG_HARD_SERIAL
      Serial.print("encoder position: "); Serial.println(knob_position / 4);
      Serial.print("sending uptime osc: "); Serial.println(uptime);
    #endif
    //TODO sending blocks receiving
    OSCMessage msgPing("/servo/uptime");
    msgPing.add(uptime);
    Udp.beginPacket(targetIP, targetPort);
    msgPing.send(Udp); // send the bytes to the SLIP stream
    Udp.endPacket(); // mark the end of the OSC Packet
    msgPing.empty(); // free space occupied by message

    #ifdef DEBUG_HARD_SERIAL
      Serial.print("sending servo position osc: "); Serial.println(Servo1.read());
    #endif

    OSCMessage msgPos("/servo/position");
    msgPos.add(Servo1.read());
    Udp.beginPacket(targetIP, targetPort);
    msgPos.send(Udp); // send the bytes to the SLIP stream
    Udp.endPacket(); // mark the end of the OSC Packet
    msgPos.empty(); // free space occupied by message

    // restart UDP connection so we are ready to accept incoming ports
    Udp.stop();
    Udp.begin(inPort);

    // pixels.setPixelColor(0, pixels.Color(0, 0, 30));
    // pixels.show();
  }

  // Encoder
  // counting the rise/fall from each pin, of which there are four in each notch of the encoder.
  // jumps by 4
  long knob_new_position;
  knob_new_position = knob.read();
  if (knob_new_position != knob_position){
    knob_position = knob_new_position;
    // set limists
    if (knob_position < 0) knob_position = 0;
    if (knob_position > 4*180) knob_position = 4*180;

    // update servo position
    //TODO sync position with current OSC position value
    Servo1.write(knob_position);
    //TODO add manual OSC flag
    // pixels.setPixelColor(0, pixels.Color(0, 0, 100));
    // pixels.show();
  };

}
