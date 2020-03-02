#include <Arduino.h>
#include <UIPEthernet.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <Servo.h>

// Networking / UDP Setup
EthernetUDP Udp;

// destination address
IPAddress targetIP(10, 0, 10, 101);
const unsigned int targetPort = 1234;
const unsigned int inPort = 8888;

byte mac[] = { 0x54, 0x34, 0x41, 0x30, 0x30, 0x31 };

unsigned long previousMillis = 0;
const long interval = 1000;
int counter = 0;  // for test only

int servoPin = 2;
// Create a servo object
Servo Servo1;

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
}

void servoOSCHandler(OSCMessage &msg, int addrOffset) {
  int inValue = msg.getFloat(0);

  Serial.print("osc msg value: ");
  Serial.println(inValue);
  Servo1.write(inValue);
}

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Servo1.attach(servoPin);

  //quick servo test
  // Servo1.write(0);
  // delay(1000);
  // Servo1.write(90);

  // start the Ethernet connection:
  if (Ethernet.begin(mac) == 0) {
      Serial.println("Failed to configure Ethernet using DHCP");
      // no point in carrying on, so do nothing forevermore:
      for (;;);
    }
    // print your local IP address:
    printIPAddress();

    Udp.begin(inPort);
}

// the loop function runs over and over again forever
void loop() {
  switch (Ethernet.maintain())
   {
     case 1:
       //renewed fail
       Serial.println("Error: renewed fail");
       break;

     case 2:
       //renewed success
       Serial.println("Renewed success");

       //print your local IP address:
       printIPAddress();
       break;

     case 3:
       //rebind fail
       Serial.println("Error: rebind fail");
       break;

     case 4:
       //rebind success
       Serial.println("Rebind success");

       //print your local IP address:
       printIPAddress();
       break;

     default:
       //nothing happened
       break;

   }

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

   }

   unsigned long currentMillis = millis();
   if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    Serial.print("sending test osc: "); Serial.println(counter);

    //TODO sending blocks receiving
    OSCMessage msg("/alive");
    msg.add(counter);
    Udp.beginPacket(targetIP, targetPort);
    msg.send(Udp); // send the bytes to the SLIP stream
    Udp.endPacket(); // mark the end of the OSC Packet
    msg.empty(); // free space occupied by message

    // restart UDP connection so we are ready to accept incoming ports
    Udp.stop();
    Udp.begin(inPort);
    counter++;
  }
}
