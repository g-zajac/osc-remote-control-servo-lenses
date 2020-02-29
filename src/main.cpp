#include <Arduino.h>
#include <UIPEthernet.h>
#include <OSCMessage.h>
#include <OSCBundle.h>

// Networking / UDP Setup
EthernetUDP Udp;

// destination address
IPAddress targetIP(10, 0, 10, 101);
const unsigned int targetPort = 8000;
const unsigned int inPort = 8888;

byte mac[] = { 0x54, 0x34, 0x41, 0x30, 0x30, 0x31 };

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

void pingOSCHandler(OSCMessage &msg, int addrOffset) {
  int inValue = msg.getInt(0);

  Serial.print("osc msg value: ");
  Serial.println(inValue);
}

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

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

       OSCMessage msg("/test");
       msg.add(100);
       Udp.beginPacket(targetIP, targetPort);
       msg.send(Udp); // send the bytes to the SLIP stream
       Udp.endPacket(); // mark the end of the OSC Packet
       msg.empty(); // free space occupied by message

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
       msgIn.route("/servo", pingOSCHandler);
     }

     //finish reading this packet:
     Udp.flush();

     //restart UDP connection to receive packets from other clients
     Udp.stop();
     success = Udp.begin(inPort);

   }

}
