#include <Arduino.h>
#include <UIPEthernet.h>

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
}
