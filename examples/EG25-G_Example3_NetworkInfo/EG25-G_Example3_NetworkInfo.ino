/*

  EG25-G Example
  ===============

  Network Info
  
  Written by: Paul Clark
  Date: November 18th 2020

  This example demonstrates how to register the EG25 on the selected network.

  Feel like supporting open source hardware?
  Buy a board from Firechip!

  Licence: MIT
  Please see LICENSE.md for full details

*/

#include <FC0003390327.h> //Click here to get the library: http://librarymanager/All#Firechip_Quectel_EG25-G_Arduino_Library

// Uncomment the next line to connect to the EG25-G using hardware Serial1
#define atSerial Serial1

// Uncomment the next line to create a SoftwareSerial object to pass to the EG25-G library instead
//SoftwareSerial atSerial(8, 9);

// Create a EG25_G object to use throughout the sketch
// Usually we would tell the library which GPIO pin to use to control the EG25 power (see below),
// but we can start the EG25 without a power pin. It just means we need to manually 
// turn the power on if required! ;-D
EG25_G myEG25;

// Create a EG25_G object to use throughout the sketch
// We need to tell the library what GPIO pin is connected to the EG25 power pin.
// If you're using the MicroMod Asset Tracker and the MicroMod Artemis Processor Board,
// the pin name is G2 which is connected to pin AD34.
// Change the pin number if required.
//EG25_G myEG25(34);

// Map registration status messages to more readable strings
String registrationString[] =
{
  "Not registered",                         // 0
  "Registered, home",                       // 1
  "Searching for operator",                 // 2
  "Registration denied",                    // 3
  "Registration unknown",                   // 4
  "Registered, roaming",                    // 5
  "Registered, home (SMS only)",            // 6
  "Registered, roaming (SMS only)",         // 7
  "Registered, emergency service only",     // 8
  "Registered, home, CSFB not preferred",   // 9
  "Registered, roaming, CSFB not preferred"  // 10
};

// Network operator can be set to e.g.:
// MNO_SW_DEFAULT -- DEFAULT (Undefined / regulatory)
// MNO_SIM_ICCID -- SIM DEFAULT
// MNO_ATT -- AT&T 
// MNO_VERIZON -- Verizon
// MNO_TELSTRA -- Telstra
// MNO_TMO -- T-Mobile US
// MNO_CT -- China Telecom
// MNO_SPRINT
// MNO_VODAFONE
// MNO_NTT_DOCOMO
// MNO_TELUS
// MNO_SOFTBANK
// MNO_DT -- Deutsche Telekom
// MNO_US_CELLULAR
// MNO_SKT
// MNO_GLOBAL -- EG25 factory-programmed value
// MNO_STD_EUROPE
// MNO_STD_EU_NOEPCO

// If you are based in Europe, you will (probably) need to select MNO_STD_EUROPE

const mobile_network_operator_t MOBILE_NETWORK_OPERATOR = MNO_GLOBAL;

void setup()
{
  Serial.begin(115200); // Start the serial console

  // Wait for user to press key to begin
  Serial.println(F("EG25-G Example"));
  Serial.println(F("Press any key to begin"));
  
  while (!Serial.available()) // Wait for the user to press a key (send any serial character)
    ;
  while (Serial.available()) // Empty the serial RX buffer
    Serial.read();

  //myEG25.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  // For the MicroMod Asset Tracker, we need to invert the power pin so it pulls high instead of low
  // Comment the next line if required
  myEG25.invertPowerPin(true); 

  // Initialize the EG25
  if (myEG25.begin(atSerial, 9600) )
  {
    Serial.println(F("EG25-G connected!"));
  }
  else
  {
    Serial.println(F("Unable to communicate with the EG25."));
    Serial.println(F("Manually power-on (hold the EG25 On button for 3 seconds) on and try again."));
    while (1) ; // Loop forever on fail
  }
  Serial.println();

  if (!myEG25.setNetworkProfile(MOBILE_NETWORK_OPERATOR))
  {
    Serial.println(F("Error setting network. Try cycling the power."));
    while (1) ;
  }
  
  Serial.println(F("Network profile set. Ready to go!"));
  
  // RSSI: Received signal strength:
  Serial.println("RSSI: " + String(myEG25.rssi()));
  // Registration Status
  int regStatus = myEG25.registration();
  if ((regStatus >= 0) && (regStatus <= 10))
  {
    Serial.println("Network registration: " + registrationString[regStatus]);
  }

  // Print the Context IDs, Access Point Names and IP Addresses
  Serial.println(F("Available PDP (Packet Data Protocol) APNs (Access Point Names) and IP Addresses:"));
  Serial.println(F("Context ID:\tAPN Name:\tIP Address:"));
  for (int cid = 0; cid < EG25_G_NUM_PDP_CONTEXT_IDENTIFIERS; cid++)
  {
    String apn = "";
    IPAddress ip(0, 0, 0, 0);
    myEG25.getAPN(cid, &apn, &ip);
    if (apn.length() > 0)
    {
      Serial.print(cid);
      Serial.print(F("\t"));
      Serial.print(apn);
      Serial.print(F("\t"));
      Serial.println(ip);
    }
  }

  Serial.println();
  
  if (regStatus > 0)
  {
    Serial.println(F("All set. Go to the next example!"));
  }
}

void loop()
{
  // Do nothing. Now that we're registered move on to the next example.
}
