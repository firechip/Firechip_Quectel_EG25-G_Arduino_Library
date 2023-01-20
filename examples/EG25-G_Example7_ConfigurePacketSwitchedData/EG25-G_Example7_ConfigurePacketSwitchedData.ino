/*

  EG25-G Example
  ===============

  Configure Packet Switched Data

  Written by: Paul Clark
  Date: November 18th 2020

  This example demonstrates how to configure Packet Switched Data on the EG25-G.

  The earlier examples let you configure the network profile and select an operator.
  The default operator - defined in your SIM - will be allocated to "Context ID 1".
  This example defines a Packet Switched Data Profile ID, based on the selected Context ID, and then activates it.
  The profile parameters are also saved to NVM so they can be used by the next examples.
  The only complicated bit is that - strictly - we need to disconnect from the network first in order to find out what
  the defined IP type is for the chosen Context ID - as opposed to what is granted by the network. However, we'll
  take a guess that it is the default (IPv4v6). You can change this if required by editing the call to setPDPconfiguration.

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

// processPSDAction is provided to the EG25-G library via a 
// callback setter -- setPSDActionCallback. (See setup())
void processPSDAction(int result, IPAddress ip)
{
  Serial.println();
  Serial.print(F("PSD Action:  result: "));
  Serial.print(String(result));
  if (result == 0)
    Serial.print(F(" (success)"));
  Serial.print(F("  IP Address: \""));
  Serial.print(String(ip[0]));
  Serial.print(F("."));
  Serial.print(String(ip[1]));
  Serial.print(F("."));
  Serial.print(String(ip[2]));
  Serial.print(F("."));
  Serial.print(String(ip[3]));
  Serial.println(F("\""));
}

void setup()
{
  String currentOperator = "";

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

  // First check to see if we're connected to an operator:
  if (myEG25.getOperator(&currentOperator) == EG25_G_SUCCESS)
  {
    Serial.print(F("Connected to: "));
    Serial.println(currentOperator);
  }
  else
  {
    Serial.print(F("The EG25 is not yet connected to an operator. Please use the previous examples to connect. Or wait and retry. Freezing..."));
    while (1)
      ; // Do nothing more
  }

  int minCID = EG25_G_NUM_PDP_CONTEXT_IDENTIFIERS; // Keep a record of the highest and lowest CIDs
  int maxCID = 0;

  Serial.println(F("The available Context IDs are:"));
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
    if (cid < minCID)
      minCID = cid; // Record the lowest CID
    if (cid > maxCID)
      maxCID = cid; // Record the highest CID
  }
  Serial.println();

  Serial.println(F("Which Context ID do you want to use for your Packet Switched Data connection?"));
  Serial.println(F("Please enter the number (followed by LF / Newline): "));
  
  char c = 0;
  bool selected = false;
  int selection = 0;
  while (!selected)
  {
    while (!Serial.available()) ; // Wait for a character to arrive
    c = Serial.read(); // Read it
    if (c == '\n') // Is it a LF?
    {
      if ((selection >= minCID) && (selection <= maxCID))
      {
        selected = true;
        Serial.println("Using CID: " + String(selection));
      }
      else
      {
        Serial.println(F("Invalid CID. Please try again:"));
        selection = 0;
      }
    }
    else
    {
      selection *= 10; // Multiply selection by 10
      selection += c - '0'; // Add the new digit to selection      
    }
  }

  // Deactivate the profile
  if (myEG25.performPDPaction(0, EG25_G_PSD_ACTION_DEACTIVATE) != EG25_G_SUCCESS)
  {
    Serial.println(F("Warning: performPDPaction (deactivate profile) failed. Probably because no profile was active."));
  }

  // Map PSD profile 0 to the selected CID
  if (myEG25.setPDPconfiguration(0, EG25_G_PSD_CONFIG_PARAM_MAP_TO_CID, selection) != EG25_G_SUCCESS)
  {
    Serial.println(F("setPDPconfiguration (map to CID) failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }

  // Set the protocol type - this needs to match the defined IP type for the CID (as opposed to what was granted by the network)
  if (myEG25.setPDPconfiguration(0, EG25_G_PSD_CONFIG_PARAM_PROTOCOL, EG25_G_PSD_PROTOCOL_IPV4V6_V4_PREF) != EG25_G_SUCCESS)
  // You _may_ need to change the protocol type: ----------------------------------------^
  {
    Serial.println(F("setPDPconfiguration (set protocol type) failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }

  // Set a callback to process the results of the PSD Action
  myEG25.setPSDActionCallback(&processPSDAction);

  // Activate the profile
  if (myEG25.performPDPaction(0, EG25_G_PSD_ACTION_ACTIVATE) != EG25_G_SUCCESS)
  {
    Serial.println(F("performPDPaction (activate profile) failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }

  for (int i = 0; i < 100; i++) // Wait for up to a second for the PSD Action URC to arrive
  {
    myEG25.poll(); // Keep processing data from the EG25 so we can process the PSD Action
    delay(10);
  }

  // Save the profile to NVM - so we can load it again in the later examples
  if (myEG25.performPDPaction(0, EG25_G_PSD_ACTION_STORE) != EG25_G_SUCCESS)
  {
    Serial.println(F("performPDPaction (save to NVM) failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }

}

void loop()
{
  myEG25.poll(); // Keep processing data from the EG25 so we can process URCs from the PSD Action
}
