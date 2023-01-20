/*

  EG25-G Example
  ===============

  Ping

  Written by: Paul Clark
  Date: November 18th 2020

  This example uses the EG25's mobile data connection to ping a server.

  Feel like supporting open source hardware?
  Buy a board from Firechip!

  Licence: MIT
  Please see LICENSE.md for full details

*/

#include <IPAddress.h>

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

String pingMe = ""; // The name of the server we are going to ping

// processPingResult is provided to the EG25-G library via a 
// callback setter -- setPingCallback. (See the end of setup())
void processPingResult(int retry, int p_size, String remote_hostname, IPAddress ip, int ttl, long rtt)
{
  Serial.println();
  Serial.print(F("Ping Result:  Retry #:"));
  Serial.print(retry);
  Serial.print(F("  Ping Size (Bytes):"));
  Serial.print(p_size);
  Serial.print(F("  Remote Host:\""));
  Serial.print(remote_hostname);
  Serial.print(F("\"  IP Address:\""));
  Serial.print(String(ip[0]));
  Serial.print(F("."));
  Serial.print(String(ip[1]));
  Serial.print(F("."));
  Serial.print(String(ip[2]));
  Serial.print(F("."));
  Serial.print(String(ip[3]));
  Serial.print(F("\"  Time To Live (hops):"));
  Serial.print(ttl);
  Serial.print(F("  Round Trip (ms):"));
  Serial.print(rtt);
  Serial.println();
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

  // Deactivate the profile - in case one is already active
  if (myEG25.performPDPaction(0, EG25_G_PSD_ACTION_DEACTIVATE) != EG25_G_SUCCESS)
  {
    Serial.println(F("Warning: performPDPaction (deactivate profile) failed. Probably because no profile was active."));
  }

  // Load the profile from NVM - these were saved by a previous example
  if (myEG25.performPDPaction(0, EG25_G_PSD_ACTION_LOAD) != EG25_G_SUCCESS)
  {
    Serial.println(F("performPDPaction (load from NVM) failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }

  // Activate the profile
  if (myEG25.performPDPaction(0, EG25_G_PSD_ACTION_ACTIVATE) != EG25_G_SUCCESS)
  {
    Serial.println(F("performPDPaction (activate profile) failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }

  Serial.println();
  Serial.println(F("*** Set the Serial Monitor line ending to Newline ***"));

  Serial.println();
  Serial.println(F("Enter the name of the server you want to ping (followed by LF / Newline): "));

  // Set a callback to process the Ping result
  myEG25.setPingCallback(&processPingResult);
}

void loop()
{
  if (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n')
    {
      // Newline received so let's do that ping!
      myEG25.ping(pingMe); // Use the default parameters
      
      // Use custom parameters
      //int retries = 4; // number of retries
      //int p_size = 32; // packet size (bytes)
      //unsigned long timeout = 5000; // timeout (ms)
      //int ttl = 32; // Time To Live
      //myEG25.ping(pingMe, retries, p_size, timeout, ttl);
      
      pingMe = ""; // Clear the server name for the next try
    }
    else
    {
      // Add serial characters to the server address
      pingMe += c;
    }
  }
  
  myEG25.poll(); // Keep processing data from the EG25 so we can catch the Ping result
}
