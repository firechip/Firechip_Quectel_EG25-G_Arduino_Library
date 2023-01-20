/*

  EG25-G Example
  ===============

  Set Clock With NTP

  Written by: Paul Clark
  Date: January 9th 2022

  This example demonstrates how to set the EG25-G's internal Real Time Clock using NTP.
  When the EG25-G registers on a network, it will set its clock automatically if:
    Automatic time zone is enabled
    Your network supports NITZ (Network Identity and Time Zone)
  But for things like AssistNow Offline, it is convenient to have the EG25's RTC set to UTC.
  Then the clock can be used to select the AssistNow data for now/today without time zone headaches.
  This example shows how to:
    Disable the automatic time zone (using autoTimeZoneForBegin before .begin)
    Set the EG25's clock to UTC using NTP (see EG25-G_NTP.ino)

  The PDP profile is read from NVM. Please make sure you have run examples 4 & 7 previously to set up the profile.  

  Feel like supporting open source hardware?
  Buy a board from Firechip!

  Licence: MIT
  Please see LICENSE.md for full details

*/

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

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

// Create a EG25_G object to use throughout the sketch
// If you are using the LTE GNSS Breakout, and have access to the EG25's RESET_N pin, you can pass that to the library too
// allowing it to do an emergency shutdown if required.
// Change the pin numbers if required.
//EG25_G myEG25(34, 35); // PWR_ON, RESET_N

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  String currentOperator = "";

  Serial.begin(115200); // Start the serial console

  // Wait for user to press key to begin
  Serial.println(F("EG25-G Example"));
  Serial.println(F("Wait for the EG25 NI LED to light up - then press any key to begin"));
  
  while (!Serial.available()) // Wait for the user to press a key (send any serial character)
    ;
  while (Serial.available()) // Empty the serial RX buffer
    Serial.read();

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

  //myEG25.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  // For the MicroMod Asset Tracker, we need to invert the power pin so it pulls high instead of low
  // Comment the next line if required
  myEG25.invertPowerPin(true);

  // Disable the automatic time zone so we can use UTC. We need to do this _before_ .begin
  myEG25.autoTimeZoneForBegin(false);

  // Initialize the EG25
  if (myEG25.begin(atSerial, 115200) )
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

  // Deactivate the PSD profile - in case one is already active
  if (myEG25.performPDPaction(0, EG25_G_PSD_ACTION_DEACTIVATE) != EG25_G_SUCCESS)
  {
    Serial.println(F("Warning: performPDPaction (deactivate profile) failed. Probably because no profile was active."));
  }

  // Load the PSD profile from NVM - these were saved by a previous example
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

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

  //Get the time from an NTP server and use it to set the clock. See EG25-G_NTP.ino
  uint8_t y, mo, d, h, min, s;
  bool success = getNTPTime(&y, &mo, &d, &h, &min, &s);
  if (!success)
  {
    Serial.println(F("getNTPTime failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }

  //Set the EG25's RTC. Set the time zone to zero so the clock uses UTC
  if (myEG25.setClock(y, mo, d, h, min, s, 0) != EG25_G_SUCCESS)
  {
    Serial.println(F("setClock failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }
  
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

  // Read and print the clock as a String
  Serial.print(F("The UTC time is: "));
  String theTime = myEG25.clock();
  Serial.println(theTime);
}

void loop()
{
  // Nothing to do here
}
