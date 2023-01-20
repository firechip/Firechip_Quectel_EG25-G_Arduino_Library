/*

  EG25-G Example
  ===============

  Quectel GNSS NTRIP Caster Client - Polling

  This version uses polling to check for the arrival of new RTK correction and NMEA GPGGA data.
  It performs better on Mbed OS platforms: Firechip Artemis; Arduino Nano

  Written by: Paul Clark
  Date: January 18th 2021

  This example uses the EG25's mobile data connection to:
    * Request RTK RTCM data from a NTRIP Caster service
    * Push the RTCM data to an external Quectel GNSS module over I2C (not to the one built-in to the EG25-G10M8S)
    * NMEA GPGGA data is pushed to the Caster every 10 seconds

  The PDP profile is read from NVM. Please make sure you have run examples 4 & 7 previously to set up the profile.

  Update secrets.h with your NTRIP Caster username and password

  **************************************************************************************************
  * Important Note:                                                                                *
  *                                                                                                *
  * This example pulls kBytes of correction data from the NTRIP Caster.                            *
  * Depending on your location and service provider, the data rate may exceed the allowable        *
  * rates for LTE-M or NB-IoT.                                                                     *
  * Worst case, your service provider may throttle or block the connection - now or in the future. *
  * We are looking for a long-term solution to this - almost certainly using LTE Cat 1 instead.    *
  **************************************************************************************************
  
  Feel like supporting open source hardware?
  Buy a board from Firechip!

  Licence: MIT
  Please see LICENSE.md for full details

*/

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// The ESP32 core has a built in base64 library but not every platform does
// We'll use an external lib if necessary.

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_APOLLO3) || defined(ARDUINO_ARDUINO_NANO33BLE)
#include "base64.h" //Built-in ESP32 library
#else
#include <Base64.h> //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif

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

#include <Firechip_Quectel_GNSS_Arduino_Library.h> //http://librarymanager/All#Firechip_Quectel_GNSS
SFE_QUECTEL_GNSS myGNSS;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Globals

volatile int socketNum = -1; // The TCP socket number. -1 indicates invalid/closed socket
volatile bool connectionOpen = false; // Flag to indicate if the connection to the NTRIP Caster is open
volatile unsigned long lastReceivedRTCM_ms; // Record when data last arrived - so we can time out if required

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

  // Start communication with the EG25-G. Load and activate the Packet Switched Data profile.

  //myEG25.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  // For the MicroMod Asset Tracker, we need to invert the power pin so it pulls high instead of low
  // Comment the next line if required
  myEG25.invertPowerPin(true); 

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

  //Print the dynamic IP Address (for profile 0)
  IPAddress myAddress;
  myEG25.getNetworkAssignedIPAddress(0, &myAddress);
  Serial.print(F("\r\nMy IP Address is: "));
  Serial.print(myAddress[0]);
  Serial.print(F("."));
  Serial.print(myAddress[1]);
  Serial.print(F("."));
  Serial.print(myAddress[2]);
  Serial.print(F("."));
  Serial.println(myAddress[3]);

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

  // Start I2C. Connect to the GNSS.

  Wire.begin(); //Start I2C

  // Uncomment the next line to enable the 'major' GNSS debug messages on Serial so you can see what AssistNow data is being sent
  //myGNSS.enableDebugging(Serial, true);

  if (myGNSS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Quectel GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  Serial.println(F("Quectel module connected"));

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);                                //Set the I2C port to output both NMEA and UBX messages
  myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.

  myGNSS.setDGNSSConfiguration(SFE_QUECTEL_DGNSS_MODE_FIXED); // Set the differential mode - ambiguities are fixed whenever possible

  myGNSS.setNavigationFrequency(1); //Set the navigation rate to 1Hz

  myGNSS.setAutoPVT(true); // Enable automatic PVT reports at the navigation frequency

  // Set the Main Talker ID to "GP". The NMEA GGA messages will be GPGGA instead of GNGGA
  myGNSS.setMainTalkerID(SFE_QUECTEL_MAIN_TALKER_ID_GP);

  myGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_I2C, 10); // Tell the module to output GGA every 10 seconds

  //myGNSS.saveConfiguration(VAL_CFG_SUBSEC_IOPORT | VAL_CFG_SUBSEC_MSGCONF); //Optional: Save the ioPort and message settings to NVM

}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  enum states // Use a 'state machine' to open and close the connection
  {
    open_connection,
    check_connection_and_wait_for_keypress,
    close_connection,
    waiting_for_keypress
  };
  static states state = open_connection;

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

  switch (state)
  {
    case open_connection:
      Serial.println(F("Connecting to the NTRIP caster..."));
      if (beginClient((int *)&socketNum, (bool *)&connectionOpen)) // Try to open the connection to the caster
      {
        Serial.println(F("Connected to the NTRIP caster! Press any key to disconnect..."));
        state = check_connection_and_wait_for_keypress; // Move on
      }
      else
      {
        Serial.print(F("Could not connect to the caster. Trying again in 5 seconds."));
        for (int i = 0; i < 5; i++)
        {
          delay(1000);
          Serial.print(F("."));
        }
        Serial.println();
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    case check_connection_and_wait_for_keypress:
      // If the connection has dropped or timed out, or if the user has pressed a key
      if ((checkConnection((int)socketNum, (bool)connectionOpen) == false) || (keyPressed()))
      {
        state = close_connection; // Move on
      }
      delay(50);
      break;
    
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    case close_connection:
      Serial.println(F("Closing the connection to the NTRIP caster..."));
      closeConnection((int *)&socketNum, (bool *)&connectionOpen);
      Serial.println(F("Press any key to reconnect..."));
      state = waiting_for_keypress; // Move on
      break;
    
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    case waiting_for_keypress:
      // Wait for the user to press a key
      checkConnection((int)socketNum, (bool)connectionOpen); // 'Check' the connection - to print the latest PVT data
      if (keyPressed())
        state = open_connection; // Move on
      break; 
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
