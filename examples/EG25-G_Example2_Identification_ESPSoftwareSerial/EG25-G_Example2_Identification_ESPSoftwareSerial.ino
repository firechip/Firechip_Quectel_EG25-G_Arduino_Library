/*

  EG25-G Example
  ===============

  Identification - using ESPSoftwareSerial
  
  Written by: Paul Clark
  Date: December 29th 2021

  This example demonstrates how to read the EG25's:
    Manufacturer identification
    Model identification
    Firmware version identification
    Product Serial No.
    IMEI identification
    IMSI identification
    SIM CCID
    Subscriber number
    Capabilities
    SIM state

  The ESP32 core doesn't include SoftwareSerial. Instead we use the library by Peter Lerup and Dirk O. Kaar:
  https://github.com/plerup/espsoftwareserial

  Feel like supporting open source hardware?
  Buy a board from Firechip!

  Licence: MIT
  Please see LICENSE.md for full details

*/

// Include SoftwareSerial.h _before_ including FC0003390327.h
// to allow the EG25-G library to detect SoftwareSerial.h using an #if __has_include
#include <SoftwareSerial.h> //Click here to get the library: http://librarymanager/All#ESPSoftwareSerial_ESP8266/ESP32

#include <FC0003390327.h> //Click here to get the library: http://librarymanager/All#Firechip_Quectel_EG25-G_Arduino_Library

// Create a SoftwareSerial object to pass to the EG25-G library
// Note: we need to call atSerial.begin and atSerial.end in setup() - see below for details
SoftwareSerial atSerial;

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

// Map SIM states to more readable strings
String simStateString[] =
{
  "Not present",      // 0
  "PIN needed",       // 1
  "PIN blocked",      // 2
  "PUK blocked",      // 3
  "Not operational",  // 4
  "Restricted",       // 5
  "Operational"       // 6
};

// processSIMstate is provided to the EG25-G library via a 
// callback setter -- setSIMstateReadCallback. (See setup())
void processSIMstate(EG25_G_sim_states_t state)
{
  Serial.println();
  Serial.print(F("SIM state:           "));
  Serial.print(String(state));
  Serial.println();
}

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

  // ESPSoftwareSerial does not like repeated .begin's without a .end in between.
  // We need to .begin and .end the atSerial port here, before the myEG25.begin, to set up the pin numbers etc.
  // E.g. to use: 57600 baud; 8 databits, no parity, 1 stop bit; RXD on pin 33; TXD on pin 32; no inversion.
  Serial.println(F("Configuring SoftwareSerial atSerial"));
  atSerial.begin(57600, SWSERIAL_8N1, 33, 32, false);
  atSerial.end();

  // Initialize the EG25
  if (myEG25.begin(atSerial, 57600) )
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

  Serial.println("Manufacturer ID:     " + String(myEG25.getManufacturerID()));
  Serial.println("Model ID:            " + String(myEG25.getModelID()));
  Serial.println("Firmware Version:    " + String(myEG25.getFirmwareVersion()));
  Serial.println("Product Serial No.:  " + String(myEG25.getSerialNo()));
  Serial.println("IMEI:                " + String(myEG25.getIMEI()));
  Serial.println("IMSI:                " + String(myEG25.getIMSI()));
  Serial.println("SIM CCID:            " + String(myEG25.getCCID()));
  Serial.println("Subscriber No.:      " + String(myEG25.getSubscriberNo()));
  Serial.println("Capabilities:        " + String(myEG25.getCapabilities()));

  // Set a callback to return the SIM state once requested
  myEG25.setSIMstateReportCallback(&processSIMstate);
  // Now enable SIM state reporting for states 0 to 6 (by setting the reporting mode LSb)
  if (myEG25.setSIMstateReportingMode(1) == EG25_G_SUCCESS)
    Serial.println("SIM state reports requested...");
  // You can disable the SIM status reports again by calling assetTracker.setSIMstateReportingMode(0)
}

void loop()
{
  myEG25.poll(); // Keep processing data from the EG25 so we can extract the SIM status
}
