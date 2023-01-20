/*

  EG25-G Example
  ===============

  Send SMS

  Written by: Paul Clark
  Date: November 18th 2020

  This example demonstrates how to send SMS messages using the EG25.

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

  Serial.println();
  Serial.println(F("*** Set the Serial Monitor line ending to Newline ***"));
}

void loop()
{
  String destinationNumber = "";
  String message = "";
  boolean keepGoing = true;
  
  Serial.println();
  Serial.println(F("Enter the destination number (followed by LF / Newline): "));
  
  while (keepGoing)
  {
    if (Serial.available())
    {
      char c = Serial.read();
      if (c == '\n')
      {
        keepGoing = false; // Stop if we receive a newline
      }
      else
      {
        destinationNumber += c; // Add serial characters to the destination number
      }
    }
  }
  
  keepGoing = true;
  Serial.println();
  Serial.println(F("Enter the message (followed by LF): "));
  
  while (keepGoing)
  {
    if (Serial.available())
    {
      char c = Serial.read();
      if (c == '\n')
      {
        keepGoing = false; // Stop if we receive a newline
      }
      else
      {
        message += c; // Add serial characters to the destination number
      }
    }
  }

  // Once we receive a newline, send the text.
  Serial.println("Sending: \"" + message + "\" to " + destinationNumber);
  // Call myEG25.sendSMS(String number, String message) to send an SMS message.
  if (myEG25.sendSMS(destinationNumber, message) == EG25_G_SUCCESS)
  {
    Serial.println(F("sendSMS was successful"));
  }
}
