/*

  EG25-G Example
  ===============

  ThingSpeak (HTTP POST / GET)

  Written by: Paul Clark
  Date: November 18th 2020

  This example uses the EG25's mobile data connection to send random temperatures to ThingSpeak using HTTP POST or GET.
  https://thingspeak.com/

  You will need to:
    Create a ThingSpeak User Account – https://thingspeak.com/login
    Create a new Channel by selecting Channels, My Channels, and then New Channel
    Note the Write API Key and copy&paste it into myWriteAPIKey below
  The random temperature reading will be added to the channel as "Field 1"

  Feel like supporting open source hardware?
  Buy a board from Firechip!

  Licence: MIT
  Please see LICENSE.md for full details

*/

#include <IPAddress.h>

// ThingSpeak via HTTP POST / GET

String myWriteAPIKey = "PFIOEXW1VF21T7O6"; // Change this to your API key

String serverName = "api.thingspeak.com"; // Domain Name for HTTP POST / GET

// EG25-G

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

// processHTTPcommandResult is provided to the EG25-G library via a 
// callback setter -- setHTTPCommandCallback. (See the end of setup())
void processHTTPcommandResult(int profile, int command, int result)
{
  Serial.println();
  Serial.print(F("HTTP Command Result:  profile: "));
  Serial.print(profile);
  Serial.print(F("  command: "));
  Serial.print(command);
  Serial.print(F("  result: "));
  Serial.print(result);
  if (result == 0)
    Serial.print(F(" (fail)"));
  if (result == 1)
    Serial.print(F(" (success)"));
  Serial.println();

  // Get and print the most recent HTTP protocol error
  int error_class;
  int error_code;
  myEG25.getHTTPprotocolError(0, &error_class, &error_code);
  Serial.print(F("Most recent HTTP protocol error:  class: "));
  Serial.print(error_class);
  Serial.print(F("  code: "));
  Serial.print(error_code);
  if (error_code == 0)
    Serial.print(F(" (no error)"));
  Serial.println();

  // Read and print the HTTP POST result
  String postResult = "";
  myEG25.getFileContents("post_response.txt", &postResult);
  Serial.print(F("HTTP command result was: "));
  Serial.println(postResult);

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

  // Activate the PSD profile
  if (myEG25.performPDPaction(0, EG25_G_PSD_ACTION_ACTIVATE) != EG25_G_SUCCESS)
  {
    Serial.println(F("performPDPaction (activate profile) failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }

  // Reset HTTP profile 0
  myEG25.resetHTTPprofile(0);
  
  // Set the server name
  myEG25.setHTTPserverName(0, serverName);
  
  // Use HTTPS
  myEG25.setHTTPsecure(0, false); // Setting this to true causes the POST / GET to fail. Not sure why...

  // Set a callback to process the HTTP command result
  myEG25.setHTTPCommandCallback(&processHTTPcommandResult);
}

void loop()
{
  float temperature = ((float)random(2000,3000)) / 100.0; // Create a random temperature between 20 and 30

//---

  // Send data using HTTP POST
  String httpRequestData = "api_key=" + myWriteAPIKey + "&field1=" + String(temperature);

  Serial.print(F("POSTing a temperature of "));
  Serial.print(String(temperature));
  Serial.println(F(" to ThingSpeak"));
        
  // Send HTTP POST request to /update. The response will be written to post_response.txt in the EG25's file system
  myEG25.sendHTTPPOSTdata(0, "/update", "post_response.txt", httpRequestData, EG25_G_HTTP_CONTENT_APPLICATION_X_WWW);

//---

//  // Send data using HTTP GET
//  String path = "/update?api_key=" + myWriteAPIKey + "&field1=" + String(temperature);
//
//  Serial.print(F("Send a temperature of "));
//  Serial.print(String(temperature));
//  Serial.println(F(" to ThingSpeak using HTTP GET"));
//        
//  // Send HTTP POST request to /update. The response will be written to post_response.txt in the EG25's file system
//  myEG25.sendHTTPGET(0, path, "post_response.txt");
  
//---

  // Wait for 20 seconds
  for (int i = 0; i < 20000; i++)
  {
    myEG25.poll(); // Keep processing data from the EG25 so we can catch the HTTP command result
    delay(1);
  }
}
