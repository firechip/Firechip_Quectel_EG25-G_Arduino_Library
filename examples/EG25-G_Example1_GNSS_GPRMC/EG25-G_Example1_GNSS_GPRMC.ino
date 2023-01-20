/*

  EG25-G Example
  ===============

  GNSS GPRMC
  
  Written by: Paul Clark
  Date: November 18th 2020

  This example enables the EG25-GnnM8S' built-in GNSS receiver and reads the GPRMC message to
  get position, speed and time data.

  Feel like supporting open source hardware?
  Buy a board from Firechip!

  Licence: MIT
  Please see LICENSE.md for full details

*/

#include <FC0003390327.h> //Click here to get the library: http://librarymanager/All#Firechip_Quectel_EG25-G_Arduino_Library

// To create a SoftwareSerial object to pass to the EG25-G library instead
#ifdef EG25_G_SOFTWARE_SERIAL_ENABLED
SoftwareSerial atSerial(8, 9);
#else
// To connect to the EG25-G using hardware Serial1
#define atSerial Serial1
#endif
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

PositionData gps;
SpeedData spd;
ClockData clk;
boolean valid;

#define GPS_POLL_RATE 5000 // Read GPS every 5 seconds
unsigned long lastGpsPoll = 0;

void setup()
{
  Serial.begin(115200); // Start the serial console

  // Wait for user to press key to begin
  Serial.println(F("EG25-G Example"));
  Serial.println(F("Press any key to begin GNSS'ing"));
  
  while (!Serial.available()) // Wait for the user to press a key (send any serial character)
    ;
  while (Serial.available()) // Empty the serial RX buffer
    Serial.read();

  //myEG25.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  // For the MicroMod Asset Tracker, we need to invert the power pin so it pulls high instead of low
  // Comment the next line if required
  myEG25.invertPowerPin(true); 

  // Initialize the EG25
  if (myEG25.begin(atSerial, 9600) ) {
    Serial.println(F("EG25-G connected!"));
  }

  // Enable power for the GNSS active antenna
  // On the MicroMod Asset Tracker, the EG25 GPIO2 pin is used to control power for the antenna.
  // We need to pull GPIO2 (Pin 23) high to enable the power.
  myEG25.setGpioMode(myEG25.GPIO2, myEG25.GPIO_OUTPUT, 1);

  // From the Quectel EG25-G Positioning Implementation Application Note UBX-20012413 - R01
  // To enable the PPS output we need to:
  // Configure GPIO6 for TIME_PULSE_OUTPUT - .init does this
  // Enable the timing information request with +UTIMEIND=1 - setUtimeIndication()
  // Reset the time offset configuration with +UTIMECFG=0,0 - setUtimeConfiguration()
  // Request PPS output using GNSS+LTE (Best effort) with +UTIME=1,1 - setUtimeMode()
  // The bits that don't seem to be mentioned in the documentation are:
  //   +UTIME=1,1 also enables the GNSS module and so we don't need to call gpsPower.
  //   +UTIME=1,1 only works when the GNSS module if off. It returns an ERROR if the GNSS is already on.

  // Enable the timing information request (URC)
  //myEG25.setUtimeIndication(); // Use default (EG25_G_UTIME_URC_CONFIGURATION_ENABLED)
  
  // Clear the time offset
  myEG25.setUtimeConfiguration(); // Use default offset (offsetNanoseconds = 0, offsetSeconds = 0)
  
  // Set the UTIME mode to pulse-per-second output using a best effort from GNSS and LTE
  myEG25.setUtimeMode(); // Use defaults (mode = EG25_G_UTIME_MODE_PPS, sensor = EG25_G_UTIME_SENSOR_GNSS_LTE)

  myEG25.gpsEnableRmc(); // Enable GPRMC messages
}

void loop()
{
  if ((lastGpsPoll == 0) || (lastGpsPoll + GPS_POLL_RATE < millis()))
  {
    // Call (myEG25.gpsGetRmc to get coordinate, speed, and timing data
    // from the GPS module. Valid can be used to check if the GPS is
    // reporting valid data
    if (myEG25.gpsGetRmc(&gps, &spd, &clk, &valid) == EG25_G_SUCCESS)
    {
      printGPS();
      lastGpsPoll = millis();
    }
    else
    {
      delay(1000); // If RMC read fails, wait a second and try again
    }
  }
}

void printGPS(void)
{
  Serial.println();
  Serial.println("UTC: " + String(gps.utc));
  Serial.print("Time: ");
  if (clk.time.hour < 10) Serial.print('0'); // Print leading 0
  Serial.print(String(clk.time.hour) + ":");
  if (clk.time.minute < 10) Serial.print('0'); // Print leading 0
  Serial.print(String(clk.time.minute) + ":");
  if (clk.time.second < 10) Serial.print('0'); // Print leading 0
  Serial.print(String(clk.time.second) + ".");
  if (clk.time.ms < 10) Serial.print('0'); // Print leading 0
  Serial.println(String(clk.time.ms));
  Serial.println("Latitude: " + String(gps.lat, 7));
  Serial.println("Longitude: " + String(gps.lon, 7));
  Serial.println("Speed: " + String(spd.speed, 4) + " @ " + String(spd.cog, 4));
  Serial.println("Date (MM/DD/YY): " + String(clk.date.month) + "/" + 
    String(clk.date.day) + "/" + String(clk.date.year));
  Serial.println("Magnetic variation: " + String(spd.magVar));
  Serial.println("Status: " + String(gps.status));
  Serial.println("Mode: " + String(gps.mode));
  Serial.println();
}
