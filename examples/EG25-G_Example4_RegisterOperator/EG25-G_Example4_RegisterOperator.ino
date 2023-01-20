/*

  EG25-G Example
  ===============

  Register Operator

  Written by: Paul Clark
  Date: November 18th 2020

  This example demonstrates how to register the EG25 with a network operator.

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

// MNO_GLOBAL is the factory-programmed default
// If you are in Europe, you may find no operators unless you choose MNO_STD_EUROPE
const mobile_network_operator_t MOBILE_NETWORK_OPERATOR = MNO_GLOBAL;

const String MOBILE_NETWORK_STRINGS[] = {"default (Undefined/Regulatory)", "SIM ICCID", "AT&T", "Verizon", 
  "Telstra", "T-Mobile US", "China Telecom", "Sprint", "Vodafone", "NTT DoCoMo", "Telus", "SoftBank",
  "Deutsche Telekom", "US Cellular", "SKT", "global (factory default)", "standard Europe",
  "standard Europe No-ePCO", "NOT RECOGNIZED"};

// Convert the operator number into an index for MOBILE_NETWORK_STRINGS
int convertOperatorNumber( mobile_network_operator_t mno)
{
  switch (mno)
  {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
      return ((int)mno);
      break;
    case 8:
      return 7;
      break;
    case 19:
      return 8;
      break;
    case 20:
      return 9;
      break;
    case 21:
      return 10;
      break;
    case 28:
      return 11;
      break;
    case 31:
      return 12;
      break;
    case 32:
      return 13;
      break;
    case 39:
      return 14;
      break;
    case 90:
      return 15;
      break;
    case 100:
      return 16;
      break;
    case 101:
      return 17;
      break;
    default: // NOT RECOGNIZED
      return 18;
      break;
  }
}

// This defines the size of the ops struct array. To narrow the operator
// list, set MOBILE_NETWORK_OPERATOR to AT&T, Verizon etc. instead
// of MNO_SW_DEFAULT.
#define MAX_OPERATORS 10

// Uncomment this line if you want to be able to communicate directly with the EG25 in the main loop
//#define DEBUG_PASSTHROUGH_ENABLED

void setup()
{
  int opsAvailable;
  struct operator_stats ops[MAX_OPERATORS];
  String currentOperator = "";
  bool newConnection = true;

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

  // First check to see if we're already connected to an operator:
  if (myEG25.getOperator(&currentOperator) == EG25_G_SUCCESS) {
    Serial.print(F("Already connected to: "));
    Serial.println(currentOperator);
    // If already connected provide the option to type y to connect to new operator
    Serial.println(F("Press y to connect to a new operator, or any other key to continue.\r\n"));
    while (!Serial.available()) ;
    if (Serial.read() != 'y')
    {
      newConnection = false;
    }
    else
    {
      myEG25.deregisterOperator(); // Deregister from the current operator so we can connect to a new one
    }
    while (Serial.available()) Serial.read();
  }

  if (newConnection) {
    // Set MNO to either Verizon, T-Mobile, AT&T, Telstra, etc.
    // This will narrow the operator options during our scan later
    Serial.println(F("Setting mobile-network operator"));
    if (myEG25.setNetworkProfile(MOBILE_NETWORK_OPERATOR))
    {
      Serial.print(F("Set mobile network operator to "));
      Serial.println(MOBILE_NETWORK_STRINGS[convertOperatorNumber(MOBILE_NETWORK_OPERATOR)] + "\r\n");
    }
    else
    {
      Serial.println(F("Error setting MNO. Try cycling the power. Freezing..."));
      while (1) ;
    }
    
    // Wait for user to press button before initiating network scan.
    Serial.println(F("Press any key scan for networks.."));
    serialWait();

    Serial.println(F("Scanning for networks...this may take up to 3 minutes\r\n"));
    // myEG25.getOperators takes in a operator_stats struct pointer and max number of
    // structs to scan for, then fills up those objects with operator names and numbers
    opsAvailable = myEG25.getOperators(ops, MAX_OPERATORS); // This will block for up to 3 minutes

    if (opsAvailable > 0)
    {
      // Pretty-print operators we found:
      Serial.println("Found " + String(opsAvailable) + " operators:");
      printOperators(ops, opsAvailable);
      Serial.println(String(opsAvailable + 1) + ": use automatic selection");
      Serial.println();

      // Wait until the user presses a key to initiate an operator connection
      Serial.println("Press 1-" + String(opsAvailable + 1) + " to select an operator.");
      char c = 0;
      bool selected = false;
      while (!selected) {
        while (!Serial.available()) ;
        c = Serial.read();
        int selection = c - '0';
        if ((selection >= 1) && (selection <= (opsAvailable + 1))) {
          selected = true;
          Serial.println("Connecting to option " + String(selection));
          if (selection == (opsAvailable + 1))
          {
            if (myEG25.automaticOperatorSelection() == EG25_G_SUCCESS)
            {
              Serial.println("Automatic operator selection: successful\r\n");
            }
            else
            {
              Serial.println(F("Automatic operator selection: error. Reset and try again, or try another network."));
            }
          }
          else
          {
            if (myEG25.registerOperator(ops[selection - 1]) == EG25_G_SUCCESS)
            {
              Serial.println("Network " + ops[selection - 1].longOp + " registered\r\n");
            }
            else
            {
              Serial.println(F("Error connecting to operator. Reset and try again, or try another network."));
            }
          }
        }
      }
    }
    else
    {
      Serial.println(F("Did not find an operator. Double-check SIM and antenna, reset and try again, or try another network."));
      while (1) ;
    }
  }

  // At the very end print connection information
  printInfo();
}

void loop()
{
  // Loop provides a debugging interface.
  if (atSerial.available()) {
    Serial.write((char) atSerial.read());
  }
#ifdef DEBUG_PASSTHROUGH_ENABLED
  if (Serial.available()) {
    atSerial.write((char) Serial.read());
  }
#endif
}

void printInfo(void) {
  String currentApn = "";
  IPAddress ip(0, 0, 0, 0);
  String currentOperator = "";

  Serial.println(F("Connection info:"));
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

  // Operator name or number
  if (myEG25.getOperator(&currentOperator) == EG25_G_SUCCESS)
  {
    Serial.print(F("Operator: "));
    Serial.println(currentOperator);
  }

  // Received signal strength
  Serial.println("RSSI: " + String(myEG25.rssi()));
  Serial.println();
}

void printOperators(struct operator_stats * ops, int operatorsAvailable)
{
  for (int i = 0; i < operatorsAvailable; i++)
  {
    Serial.print(String(i + 1) + ": ");
    Serial.print(ops[i].longOp + " (" + String(ops[i].numOp) + ") - ");
    switch (ops[i].stat)
    {
    case 0:
      Serial.print(F("UNKNOWN"));
      break;
    case 1:
      Serial.print(F("AVAILABLE"));
      break;
    case 2:
      Serial.print(F("CURRENT"));
      break;
    case 3:
      Serial.print(F("FORBIDDEN"));
      break;
    }
    switch (ops[i].act)
    {
    // EG25-G only supports LTE
    case 7:
      Serial.print(F(" - LTE"));
      break;
    }
  }
  Serial.println();
}

void serialWait()
{
  while (Serial.available()) Serial.read();
  while (!Serial.available()) ;
  delay(100);
  while (Serial.available()) Serial.read();
}
