
#include "secrets.h" // Update secrets.h with your AssistNow token string

// Quectel AssistNow https servers
const char assistNowOfflineServer[] = "offline-live1.services.Quectel.com";
//const char assistNowOfflineServer[] = "offline-live2.services.Quectel.com"; // Alternate server

const char getQuery[] = "GetOfflineData.ashx?";
const char tokenPrefix[] = "token=";
const char tokenSuffix[] = ";";
const char getGNSS[] = "gnss=gps,glo;"; // GNSS can be: gps,qzss,glo,bds,gal
const char getFormat[] = "format=mga;"; // Data format. Leave set to mga for M8 onwards. Can be aid.
const char getPeriod[] = "period=1;"; // Optional. The number of weeks into the future that the data will be valid. Can be 1-5. Default = 4.
const char getMgaResolution[] = "resolution=1;"; // Optional. Data resolution: 1 = every day; 2 = every other day; 3 = every 3rd day.
//Note: always use resolution=1. findMGAANOForDate does not yet support finding the 'closest' date. It needs an exact match.

volatile bool httpResultSeen = false; // Flag to indicate that the HTTP URC was received

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

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

  httpResultSeen = true; // Set the flag
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bool getAssistNowOfflineData(String theFilename)
{
  // Use HTTP GET to receive the AssistNow_Offline data. Store it in the EG25-G's internal file system.

  String theServer = assistNowOfflineServer; // Convert the AssistNow server to String

  const int REQUEST_BUFFER_SIZE  = 256;
  char theRequest[REQUEST_BUFFER_SIZE];

  // Assemble the request
  // Note the slash at the beginning
  snprintf(theRequest, REQUEST_BUFFER_SIZE, "/%s%s%s%s%s%s%s%s",
    getQuery,
    tokenPrefix,
    myAssistNowToken,
    tokenSuffix,
    getGNSS,
    getFormat,
    getPeriod,
    getMgaResolution
    );


  String theRequestStr = theRequest; // Convert to String

  Serial.print(F("getAssistNowOfflineData: HTTP GET is https://"));
  Serial.print(theServer);
  Serial.println(theRequestStr);

  Serial.print(F("getAssistNowOfflineData: the AssistNow data will be stored in: "));
  Serial.println(theFilename);

  // Reset HTTP profile 0
  myEG25.resetHTTPprofile(0);
  
  // Set the server name
  myEG25.setHTTPserverName(0, theServer);
  
  // Use HTTPS
  myEG25.setHTTPsecure(0, false); // Setting this to true causes the GET to fail. Maybe due to the default CMNG profile?

  // Set a callback to process the HTTP command result
  myEG25.setHTTPCommandCallback(&processHTTPcommandResult);

  httpResultSeen = false; // Clear the flag

  // HTTP GET
  myEG25.sendHTTPGET(0, theRequestStr, theFilename);

  // Wait for 20 seconds while calling myEG25.bufferedPoll() to see the HTTP result.
  Serial.print(F("getAssistNowOfflineData: Waiting up to 20 seconds for the HTTP Result"));
  int i = 0;
  while ((i < 20000) && (httpResultSeen == false))
  {
    myEG25.bufferedPoll(); // Keep processing data from the EG25 so we can catch the HTTP command result
    i++;
    delay(1);
    if (i % 1000 == 0)
      Serial.print(F("."));
  }
  Serial.println();
  
  if (httpResultSeen == false)
  {
    Serial.print(F("getAssistNowOfflineData: HTTP GET failed!"));
    return false;
  }

  return true;
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void prettyPrintString(String theString) // Pretty-print a String in HEX and ASCII format
{
  int theLength = theString.length();
  
  Serial.println();
  Serial.print(F("String length is "));
  Serial.print(theLength);
  Serial.print(F(" (0x"));
  Serial.print(theLength, HEX);
  Serial.println(F(")"));
  Serial.println();

  for (int i = 0; i < theLength; i += 16)
  {
    if (i < 10000) Serial.print(F("0"));
    if (i < 1000) Serial.print(F("0"));
    if (i < 100) Serial.print(F("0"));
    if (i < 10) Serial.print(F("0"));
    Serial.print(i);

    Serial.print(F(" 0x"));

    if (i < 0x1000) Serial.print(F("0"));
    if (i < 0x100) Serial.print(F("0"));
    if (i < 0x10) Serial.print(F("0"));
    Serial.print(i, HEX);

    Serial.print(F(" "));

    int j;
    for (j = 0; ((i + j) < theLength) && (j < 16); j++)
    {
      if (theString[i + j] < 0x10) Serial.print(F("0"));
      Serial.print(theString[i + j], HEX);
      Serial.print(F(" "));
    }

    if (((i + j) == theLength) && (j < 16))
    {
      for (int k = 0; k < (16 - (theLength % 16)); k++)
      {
        Serial.print(F("   "));
      }
    }
      
    for (j = 0; ((i + j) < theLength) && (j < 16); j++)
    {
      if ((theString[i + j] >= 0x20) && (theString[i + j] <= 0x7E))
        Serial.write(theString[i + j]);
      else
        Serial.print(F("."));
    }

    Serial.println();
  }

  Serial.println();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void prettyPrintChars(char *theData, int theLength) // Pretty-print char data in HEX and ASCII format
{
  Serial.println();
  Serial.print(F("String length is "));
  Serial.print(theLength);
  Serial.print(F(" (0x"));
  Serial.print(theLength, HEX);
  Serial.println(F(")"));
  Serial.println();

  for (int i = 0; i < theLength; i += 16)
  {
    if (i < 10000) Serial.print(F("0"));
    if (i < 1000) Serial.print(F("0"));
    if (i < 100) Serial.print(F("0"));
    if (i < 10) Serial.print(F("0"));
    Serial.print(i);

    Serial.print(F(" 0x"));

    if (i < 0x1000) Serial.print(F("0"));
    if (i < 0x100) Serial.print(F("0"));
    if (i < 0x10) Serial.print(F("0"));
    Serial.print(i, HEX);

    Serial.print(F(" "));

    int j;
    for (j = 0; ((i + j) < theLength) && (j < 16); j++)
    {
      if (theData[i + j] < 0x10) Serial.print(F("0"));
      Serial.print(theData[i + j], HEX);
      Serial.print(F(" "));
    }

    if (((i + j) == theLength) && (j < 16))
    {
      for (int k = 0; k < (16 - (theLength % 16)); k++)
      {
        Serial.print(F("   "));
      }
    }
      
    for (j = 0; ((i + j) < theLength) && (j < 16); j++)
    {
      if ((theData[i + j] >= 0x20) && (theData[i + j] <= 0x7E))
        Serial.write(theData[i + j]);
      else
        Serial.print(F("."));
    }

    Serial.println();
  }

  Serial.println();
}
