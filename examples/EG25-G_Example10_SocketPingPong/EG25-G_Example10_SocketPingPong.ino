/*

  EG25-G Example
  ===============

  Socket "Ping Pong" - TCP Data Transfers

  Written by: Paul Clark
  Date: December 30th 2021

  This example demonstrates how to transfer data from one EG25-G to another using TCP sockets.

  The PDP profile is read from NVM. Please make sure you have run examples 4 & 7 previously to set up the profile.
  
  If you select "Ping":
    The code asks for the IP Address of the "Pong" EG25-G
    The code then opens a TCP socket to the "Pong" EG25-G using port number TCP_PORT
    The code sends an initial "Ping" using Write Socket Data (+USOWR)
    The code polls continuously. When a +UUSORD URC message is received, data is read and passed to the socketReadCallback.
    When "Pong" is received by the callback, the code sends "Ping" in reply
    The Ping-Pong repeats 100 times
    The socket is closed after the 100th Ping is sent
  If you select "Pong":
    The code opens a TCP socket and waits for a connection and for data to arrive
    The code polls continuously. When a +UUSORD URC message is received, data is read and passed to the socketReadCallback.
    When "Ping" is received by the callback, the code sends "Pong" in reply
    The socket is closed after 120 seconds
  Start the "Pong" first!

  You may find that your service provider is blocking incoming TCP connections to the EG25-G, preventing the "Pong" from working...
  If that is the case, you can use this code to play ping-pong with another computer acting as a TCP Echo Server.
  Here's a quick how-to (assuming you are familiar with Python):
    Open up a Python editor on your computer
    Copy the Multi_TCP_Echo.py from the GitHub repo Utils folder: https://github.com/firechip/Firechip_Quectel_EG25-G_Arduino_Library/tree/main/Utils
    Log in to your router
    Find your local IP address (usually 192.168.0.something)
    Go into your router's Security / Port Forwarding settings:
      Create a new port forwarding rule
      The IP address is your local IP address
      Set the local port range to 1200-1200 (if you changed TCP_PORT, use that port number instead)
      Set the external port range to 1200-1200
      Set the protocol to TCP (or BOTH)
      Enable the rule
    This will open up a direct connection from the outside world, through your router, to port 1200 on your computer
      Remember to lock it down again when you're done!
    Edit the Python code and change 'HOST' to your local IP number:
      HOST = '192.168.0.nnn'
    Run the Python code
    Ask Google for your computer's public IP address:
      Google "what is my IP address"
    Run this code and choose the "Ping" option
    Enter your computer's public IP address when asked
    Sit back and watch the ping-pong!
    The code will stop after 100 Pings+Echos and 100 Pongs+Echos
      That's 400 TCP transfers in total!

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

// Create a EG25_G object to use throughout the sketch
// If you are using the LTE GNSS Breakout, and have access to the EG25's RESET_N pin, you can pass that to the library too
// allowing it to do an emergency shutdown if required.
// Change the pin numbers if required.
//EG25_G myEG25(34, 35); // PWR_ON, RESET_N

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

unsigned int TCP_PORT = 1200; // Change this if required

bool iAmPing;

// Keep track of how many ping-pong exchanges have taken place. "Ping" closes the socket when pingCount reaches pingPongLimit.
volatile int pingCount = 0;
volatile int pongCount = 0;
const int pingPongLimit = 100;

// Keep track of how long the socket has been open. "Pong" closes the socket when timeLimit (millis) is reached.
unsigned long startTime;
const unsigned long timeLimit = 120000; // 120 seconds

#include <IPAddress.h> // Needed for sockets
volatile int socketNum;

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// processSocketListen is provided to the EG25-G library via a 
// callback setter -- setSocketListenCallback. (See setup())
void processSocketListen(int listeningSocket, IPAddress localIP, unsigned int listeningPort, int socket, IPAddress remoteIP, unsigned int port)
{
  Serial.println();
  Serial.print(F("Socket connection made: listeningSocket "));
  Serial.print(listeningSocket);
  Serial.print(F(" localIP "));
  Serial.print(localIP[0]);
  Serial.print(F("."));
  Serial.print(localIP[1]);
  Serial.print(F("."));
  Serial.print(localIP[2]);
  Serial.print(F("."));
  Serial.print(localIP[3]);
  Serial.print(F(" listeningPort "));
  Serial.print(listeningPort);
  Serial.print(F(" socket "));
  Serial.print(socket);
  Serial.print(F(" remoteIP "));
  Serial.print(remoteIP[0]);
  Serial.print(F("."));
  Serial.print(remoteIP[1]);
  Serial.print(F("."));
  Serial.print(remoteIP[2]);
  Serial.print(F("."));
  Serial.print(remoteIP[3]);
  Serial.print(F(" port "));
  Serial.println(port);
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// processSocketData is provided to the EG25-G library via a 
// callback setter -- setSocketReadCallback. (See setup())
void processSocketData(int socket, String theData)
{
  Serial.println();
  Serial.print(F("Data received on socket "));
  Serial.print(socket);
  Serial.print(F(" : "));
  Serial.println(theData);
  
  if (theData == String("Ping")) // Look for the "Ping"
  {
    const char pong[] = "Pong";
    myEG25.socketWrite(socket, pong); // Send the "Pong"
    pongCount++;
  }

  if (theData == String("Pong")) // Look for the "Pong"
  {
    const char ping[] = "Ping";
    myEG25.socketWrite(socket, ping); // Send the "Ping"
    pingCount++;
  }

  Serial.print(F("pingCount = "));
  Serial.print(pingCount);
  Serial.print(F(" : pongCount = "));
  Serial.println(pongCount);
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// processSocketClose is provided to the EG25-G library via a 
// callback setter -- setSocketCloseCallback. (See setup())
// 
// Note: the EG25-G only sends a +UUSOCL URC when the socket os closed by the remote
void processSocketClose(int socket)
{
  Serial.println();
  Serial.print(F("Socket "));
  Serial.print(socket);
  Serial.println(F(" closed!"));
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

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

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void setup()
{
  String currentOperator = "";

  Serial.begin(115200); // Start the serial console

  // Wait for user to press key to begin
  Serial.println(F("EG25-G Example"));
  Serial.println(F("Wait until the EG25's NI LED lights up - then press any key to begin"));
  
  while (!Serial.available()) // Wait for the user to press a key (send any serial character)
    ;
  while (Serial.available()) // Empty the serial RX buffer
    Serial.read();

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

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

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

  // Set a callback to process the results of the PSD Action - OPTIONAL
  //myEG25.setPSDActionCallback(&processPSDAction);

  // Activate the profile
  if (myEG25.performPDPaction(0, EG25_G_PSD_ACTION_ACTIVATE) != EG25_G_SUCCESS)
  {
    Serial.println(F("performPDPaction (activate profile) failed! Freezing..."));
    while (1)
      ; // Do nothing more
  }

  //for (int i = 0; i < 100; i++) // Wait for up to a second for the PSD Action URC to arrive - OPTIONAL
  //{
  //  myEG25.bufferedPoll(); // Keep processing data from the EG25 so we can process the PSD Action
  //  delay(10);
  //}

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

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  // Set a callback to process the socket listen
  myEG25.setSocketListenCallback(&processSocketListen);
  
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  // Set a callback to process the socket data
  myEG25.setSocketReadCallback(&processSocketData);
  
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  // Set a callback to process the socket close
  // 
  // Note: the EG25-G only sends a +UUSOCL URC when the socket os closed by the remote
  myEG25.setSocketCloseCallback(&processSocketClose);
  
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  Serial.println(F("\r\nDo you want to Ping or Pong? (Always start the Pong first!)\r\n"));
  Serial.println(F("1: Ping"));
  Serial.println(F("2: Pong"));
  Serial.println(F("\r\nPlease enter the number (followed by LF / Newline): "));
  
  char c = 0;
  bool selected = false;
  int selection = 0;
  while (!selected)
  {
    while (!Serial.available()) ; // Wait for a character to arrive
    c = Serial.read(); // Read it
    if (c == '\n') // Is it a LF?
    {
      if ((selection >= 1) && (selection <= 2))
      {
        selected = true;
        if (selection == 1)
          Serial.println(F("\r\nPing selected!"));
        else
          Serial.println(F("\r\nPong selected!"));
      }
      else
      {
        Serial.println(F("Invalid choice. Please try again:"));
        selection = 0;
      }
    }
    else if ((c >= '0') && (c <= '9'))
    {
      selection = c - '0'; // Store a single digit
    }
  }

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  if (selection == 1) // Ping
  {
    iAmPing = true;
    
    Serial.println(F("\r\nPlease enter the IP number you want to ping (nnn.nnn.nnn.nnn followed by LF / Newline): "));

    char c = 0;
    bool selected = false;
    int val = 0;
    IPAddress theAddress = {0,0,0,0};
    int field = 0;
    while (!selected)
    {
      while (!Serial.available()) ; // Wait for a character to arrive
      c = Serial.read(); // Read it
      if (c == '\n') // Is it a LF?
      {
        theAddress[field] = val; // Store the current value
        if (field == 3)
          selected = true;
        else
        {
          Serial.println(F("Invalid IP Address. Please try again:"));
          val = 0;
          field = 0;
        }
      }
      else if (c == '.') // Is it a separator
      {
        theAddress[field] = val; // Store the current value
        if (field <= 2)
          field++; // Increment the field
        val = 0; // Reset the value
      }
      else if ((c >= '0') && (c <= '9'))
      {
        val *= 10; // Multiply by 10
        val += c - '0'; // Add the digit
      }
    }

    Serial.print(F("Remote address is: "));
    Serial.print(theAddress[0]);
    Serial.print(F("."));
    Serial.print(theAddress[1]);
    Serial.print(F("."));
    Serial.print(theAddress[2]);
    Serial.print(F("."));
    Serial.println(theAddress[3]);
    
    // Open the socket
    socketNum = myEG25.socketOpen(EG25_G_TCP);
    if (socketNum == -1)
    {
      Serial.println(F("socketOpen failed! Freezing..."));
      while (1)
        myEG25.bufferedPoll(); // Do nothing more except process any received data
    }

    Serial.print(F("Using socket "));
    Serial.println(socketNum);

    // Connect to the remote IP Address
    if (myEG25.socketConnect(socketNum, theAddress, TCP_PORT) != EG25_G_SUCCESS)
    {
      Serial.println(F("socketConnect failed! Freezing..."));
      while (1)
        myEG25.bufferedPoll(); // Do nothing more except process any received data
    }
    else
    {
      Serial.println(F("Socket connected!"));
    }

    // Send the first ping to start the ping-pong
    const char ping[] = "Ping";
    myEG25.socketWrite(socketNum, ping); // Send the "Ping"
        
  }

    
// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  else // if (selection == 2) // Pong
  {
    iAmPing = false;
    
    // Open the socket
    socketNum = myEG25.socketOpen(EG25_G_TCP);
    if (socketNum == -1)
    {
      Serial.println(F("socketOpen failed! Freezing..."));
      while (1)
        myEG25.bufferedPoll(); // Do nothing more except process any received data
    }

    Serial.print(F("Using socket "));
    Serial.println(socketNum);

    // Start listening for a connection
    if (myEG25.socketListen(socketNum, TCP_PORT) != EG25_G_SUCCESS)
    {
      Serial.println(F("socketListen failed! Freezing..."));
      while (1)
        myEG25.bufferedPoll(); // Do nothing more except process any received data
    }
  }

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  startTime = millis(); // Record that start time

}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void loop()
{
  myEG25.bufferedPoll(); // Process the backlog (if any) and any fresh serial data

  if (iAmPing) // Ping - close the socket when we've reached pingPongLimit
  {
    if (pingCount >= pingPongLimit)
    {
      printSocketParameters(socketNum);
      myEG25.socketClose(socketNum); // Close the socket - no more pings will be sent
      while (1)
        myEG25.bufferedPoll(); // Do nothing more except process any received data
    }
  }
  
  else // Pong - close the socket when we've reached the timeLimit
  {
    if (millis() > (startTime + timeLimit))
    {
      printSocketParameters(socketNum);
      myEG25.socketClose(socketNum); // Close the socket - no more pongs will be sent
      while (1)
        myEG25.bufferedPoll(); // Do nothing more except process any received data
    }
  }
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// Print the socket parameters
// Note: the socket must be open. ERRORs will be returned if the socket is closed.
void printSocketParameters(int socket)
{
  Serial.println(F("Socket parameters:"));
  
  Serial.print(F("Socket type: "));
  EG25_G_socket_protocol_t socketType;
  myEG25.querySocketType(socket, &socketType);
  if (socketType == EG25_G_TCP)
    Serial.println(F("TCP"));
  else if (socketType == EG25_G_UDP)
    Serial.println(F("UDP"));
  else
    Serial.println(F("UNKNOWN! (Error!)"));
  
  Serial.print(F("Last error: "));
  int lastError;
  myEG25.querySocketLastError(socket, &lastError);
  Serial.println(lastError);
  
  Serial.print(F("Total bytes sent: "));
  uint32_t bytesSent;
  myEG25.querySocketTotalBytesSent(socket, &bytesSent);
  Serial.println(bytesSent);
  
  Serial.print(F("Total bytes received: "));
  uint32_t bytesReceived;
  myEG25.querySocketTotalBytesReceived(socket, &bytesReceived);
  Serial.println(bytesReceived);
  
  Serial.print(F("Remote IP Address: "));
  IPAddress remoteAddress;
  int remotePort;
  myEG25.querySocketRemoteIPAddress(socket, &remoteAddress, &remotePort);
  Serial.print(remoteAddress[0]);
  Serial.print(F("."));
  Serial.print(remoteAddress[1]);
  Serial.print(F("."));
  Serial.print(remoteAddress[2]);
  Serial.print(F("."));
  Serial.println(remoteAddress[3]);

  Serial.print(F("Socket status (TCP sockets only): "));
  EG25_G_tcp_socket_status_t socketStatus;
  myEG25.querySocketStatusTCP(socket, &socketStatus);
  if (socketStatus == EG25_G_TCP_SOCKET_STATUS_INACTIVE)
    Serial.println(F("INACTIVE"));
  else if (socketStatus == EG25_G_TCP_SOCKET_STATUS_LISTEN)
    Serial.println(F("LISTEN"));
  else if (socketStatus == EG25_G_TCP_SOCKET_STATUS_SYN_SENT)
    Serial.println(F("SYN_SENT"));
  else if (socketStatus == EG25_G_TCP_SOCKET_STATUS_SYN_RCVD)
    Serial.println(F("SYN_RCVD"));
  else if (socketStatus == EG25_G_TCP_SOCKET_STATUS_ESTABLISHED)
    Serial.println(F("ESTABLISHED"));
  else if (socketStatus == EG25_G_TCP_SOCKET_STATUS_FIN_WAIT_1)
    Serial.println(F("FIN_WAIT_1"));
  else if (socketStatus == EG25_G_TCP_SOCKET_STATUS_FIN_WAIT_2)
    Serial.println(F("FIN_WAIT_2"));
  else if (socketStatus == EG25_G_TCP_SOCKET_STATUS_CLOSE_WAIT)
    Serial.println(F("CLOSE_WAIT"));
  else if (socketStatus == EG25_G_TCP_SOCKET_STATUS_CLOSING)
    Serial.println(F("CLOSING"));
  else if (socketStatus == EG25_G_TCP_SOCKET_STATUS_LAST_ACK)
    Serial.println(F("LAST_ACK"));
  else if (socketStatus == EG25_G_TCP_SOCKET_STATUS_TIME_WAIT)
    Serial.println(F("TIME_WAIT"));
  else
    Serial.println(F("UNKNOWN! (Error!)"));
      
  Serial.print(F("Unacknowledged outgoing bytes: "));
  uint32_t bytesUnack;
  myEG25.querySocketOutUnackData(socket, &bytesUnack);
  Serial.println(bytesUnack);
}
