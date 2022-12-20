#include <SparkFun_UHF_RFID_Reader.h>
#include <SoftwareSerial.h>

SoftwareSerial softSerial(11, 3); //RX, TX

RFID nano;  // RFID instance

/*************************************
* Useful Definitions
*************************************/
#define FALSE 0
#define TRUE  1

#define LEFT 0
#define RIGHT 1

#define MTR_BIT  0
#define HOME_BIT 1
#define ROOM_BIT 2
#define SIDE_BIT 3
#define Z_BIT    4
#define OBJ_BIT  5

#define RFID_2_MEGA 50

/*************************************
* LED Pin Numbers
*************************************/
int LED1 = 23;
int LED2 = 25;
int LED3 = 27;
int LED4 = 29;
int LED_STATUS = 31;

/*************************************
* RFID Tag Assignments 
*************************************/
byte room3002[] = {226,0,0,23,2,5,0,101,23,0,105,136}; // Room 1
byte room3004[] = {226,0,0,23,2,5,0,102,23,0,105,128}; // Room 2
byte room3006[] = {226,0,0,23,2,5,0,103,23,0,105,117}; // Room 3
byte room3008[] = {226,0,0,23,2,5,0,104,23,0,105,109}; // Room 4
byte room3010[] = {226,0,0,23,2,5,0,105,23,0,105,118}; // Room 5
byte room3012[] = {226,0,0,23,2,5,0,89,23,0,105,133};  // Room 6

byte room3014[] = {226,0,0,23,2,5,0,88,23,0,105,144};  // Room 7
byte room3016[] = {226,0,0,23,2,5,0,87,23,0,105,152};  // Room 8
byte room3018[] = {226,0,0,23,2,5,0,86,23,0,105,143};  // Room 9

byte room3001[]= {226,0,0,23,2,5,0,85,23,0,105,151};   // Room 10
byte room3003[]= {226,0,0,23,2,5,0,100,23,0,105,127};  // Room 11
byte room3005[]= {226,0,0,23,2,5,0,99,23,0,105,135};   // Room 12
byte room3007[]= {226,0,0,23,2,5,0,98,23,0,105,126};   // Room 13
byte roomHOME[]= {226,0,0,23,2,5,0,97,23,0,105,134};   // Room 14

/*************************************
* RFID Flags 
*************************************/
byte SETUP    = TRUE;   // GUI setup flag
byte DELIVERY = FALSE;  // Delivery in progress
byte AT_ROOM  = FALSE;  // Room flag
byte RETURN   = FALSE;  // Return home flag
byte AT_HOME  = TRUE;   // Home flag

byte RCV_MSG = FALSE;

/*************************************
* RFID Variables 
*************************************/
byte homeRoom = 0x20; // Home location

byte currentRoom = homeRoom;  // Initial location is home
byte desiredRoom = 0x00;      // Desired room set by GUI
byte prevRoom = 0x00;         

byte rx_msg[2] = {0x00};
byte tx_msg[2] = {0x00};
byte temp_msg = 0x00;

byte roomDirection;           // Direction of room from home
byte roomSide = 0;            // Desired room wall side

byte myEPC[12]; // Most EPCs are 12 bytes

void setup() {
  Serial.begin(115200);   // RFID reader port
  Serial1.begin(115200);  // Main Arduino port

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);

  for(int i = 0; i < 5; i++)
  {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, HIGH);
    digitalWrite(LED4, HIGH);
    digitalWrite(LED_STATUS, HIGH);
    delay(50);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
    digitalWrite(LED_STATUS, LOW);
    delay(50);
  }

  if(setupNano(38400) == false)  // Configure nano to run at 38400bps
    while (1); //Freeze!

  nano.setRegion(REGION_NORTHAMERICA);
  nano.setReadPower(1750); // 17.5 dBm
}

void loop() 
{
  /***** Clear rx/tx messages and flags *****/
  temp_msg = 0x00;
  
  for(int i = 0; i < 2; i++)
  {
    rx_msg[i] = 0x00;
    tx_msg[i] = 0x00;
  }

  currentRoom = homeRoom;
  desiredRoom = 0x00;
  prevRoom = 0x00;

  roomDirection = 0;
  roomSide = 0;

  for(int i = 0; i < 12; i++)
    myEPC[i] = 0;

  SETUP    = TRUE;
  DELIVERY = FALSE;
  AT_ROOM  = FALSE;
  RETURN   = FALSE;
  AT_HOME  = TRUE;

  RCV_MSG = FALSE;

  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  digitalWrite(LED_STATUS, LOW);
  /*****************************************/
  
  while(SETUP == TRUE)  // Wait for GUI input...
  {
    waitLED(1, 200);  // Wait for room number...
    
    // Read from Port 1 (MEGA)
    if(Serial1.available() > 0) {
      rx_msg[0] = Serial1.read();  // Upper byte stores room #
      delay(1);
      rx_msg[1] = Serial1.read();  // Lower byte stores bit checks

      RCV_MSG = TRUE;
    }

    // GUI input rx'd
    if(RCV_MSG == TRUE)
    {
      digitalWrite(LED1, HIGH);   // Room number received!
      delay(200);

      desiredRoom = rx_msg[0];          // Set desired room to the rx'd room #
      checkRoomLocation(desiredRoom);   // Check direction of room from base & send info to mega

      temp_msg = tx_msg[1];

      // Set room location (which side of hallway)
      if(bitRead(temp_msg, SIDE_BIT) == LEFT)
        roomSide = LEFT;
      else
        roomSide = RIGHT;

      SETUP = FALSE;
      DELIVERY = TRUE;
    }
  }

  while(DELIVERY == TRUE) // Wait to arrive at room...
  {
    waitLED(2, 200);
    scanRFIDs();  // Scan RFID's until reach desired room

    if(currentRoom == desiredRoom && desiredRoom != homeRoom)
    {
      digitalWrite(LED2, HIGH);  // Arrived at room! 
      delay(200);

      bitClear(temp_msg, MTR_BIT);  // Set motors off
      bitSet(temp_msg, ROOM_BIT);   // Desired room reached

      if(roomSide == LEFT)
        bitClear(temp_msg, Z_BIT);  // Turn to left
      else
        bitSet(temp_msg, Z_BIT);    // Turn to right

      tx_msg[1] = temp_msg;
      
      // RFID to Mega
      Serial1.write(tx_msg, 2);
      xmitLED(RFID_2_MEGA);

      DELIVERY = FALSE;
      AT_ROOM = TRUE;

      RCV_MSG = FALSE;
    }
    else
      prevRoom = currentRoom;
  }

  while(AT_ROOM == TRUE)  // Wait to complete delivery...
  {
    waitLED(3, 200);  // Wait to complete delivery...
    
    // Read from Port 1 (MEGA)
    if(Serial1.available() > 0)
    {
      rx_msg[0] = Serial1.read();  // Upper byte stores room #
      delay(1);
      rx_msg[1] = Serial1.read();  // Lower byte stores bit checks
    }

    if(rx_msg[0] == homeRoom) // Delivery complete!
    {
      digitalWrite(LED3, HIGH);  // Delivery complete! 
      delay(200);
      
      desiredRoom = rx_msg[0];
      tx_msg[0] = desiredRoom;

      bitSet(temp_msg, MTR_BIT);  // Turn motors on
      bitClear(temp_msg, ROOM_BIT);   // Clear room bit

      // Zero turn based off previous zero turn
      if(roomSide == LEFT)        
        bitClear(temp_msg, Z_BIT);
      else
        bitSet(temp_msg, Z_BIT);

      // Set side of home for wall following
      if(roomDirection == LEFT)
        bitClear(temp_msg, SIDE_BIT);
      else
        bitSet(temp_msg, SIDE_BIT);

      tx_msg[1] = temp_msg;
      
      // RFID to Mega
      Serial1.write(tx_msg, 2);
      xmitLED(RFID_2_MEGA);
      
      AT_ROOM = FALSE;
      RETURN = TRUE;
    }
  }

  while(RETURN == TRUE)   // Wait to arrive at home...
  {
    waitLED(4, 200);  // Wait to arrive at home...
    scanRFIDs();  // Scan RFID's until reach desired room

    if(currentRoom == desiredRoom && desiredRoom == homeRoom)
    {
      digitalWrite(LED4, HIGH);  // Arrived at room! 
      delay(200);

      bitClear(temp_msg, MTR_BIT);    // Set motors off
      bitSet(temp_msg, HOME_BIT);     // Home reached
      bitClear(temp_msg, ROOM_BIT);   // Desired room clear

      if(roomDirection == LEFT)
        bitClear(temp_msg, Z_BIT);
      else
        bitSet(temp_msg, Z_BIT);

      tx_msg[0] = homeRoom;
      tx_msg[1] = temp_msg;

      // RFID to Mega
      Serial1.write(tx_msg, 2);
      xmitLED(RFID_2_MEGA);

      RETURN = FALSE;
      AT_HOME = TRUE;

      if(AT_HOME == TRUE)    
        homeLED(); 
    }
    else
      prevRoom = currentRoom;
  }
}

void checkRoomLocation(byte des_room) {  
  if(des_room == 0x02 || des_room == 0x04 || des_room == 0x06 || des_room == 0x08 || des_room == 0x0A || des_room == 0x0C)
  {
    roomDirection = LEFT;

    tx_msg[0] = desiredRoom;
    
    bitSet(tx_msg[1], MTR_BIT);     // Set motors on
    bitClear(tx_msg[1], HOME_BIT);  // Leaving home
    bitClear(tx_msg[1], SIDE_BIT);  // Room on left side of hallway
    bitClear(tx_msg[1], Z_BIT);     // Set zero turn left 

    // RFID to Mega
    Serial1.write(tx_msg, 2);
    xmitLED(RFID_2_MEGA);
  }
  else
  {
    roomDirection = RIGHT;

    if(des_room == 0x01 || des_room == 0x03 || des_room == 0x05 || des_room == 0x07)
    {
      tx_msg[0] = desiredRoom;
      
      bitSet(tx_msg[1], MTR_BIT);     // Set motors on
      bitClear(tx_msg[1], HOME_BIT);  // Leaving home
      bitClear(tx_msg[1], SIDE_BIT);  // Room on left side of hallway
      bitSet(tx_msg[1], Z_BIT);       // Set zero turn right
      
      // RFID to Mega
      Serial1.write(tx_msg, 2);
      xmitLED(RFID_2_MEGA);
    }
    else
    {
      tx_msg[0] = desiredRoom;
      
      bitSet(tx_msg[1], MTR_BIT);     // Set motors on
      bitClear(tx_msg[1], HOME_BIT);  // Leaving home
      bitSet(tx_msg[1], SIDE_BIT);    // Room on right side of hallway
      bitSet(tx_msg[1], Z_BIT);       // Set zero turn right

      // RFID to Mega
      Serial1.write(tx_msg, 2);
      xmitLED(RFID_2_MEGA);
    }
  }    
}

void scanRFIDs() {
  byte myEPClength;
  byte responseType = 0;

  while(responseType != RESPONSE_SUCCESS) 
  {
    myEPClength = sizeof(myEPC);  // Modify EPC length each time .readTagEPC is called
    responseType = nano.readTagEPC(myEPC, myEPClength, 500); // Scan for a new tag up to 500ms
  }

  checkCurrentRoom(); // Check current room location
}

void checkCurrentRoom() {
  if(myEPC[7] == roomHOME[7] && myEPC[11] == roomHOME[11])
    currentRoom = homeRoom;
  else if(myEPC[7] == room3001[7] && myEPC[11] == room3001[11])
    currentRoom = 0x01;
  else if(myEPC[7] == room3002[7] && myEPC[11] == room3002[11])
    currentRoom = 0x02;
  else if(myEPC[7] == room3003[7] && myEPC[11] == room3003[11])
    currentRoom = 0x03;
  else if(myEPC[7] == room3004[7] && myEPC[11] == room3004[11])
    currentRoom = 0x04;
  else if(myEPC[7] == room3005[7] && myEPC[11] == room3005[11])
    currentRoom = 0x05;
  else if(myEPC[7] == room3006[7] && myEPC[11] == room3006[11])
    currentRoom = 0x06;
  else if(myEPC[7] == room3007[7] && myEPC[11] == room3007[11])
    currentRoom = 0x07;
  else if(myEPC[7] == room3008[7] && myEPC[11] == room3008[11])
    currentRoom = 0x08;
  else if(myEPC[7] == room3010[7] && myEPC[11] == room3010[11])
    currentRoom = 0x0A;
  else if(myEPC[7] == room3012[7] && myEPC[11] == room3012[11])
    currentRoom = 0x0C;
  else if(myEPC[7] == room3014[7] && myEPC[11] == room3014[11])
    currentRoom = 0x0E;
  else if(myEPC[7] == room3016[7] && myEPC[11] == room3016[11])
    currentRoom = 0x10;
  else if(myEPC[7] == room3018[7] && myEPC[11] == room3018[11])
    currentRoom = 0x12;
}

boolean setupNano(long baudRate) {
  nano.begin(softSerial); //Tell the library to communicate over software serial port

  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  softSerial.begin(baudRate); //For this test, assume module is already at our desired baud rate
  while(!softSerial); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while(softSerial.available()) softSerial.read();
  
  nano.getVersion();

  if (nano.msg[0] == ERROR_WRONG_OPCODE_RESPONSE)
  {
    //This happens if the baud rate is correct but the module is doing a ccontinuous read
    nano.stopReading();
    delay(1500);
  }
  else
  {
    //The module did not respond so assume it's just been powered on and communicating at 115200bps
    softSerial.begin(115200); //Start software serial at 115200

    nano.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

    softSerial.begin(baudRate); //Start the software serial port, this time at user's chosen baud rate
  }

  //Test the connection
  nano.getVersion();
  if (nano.msg[0] != ALL_GOOD) return (false); //Something is not right

  //The M6E has these settings no matter what
  nano.setTagProtocol(); //Set protocol to GEN2

  nano.setAntennaPort(); //Set TX/RX antenna ports to 1

  return (true); //We are ready to rock
}

void waitLED(int led, int t) {
  switch(led)
  {
    case 1:
      digitalWrite(LED1, HIGH);
      delay(t);
      digitalWrite(LED1, LOW);
      delay(t);
      break;

    case 2:
      digitalWrite(LED2, HIGH);
      delay(t);
      digitalWrite(LED2, LOW);
      delay(t);
      break;

    case 3:
      digitalWrite(LED3, HIGH);
      delay(t);
      digitalWrite(LED3, LOW);
      delay(t);
      break;

    case 4:
      digitalWrite(LED4, HIGH);
      delay(t);
      digitalWrite(LED4, LOW);
      delay(t);
      break;

    default:
      break;
  }
}

void xmitLED(int t) {
  for(int i = 0; i < 10; i++)
  {
    digitalWrite(LED_STATUS, HIGH);
    delay(t);
    digitalWrite(LED_STATUS, LOW);
    delay(t);
  }
}

void homeLED() {
  for(int i = 0; i < 5; i++)
  {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
    delay(25);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, LOW);
    delay(25);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, HIGH);
    digitalWrite(LED4, LOW);
    delay(25);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    digitalWrite(LED4, HIGH);
    delay(25);
  }

  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
  digitalWrite(LED4, HIGH);
  delay(2000);
}

