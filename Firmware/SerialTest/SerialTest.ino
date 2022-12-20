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

#define MEGA_2_RFID 50
#define MEGA_2_GUI  25

/*************************************
* Communication Flags 
*************************************/
byte SETUP    = TRUE;     // Receiving room location from GUI
byte DELIVERY = FALSE;    // Delivery in progress
byte AT_ROOM  = FALSE;    // At desired room
byte RETURN   = FALSE;    // Returning to home in progress
byte AT_HOME  = FALSE;    // At home

byte RCV_MSG = FALSE;
byte ROOM_SET = FALSE; 

/*************************************
* Communication Variables 
*************************************/
byte rx_msg[2] = {0x00};
byte tx_msg[2] = {0x00};
byte temp_msg[2] = {0x00};

/*************************************
* LED Pin Numbers
*************************************/
int LED1 = 12;
int LED2 = 11;
int LED3 = 8;
int LED4 = 7;
int LED_STATUS = 6;

void setup() {
  Serial.begin(115200);   // Java port
  Serial1.begin(115200);  // RFID Arduino port

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
}

void loop() 
{
  /***** Clear rx/tx messages and flags *****/
  for(int i = 0; i < 2; i++)
  {
    rx_msg[i] = 0x00;
    tx_msg[i] = 0x00;
  }

  byte SETUP    = TRUE;
  byte DELIVERY = FALSE;
  byte AT_ROOM  = FALSE;
  byte RETURN   = FALSE;
  byte AT_HOME  = TRUE;

  RCV_MSG = FALSE;
  ROOM_SET = FALSE; 

  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  digitalWrite(LED_STATUS, LOW);
  /*****************************************/
  
  while(SETUP == TRUE)  // Wait for GUI input...
  {
    waitLED(1, 500);  // Wait for room number...
    
    // Read from Port 0 (GUI)
    if(Serial.available() > 0) {
      rx_msg[0] = Serial.read();
      delay(1);
      rx_msg[1] = Serial.read();

      for(int i = 0; i < 2; i++)
        tx_msg[i] = rx_msg[i];  // Set tx_msg to rx_msg

      RCV_MSG = TRUE; 
    }

    // GUI input rx'd
    if(RCV_MSG == TRUE) 
    {
      digitalWrite(LED1, HIGH);   // Room number received!
      delay(1000);

      // Clear rx_msg
      for(int i = 0; i < 2; i++)
        rx_msg[i] = 0x00;

      // Mega to RFID
      Serial1.write(tx_msg, 2);
      xmitLED(MEGA_2_RFID);

      // Clear tx_msg
      for(int i = 0; i < 2; i++)
        tx_msg[i] = 0x00;

      while(ROOM_SET == FALSE) // Wait for room location info...
      {
        // Read from Port 1 (RFID)
        if(Serial1.available() > 0) {
          rx_msg[0] = Serial1.read();
          delay(1);
          rx_msg[1] = Serial1.read();

          ROOM_SET = TRUE; // Room location information rcv'd from RFID Arduino
        }

        for(int i = 0; i < 2; i++)
          temp_msg[i] = rx_msg[i];  // Set temp_msg to rx_msg
        for(int i = 0; i < 2; i++)
          rx_msg[i] = 0x00;         // Clear rx_msg

        // RFID confirmed room info
        if(ROOM_SET == TRUE)
        {          
          SETUP = FALSE;    // Setup complete! Begin delivery 
          DELIVERY = TRUE;  

          // Move to desired wall distance
          if( (bitRead(temp_msg[1], MTR_BIT) == 1) && (bitRead(temp_msg[1], HOME_BIT) == 0) )
          {
            // Motor forward to move away from wall
//            while(disR < 10 && disL < 10)
//            {
//              motorStraight();
//            }
//            // Motor stop
//            motorStop();
          }

          // Zero turn 90 deg based on room direction
//          if(bitRead(temp_msg[1], Z_BIT) == LEFT)
//             //zeroTurn(LEFT, 90);
//          else if(bitRead(temp_msg[1], Z_BIT) == RIGHT){}
//             //zeroTurn(RIGHT, 90);

          //motorStop();
        }
      }
    }
  }
  
  while(DELIVERY == TRUE) // Wait to arrive at room...
  {
//    if(bitRead(temp_msg[1], Z_BIT) == LEFT)
//    {
//      wallControl(LEFT);
//      avoidObjects();
//    }
//    else
//    {
//      wallControl(RIGHT);
//      avoidObjects();
//    }

    
    waitLED(2, 500);  // Wait to arrive at room...
    
    // Read from Port 1 (RFID)
    if(Serial1.available() > 0) {
      rx_msg[0] = Serial1.read();
      delay(1);
      rx_msg[1] = Serial1.read();

      if(rx_msg[0] == temp_msg[0])
      {
        DELIVERY = FALSE; // Desired room reached
        AT_ROOM = TRUE;
      }
    }

    if(DELIVERY == FALSE && AT_ROOM == TRUE)
    {
      digitalWrite(LED2, HIGH);   // Arrived at room!
      delay(1000);

      for(int i = 0; i < 2; i++)
        temp_msg[i] = rx_msg[i];  // Set temp_msg to rx_msg
      for(int i = 0; i < 2; i++)
        rx_msg[i] = 0x00;         // Clear rx_msg

      // Signifies room reached
      if(bitRead(temp_msg[1], MTR_BIT) == 0 && bitRead(temp_msg[1], ROOM_BIT) == 1)
      {
//        motorStop();
//
//        if(bitRead(temp_msg[1], Z_BIT) == LEFT)
//          zeroTurn(LEFT, 90);
//        else
//          zeroTurn(RIGHT, 90);
      }
    
      for(int i = 0; i < 2; i++)
        tx_msg[i] = temp_msg[i];    // Set temp_msg to rx_msg
      for(int i = 0; i < 2; i++)
        temp_msg[i] = 0x00;         // Clear rx_msg
  
      // Mega to GUI
      Serial.print(tx_msg[0], HEX);
      Serial.println(tx_msg[1], HEX);  
      xmitLED(MEGA_2_GUI);
    }
  }

  while(AT_ROOM == TRUE)  // Wait to complete delivery...
  {
    waitLED(3, 500);  // Wait to complete delivery...
   
    // Read from Port 0 (Java)
    if(Serial.available() > 0)
    {
      rx_msg[0] = Serial.read();
      delay(1);
      rx_msg[1] = Serial.read();

      for(int i = 0; i < 2; i++)
        tx_msg[i] = rx_msg[i];  // Set tx_msg to rx_msg 
    }

    if(rx_msg[0] == 0x20) // Home room number received!
    {
      digitalWrite(LED3, HIGH);   // Home room number received!
      delay(1000);
      
      for(int i = 0; i < 2; i++)
        rx_msg[i] = 0x00; // Clear rx_msg

      // Mega to RFID
      Serial1.write(tx_msg, 2);
      xmitLED(MEGA_2_RFID);

      for(int i = 0; i < 2; i++)
        tx_msg[i] = 0x00;  // Clear tx_msg

      ROOM_SET = FALSE;

      while(ROOM_SET == FALSE)
      {       
        // Read from Port 1 (RFID)
        if(Serial1.available() > 0)
        {
          rx_msg[0] = Serial1.read();
          delay(1);
          rx_msg[1] = Serial1.read();

          ROOM_SET = TRUE;
        }

        for(int i = 0; i < 2; i++)
          temp_msg[i] = rx_msg[i];  // Set temp_msg to rx_msg
        for(int i = 0; i < 2; i++)
          rx_msg[i] = 0x00;         // Clear rx_msg

        if(ROOM_SET == TRUE)
        {
          AT_ROOM = FALSE;
          RETURN = TRUE;

          // Return home
          if( (bitRead(temp_msg[1], MTR_BIT) == 1) && (bitRead(temp_msg[1], ROOM_BIT) == 0) )
          {
//            if(bitRead(temp_msg[1], Z_BIT) == LEFT)
//              zeroTurn(LEFT, 90);
//            else 
//              zeroTurn(RIGHT, 90);
          }
        }
      }
    }
  }

  while(RETURN == TRUE) // Wait to arrive at home...
  {
//    if(bitRead(temp_msg[1], SIDE_BIT) == LEFT)
//    {
//      wallControl(LEFT);
//      avoidObjects();
//    }
//    else
//    {
//      wallControl(RIGHT);
//      avoidObjects();
//    }
    
    waitLED(4, 500);  // Wait to arrive at home...

    // Read from Port 1 (RFID)
    if(Serial1.available() > 0) {
      rx_msg[0] = Serial1.read();
      delay(1);
      rx_msg[1] = Serial1.read();

      if(rx_msg[0] == temp_msg[0])
      {
        RETURN = FALSE;
        AT_HOME = TRUE;
      }
    }

    if(RETURN == FALSE && AT_HOME == TRUE)  // Arrived at home!
    {
      digitalWrite(LED4, HIGH);   // Arrived at home!
      delay(1000);

      for(int i = 0; i < 2; i++)
        temp_msg[i] = rx_msg[i];    // Set temp_msg to rx_msg
      for(int i = 0; i < 2; i++)
        rx_msg[i] = 0x00;         // Clear rx_msg

      if( (bitRead(temp_msg[1], MTR_BIT) == 0) && (bitRead(temp_msg[1], HOME_BIT) == 1) )
      {
        // Zero turn 90 deg based on room direction
//        if(bitRead(temp_msg[1], Z_BIT) == LEFT)
//           //zeroTurn(LEFT, 90);
//        else if(bitRead(temp_msg[1], Z_BIT) == RIGHT){}
//           //zeroTurn(RIGHT, 90);

//        while(disR > -10 && disL > -10)
//            {
//              motorReverse();
//            }
//            // Motor stop
//            motorStop();
      }
    
      for(int i = 0; i < 2; i++)
        tx_msg[i] = temp_msg[i];    // Set temp_msg to rx_msg
      for(int i = 0; i < 2; i++)
        temp_msg[i] = 0x00;         // Clear rx_msg
  
      // Mega to GUI
      Serial.print(tx_msg[0], HEX);
      Serial.println(tx_msg[1], HEX);  
      xmitLED(MEGA_2_GUI);

      delay(500);
      homeLED();
    }
  }
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

