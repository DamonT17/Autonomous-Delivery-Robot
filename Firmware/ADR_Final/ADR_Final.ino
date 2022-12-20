/*************************************
* Useful Definitions
*************************************/
#define FALSE 0
#define TRUE  1

#define LEFT 0
#define RIGHT 1
#define FRONT 2

#define LEFT_FWD HIGH
#define LEFT_REV LOW
#define RIGHT_FWD LOW
#define RIGHT_REV HIGH

#define PWM_MIN 50
#define PWM_MAX 255

#define SAMPLE_SIZE 15

#define MTR_BIT  0
#define HOME_BIT 1
#define ROOM_BIT 2
#define SIDE_BIT 3
#define Z_BIT    4
#define OBJ_BIT  5

#define MEGA_2_RFID 50
#define MEGA_2_GUI  25

/*************************************
* Motor Pins
*************************************/
#define LEFT_MTR 10   // Good
#define LEFT_DIR A0   // Good
#define RIGHT_MTR 7   // Fingers crossed
#define RIGHT_DIR A7  // Fingers crossed

/*************************************
* Encoder Pins
*************************************/
#define LEFT_ENC_A 4    
#define LEFT_ENC_B 3    // Tie to interrupt 1
#define RIGHT_ENC_A 2   // Tie to interrupt 0
#define RIGHT_ENC_B 5

/*************************************
* Object Detection Pins
*************************************/
#define trigPin1 27                        
#define echoPin1 26                        
#define trigPin2 31                     
#define echoPin2 30                        
#define trigPin3 35                       
#define echoPin3 34 

/*************************************
* Wall Detection Pins
*************************************/
#define leftTrig1 23
#define leftEcho1 22
#define rightTrig1 47
#define rightEcho1 46

/*************************************
* LED Pin Numbers
*************************************/
int LED1 = 12;
int LED2 = 11;
int LED3 = 42;
int LED4 = 40;
int LED_STATUS = 38;

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
* Object Flags 
*************************************/
byte CLEARED_OBJ = FALSE;
byte OBJ = -1;

/********Encoder Interrupt Variables********/
long volatile posL = 0;
int lastPosL = LOW;
long volatile degL;
float volatile disL;
int n = LOW;

long volatile posR = 0;
int lastPosR = LOW;
long volatile degR;
float volatile disR;
int m = LOW;

/********Wheel Variables********/
float diamWheel = 15.24;      // 6 in. wheel [cm]
float radWheel = diamWheel/2;

float L = 40.64;  // Distance between wheels [cm]
float circWheel;  // Wheel circumference

/********Speed Variables********/
float newPosL;
float oldPosL;
float vLeft;
float uLeft;

float newPosR;
float oldPosR;
float vRight;
float uRight;

float newTime;
float oldTime;

float T = 0.2;  // Sample time
float w;
float theta = 0;
float R;
float v;
float th;

/********Motor Control Variables********/
int pwmLeft;            // Left motor PWM value
int pwmRight;           // Right motor PWM value

/********Wall Control Variables********/
float KpW = 1; float KiW = 0.1; float KdW = 2.1;

float errWall;             // Proportional Error
int errSumW;            // Sum of errors (Integral)

float controlWall;         // Controller
float setWall = 40;
float measWall;

float hypotnuse;

/********Angle Control Variables********/
float KpA = 2; float KiA = 0.08; float KdA= 2.7;

float angleError;             // Proportional Error
int angleErrorSum;            // Sum of errors (Integral)
float angleErrorD;
float lastAngleError;

float controlAngle;         // Controller
float setAngle = 0;
float measAngle;

/********Object Avoidance Control Variables********/
float KpO = 2;
float KiO = 0.0125;   // Consider placing these as function parameters...

float KpOC = 0.35;     // Center plane PID
float KiOC = 0.0125;

float setPlane1 = 60;
float measPlane1;
float errPlane1;
int errSumPlane1;

float setPlane2 = 60;
float measPlane2;
float errPlane2;
int errSumPlane2;

float setPlane3 = 60;
float measPlane3;
float errPlane3;
int errSumPlane3;
  
float controlObj;

/********Object Detection Variables********/
long duration1;
int distance1;
long duration2;
int distance2;
long duration3;
int distance3;

int maxDistance1;
int maxDistance2;
int maxDistance3; 

int minLeft;
int minCenter;
int minRight;

/********Wall Detection Variables********/
long leftDuration1;
int leftDistance1;
int leftMax1;

long rightDuration1;
int rightDistance1;
int rightMax1;

int minLeftW;
int minRightW;

int setDis;

void setup() { 
  Serial.begin(115200);   // Arduino to Java connection
  Serial1.begin(115200);  // Arduino to RFID connection

  /****Motor/Encoder Setup****/
  attachMotors();
  attachEncoders();

  attachInterrupt(0, rightEncoder, CHANGE);
  attachInterrupt(1, leftEncoder, CHANGE);

  /****Ultrasonic Setup****/
  attachUltrasonics();

  /****LED Setup****/
  attachLED();
  
  motorStop();
  delay(2000);
}

void loop()
{
  /***** Clear rx/tx messages and flags *****/
  for(int i = 0; i < 2; i++)
  {
    rx_msg[i] = 0x00;
    tx_msg[i] = 0x00;
    temp_msg[i] = 0x00;
  }

  byte SETUP    = TRUE;
  byte DELIVERY = FALSE;
  byte AT_ROOM  = FALSE;
  byte RETURN   = FALSE;
  byte AT_HOME  = TRUE;

  RCV_MSG = FALSE;
  ROOM_SET = FALSE; 

  setDis = 0;

  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  digitalWrite(LED_STATUS, LOW);
  /*****************************************/
  
  while(SETUP == TRUE)  // Wait for GUI input...
  {
    waitLED(1, 200);  // Wait for room number...
    
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
      delay(200);

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
            // Motor stop
            motorStop();

            // Zero turn 90 deg based on room direction
            if(bitRead(temp_msg[1], Z_BIT) == LEFT)
               zeroTurn(LEFT, 90);
            else if(bitRead(temp_msg[1], Z_BIT) == RIGHT)
               zeroTurn(RIGHT, 90);
          }
        }
      }
    }
  }
  
  while(DELIVERY == TRUE) // Wait to arrive at room...
  {
    angleControl();

    waitLED(2, 200);  // Wait to arrive at room...
    
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
      delay(200);

      for(int i = 0; i < 2; i++)
        temp_msg[i] = rx_msg[i];  // Set temp_msg to rx_msg
      for(int i = 0; i < 2; i++)
        rx_msg[i] = 0x00;         // Clear rx_msg

      // Signifies room reached
      if(bitRead(temp_msg[1], MTR_BIT) == 0 && bitRead(temp_msg[1], ROOM_BIT) == 1)
      {
        motorStop();

        if(bitRead(temp_msg[1], Z_BIT) == LEFT)
          zeroTurn(LEFT, 90);
        else
          zeroTurn(RIGHT, 90);
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
    waitLED(3, 200);  // Wait to complete delivery...
   
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
      delay(200);
      
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
            if(bitRead(temp_msg[1], Z_BIT) == LEFT)
              zeroTurn(LEFT, 90);
            else 
              zeroTurn(RIGHT, 90);
          }
        }
      }
    }
  }

  while(RETURN == TRUE) // Wait to arrive at home...
  {
    angleControl();
    
    waitLED(4, 200);  // Wait to arrive at home...

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
      delay(200);

      for(int i = 0; i < 2; i++)
        temp_msg[i] = rx_msg[i];    // Set temp_msg to rx_msg
      for(int i = 0; i < 2; i++)
        rx_msg[i] = 0x00;         // Clear rx_msg

      if( (bitRead(temp_msg[1], MTR_BIT) == 0) && (bitRead(temp_msg[1], HOME_BIT) == 1) )
      {
        // Zero turn 90 deg based on room direction
        if(bitRead(temp_msg[1], Z_BIT) == LEFT)
          zeroTurn(LEFT, 90);
        else if(bitRead(temp_msg[1], Z_BIT) == RIGHT)
          zeroTurn(RIGHT, 90);
       
        // Motor stop
        motorStop();
      }
    
      for(int i = 0; i < 2; i++)
        tx_msg[i] = temp_msg[i];    // Set temp_msg to rx_msg
      for(int i = 0; i < 2; i++)
        temp_msg[i] = 0x00;         // Clear rx_msg
  
      // Mega to GUI
      Serial.print(tx_msg[0], HEX);
      Serial.println(tx_msg[1], HEX);  
      xmitLED(MEGA_2_GUI);

      homeLED();
    }
  }
}

void attachMotors() { 
  pinMode(LEFT_MTR, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_MTR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
}

void attachEncoders() {   
  pinMode(LEFT_ENC_A, INPUT);      // Left encoder pin A
  digitalWrite(LEFT_ENC_A, HIGH);
  pinMode(LEFT_ENC_B, INPUT);      // Left encoder pin B
  digitalWrite(LEFT_ENC_B, HIGH);

  pinMode(RIGHT_ENC_A, INPUT);      // Right encoder pin A
  digitalWrite(RIGHT_ENC_A, HIGH);
  pinMode(RIGHT_ENC_B, INPUT);      // Right encoder pin B
  digitalWrite(RIGHT_ENC_B, HIGH);
}

void attachUltrasonics() {
  pinMode(leftTrig1, OUTPUT);   // Left side
  pinMode(leftEcho1, INPUT);

  pinMode(rightTrig1, OUTPUT);  // Right side
  pinMode(rightEcho1, INPUT);

  pinMode(trigPin1, OUTPUT);    // Front side
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
}

void attachLED() {
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

void leftEncoder() {
  n = digitalRead(LEFT_ENC_A);   // Read status of Pin A
  
  if((lastPosL == LOW) && (n == HIGH)) 
  {
    if(digitalRead(LEFT_ENC_B) == LOW) 
    {
      posL--;   // Decrement position counter if reverse  
    } 
    else 
    {
      posL++;  // Increment position counter if forward
    }

    degL = posL*0.17143;         // Degrees conversion

    circWheel = PI*diamWheel;
    disL = -((degL*circWheel)/360);  // Distance [cm]
    // Wheel diameter = 6 inches
  }  
  lastPosL = n;  // Set the last value of Pin A to state of 'n'
}

void rightEncoder() {
  m = digitalRead(RIGHT_ENC_A);   // Read status of Pin A
  
  if((lastPosR == LOW) && (m == HIGH)) 
  {
    if(digitalRead(RIGHT_ENC_B) == LOW) 
    {
      posR--;   // Decrement position counter if reverse  
    } 
    else 
    {
      posR++;  // Increment position counter if forward
    }

    degR = posR*0.17143;         // Degrees conversion

    circWheel = PI*diamWheel;
    disR = -((degR*circWheel)/360);  // Distance [cm]
    // Wheel diameter = 6 inches
  }  
  lastPosR = m;  // Set the last value of Pin A to state of 'n'
}

void motorStop() {
    digitalWrite(LEFT_MTR, LOW);
    digitalWrite(LEFT_DIR, LOW);
    digitalWrite(RIGHT_MTR, LOW);
    digitalWrite(RIGHT_DIR, LOW);

    posL = 0; disL = 0;
    posR = 0; disR = 0;
}

void zeroTurn(int direct, float angle) {
  float diameter = 40.64; // Robot diameter [cm]
  
  motorStop();

  if(direct == LEFT)
  {
    posL = 0; disL = 0;
    posR = 0; disR = 0;
    
    while(disR <= (angle/360)*PI*diameter)
    {
      forward(RIGHT_MTR, RIGHT_FWD, 120);
      reverse(LEFT_MTR, LEFT_REV, 120);
    }

    motorStop();

    posL = 0; disL = 0;
    posR = 0; disR = 0;
  }
  else if(direct == RIGHT)
  {
    posL = 0; disL = 0;
    posR = 0; disR = 0;
    
    while(abs(disR) <= (angle/360)*PI*diameter)
    {
      forward(LEFT_MTR, LEFT_FWD, 120);
      reverse(RIGHT_MTR, RIGHT_REV, 120);
    }

    motorStop();
    delay(250);

    posL = 0; disL = 0;
    posR = 0; disR = 0;
  }
}

void forward(uint8_t mtr, uint8_t dir, int val) {
  if(mtr == LEFT_MTR)
  {  
    analogWrite(LEFT_MTR, val);
    digitalWrite(LEFT_DIR, dir);
  }
  else if(mtr == RIGHT_MTR)
  {
    analogWrite(RIGHT_MTR, val);
    digitalWrite(RIGHT_DIR, dir);
  }
}

void reverse(uint8_t mtr, uint8_t dir, int val) {
  if(mtr == LEFT_MTR)
  {
    analogWrite(LEFT_MTR, val);
    digitalWrite(LEFT_DIR, dir);
  }
  else if(mtr == RIGHT_MTR)
  {
    analogWrite(RIGHT_MTR, val);
    digitalWrite(RIGHT_DIR, dir);
  }
}

void pwmLimit() { 
  if(pwmLeft >= PWM_MAX)  {pwmLeft = PWM_MAX;}
  if(pwmLeft <= PWM_MIN)  {pwmLeft = PWM_MIN;}
  if(pwmRight >= PWM_MAX) {pwmRight = PWM_MAX;}
  if(pwmRight <= PWM_MIN) {pwmRight = PWM_MIN;}
}

void motorSnO() {
  newPosL = disL;
  newPosR = disR;
  
  newTime = millis(); 

  // Wheel linear velocity [cm/s]
  vLeft = 1000*(newPosL - oldPosL)/(newTime - oldTime);   
  vRight = 1000*(newPosR - oldPosR)/(newTime - oldTime);

  // Wheel angular velocity [deg/s]
  uLeft = vLeft/radWheel;
  uRight = vRight/radWheel;

  // Robot angular velocity [deg/s]
  w = (radWheel/L)*(uRight - uLeft);

  // Instantaneous center of curvature
//  R = ((L/2)*(vLeft + vRight))/(vRight - vLeft);

  // Robot linear velocity
//  v = R*w;

  // Robot angle [deg]
  th += w;
  theta = (th*T)*(180/PI);

  oldPosL = newPosL;
  oldPosR = newPosR;
  
  oldTime = newTime;
}

void angleControl() {
  analogWrite(LEFT_MTR, pwmLeft);
  digitalWrite(LEFT_DIR, LEFT_FWD);
  analogWrite(RIGHT_MTR, pwmRight);
  digitalWrite(RIGHT_DIR, RIGHT_FWD);
  
  motorSnO();
  Serial.println(theta);

  planeDistance(3);
  
  if(minLeft <= 60 || minCenter <= 60 || minRight <= 60)
    avoidObjects();
  
  if(CLEARED_OBJ == TRUE)
  {
    wallDistance(RIGHT, 3);
    measWall =  minRightW;
    errWall = setWall - measWall;
    
    if(errWall > 0)
    {
      hypotnuse = (errWall + 17.24)/cos((30*PI)/180);
      
      zeroTurn(LEFT, 30);
      motorStop();
      delay(200);
      
      while(disR <= hypotnuse)
      {
        forward(LEFT_MTR, LEFT_FWD, 145);
        forward(RIGHT_MTR, RIGHT_FWD, 140);
      }
      
      motorStop();
      delay(200);
      zeroTurn(RIGHT, 35);
    }
    else if(errWall < 0)
    {
      hypotnuse = abs((errWall - 17.24)/cos((35*PI)/180));
      
      zeroTurn(RIGHT, 35);
      motorStop();
      delay(200);
      
      while(disR <= hypotnuse)
      {
        forward(LEFT_MTR, LEFT_FWD, 145);
        forward(RIGHT_MTR, RIGHT_FWD, 140);
      }
      
      motorStop();
      delay(200);
      zeroTurn(LEFT, 30);
    }

    CLEARED_OBJ = FALSE;
    th = 0;
    theta = 0;
    angleErrorSum = 0;
  }

  measAngle = theta;
  angleError = setAngle - measAngle;
    
  angleErrorSum = angleErrorSum + angleError;
  angleErrorD = angleError - lastAngleError;

  controlAngle += KpA*angleError + KiA*angleErrorSum + KdA*angleErrorD;

  if(measAngle < setAngle)
  {
    pwmLeft = 140 - abs(controlAngle) + 5;
    pwmRight = 140 + abs(controlAngle) - 5;

    pwmLimit();
  }
  else if(measAngle > setAngle)
  {
    pwmLeft = 140 + abs(controlAngle) + 5;
    pwmRight = 140 - abs(controlAngle) - 5;

    pwmLimit();
  }
  else
  {
    pwmLeft = 140 + 5;
    pwmRight = 140 - 5;
  }

  lastAngleError = angleError;
}

void avoidLeft() {
  measPlane1 = minLeft;
  errPlane1 = setPlane1 - measPlane1;
  errSumPlane1 = errSumPlane1 + errPlane1;

  controlObj = KpO*errPlane1 + KiO*errSumPlane1;

  pwmLeft = pwmLeft + abs(controlObj);
  pwmRight = pwmRight - abs(controlObj);

  pwmLimit();
}

void avoidCL() {
  measPlane2 = minCenter;
  errPlane2 = setPlane2 - measPlane2;
  errSumPlane2 = errSumPlane2 + errPlane2;

  controlObj = KpOC*errPlane2 + KiOC*errSumPlane2;

  pwmLeft = pwmLeft + abs(controlObj);
  pwmRight = pwmRight - abs(controlObj);

  pwmLimit();
}

void avoidCenter() {
  if(minLeft < minRight && minRightW > 50)
  {
    avoidCL();
    T = 0.3;
    OBJ = LEFT;
  }
  else if(minLeft > minRight && minLeftW > 50)
  {
    avoidCR();
    T = 0.3;
    OBJ = RIGHT;
  }
}

void avoidCR() {
  measPlane2 = minCenter;
  errPlane2 = setPlane2 - measPlane2;
  errSumPlane2 = errSumPlane2 + errPlane2;

  controlObj = KpOC*errPlane2 + KiOC*errSumPlane2;

  pwmLeft = pwmLeft - abs(controlObj);
  pwmRight = pwmRight + abs(controlObj);

  pwmLimit();
}

void avoidRight() {
  measPlane3 = minRight;
  errPlane3 = setPlane3 - measPlane3;
  errSumPlane3 = errSumPlane3 + errPlane3;

  controlObj = KpO*errPlane3 + KiO*errSumPlane3;

  pwmLeft = pwmLeft - abs(controlObj);
  pwmRight = pwmRight + abs(controlObj);

  pwmLimit();
}

void avoidObjects() {
  Serial.println("OBJECT");
  
  while(minLeft <= 60 || minCenter <= 60 || minRight <= 60)
  {        
    planeDistance(1);
    if(minLeft > 60 && minCenter > 60 && minRight > 60)
    {
      CLEARED_OBJ = TRUE;
      break;
    }

    wallDistance(LEFT, 1);
    wallDistance(RIGHT, 1);

    analogWrite(LEFT_MTR, pwmLeft);
    digitalWrite(LEFT_DIR, LEFT_FWD);
    analogWrite(RIGHT_MTR, pwmRight);
    digitalWrite(RIGHT_DIR, RIGHT_FWD);

    motorSnO();
    Serial.println(theta);
    
    if(minLeft < minCenter && minLeft < minRight && minRightW > 50)  // Object Left
    {
      avoidLeft();
      T = 0.3;
      OBJ = LEFT;
    }
    else if(minCenter < minLeft && minCenter < minRight)
      avoidCenter();
    else if(minRight < minCenter && minRight < minLeft && minLeftW > 50) // Object Right
    { 
      avoidRight();
      T = 0.3;
      OBJ = RIGHT;
    }
  }
  
  errSumPlane1 = 0; errSumPlane2 = 0; errSumPlane3 = 0;

  // Puts robot in correct orientation after avoiding object
  if(CLEARED_OBJ == TRUE)
  {
    if(theta > 0)
    {
      while(theta > 0)
      {      
        T = 0.1;
         
        analogWrite(LEFT_MTR, 135);
        digitalWrite(LEFT_DIR, LEFT_FWD);
        analogWrite(RIGHT_MTR, 75);
        digitalWrite(RIGHT_DIR, RIGHT_FWD);
        
        motorSnO();
        Serial.println(theta);
        delay(100);
      }
      
      analogWrite(LEFT_MTR, 145);
      digitalWrite(LEFT_DIR, LEFT_FWD);
      analogWrite(RIGHT_MTR, 125);
      digitalWrite(RIGHT_DIR, RIGHT_FWD);
      delay(1000);
      
      wallDistance(RIGHT, 2);
      while(minRightW < 35)
      {
        analogWrite(LEFT_MTR, 140);
        digitalWrite(LEFT_DIR, LEFT_FWD);
        analogWrite(RIGHT_MTR, 125);
        digitalWrite(RIGHT_DIR, RIGHT_FWD);

        wallDistance(RIGHT, 2);
        delay(10);
      }
    }
    else if(theta < 0)
    {
      while(theta < 0)
      {        
        T = 0.1;
        analogWrite(LEFT_MTR, 75);
        digitalWrite(LEFT_DIR, LEFT_FWD);
        analogWrite(RIGHT_MTR, 110);
        digitalWrite(RIGHT_DIR, RIGHT_FWD);
        
        motorSnO();
        Serial.println(theta);
        delay(100);
      }

      analogWrite(LEFT_MTR, 140);
      digitalWrite(LEFT_DIR, LEFT_FWD);
      analogWrite(RIGHT_MTR, 125);
      digitalWrite(RIGHT_DIR, RIGHT_FWD);
      delay(1000);
      
      wallDistance(LEFT, 2);
      while(minLeftW < 30)
      {
        analogWrite(LEFT_MTR, 140);
        digitalWrite(LEFT_DIR, LEFT_FWD);
        analogWrite(RIGHT_MTR, 125);
        digitalWrite(RIGHT_DIR, RIGHT_FWD);

        wallDistance(LEFT, 2);
        delay(10);
      }
    }

    T = 0.2;
    motorStop();
  }
}

void planeDistance(int samples) {
  int i = 0;

  maxDistance1 = 0; maxDistance2 = 0; maxDistance3 = 0;

  for(i = 0; i <= samples; i++)
  {
    digitalWrite(trigPin1, LOW);             // Initialize Trigger to LOW
    delayMicroseconds(2);
  
    digitalWrite(trigPin1, HIGH);            // Put Trigger pin HIGH to output 8-cycle burst
    delayMicroseconds(5);
    digitalWrite(trigPin1, LOW);             // Put Trigger pin LOW to stop 8-cycle burst
  
    duration1 = pulseIn(echoPin1, HIGH);      // Sound wave travel time
    distance1 = duration1*0.034/2;            // Calculated distance (cm)

    if(distance1 < 2){distance1 = 0;}         // Remove small outlier measurements
    if(distance1 > 100){distance1 = 100;}       // Remove large outlier measurements
    
    if(maxDistance1 < distance1)
      maxDistance1 = distance1;
  }
  
  for(i= 0; i <= samples; i++)
  {
    digitalWrite(trigPin2, LOW);             // Initialize Trigger to LOW
    delayMicroseconds(2);
  
    digitalWrite(trigPin2, HIGH);            // Put Trigger pin HIGH to output 8-cycle burst
    delayMicroseconds(5);
    digitalWrite(trigPin2, LOW);             // Put Trigger pin LOW to stop 8-cycle burst
  
    duration2 = pulseIn(echoPin2, HIGH);      // Sound wave travel time
    distance2 = duration2*0.034/2;            // Calculated distance (cm)

    if(distance2 < 2){distance2 = 0;}         // Remove small outlier measurements
    if(distance2 > 100){distance2 = 100;}       // Remove large outlier measurements
    
    if(maxDistance2 < distance2)
      maxDistance2 = distance2;
  }

  for(i = 0; i <= samples; i++)
  {
    digitalWrite(trigPin3, LOW);             // Initialize Trigger to LOW
    delayMicroseconds(2);
  
    digitalWrite(trigPin3, HIGH);            // Put Trigger pin HIGH to output 8-cycle burst
    delayMicroseconds(5);
    digitalWrite(trigPin3, LOW);             // Put Trigger pin LOW to stop 8-cycle burst
  
    duration3 = pulseIn(echoPin3, HIGH);      // Sound wave travel time
    distance3 = duration3*0.034/2;            // Calculated distance (cm)

    if(distance3 < 2){distance3 = 0;}         // Remove small outlier measurements
    if(distance3 > 100){distance3 = 100;}       // Remove large outlier measurements
    
    if(maxDistance3 < distance3)
      maxDistance3 = distance3;
  }

  minLeft = maxDistance1;
  minCenter = maxDistance2;
  minRight = maxDistance3;
}

void wallDistance(uint8_t side, int samples) {
  int i = 0;
  leftMax1 = 0; rightMax1 = 0;
  
  if(side == LEFT)
  {
    for(i = 0; i <= samples; i++)
    {
      digitalWrite(leftTrig1, LOW);
      delayMicroseconds(2);
    
      digitalWrite(leftTrig1, HIGH);
      delayMicroseconds(5);
      digitalWrite(leftTrig1, LOW);
    
      leftDuration1 = pulseIn(leftEcho1, HIGH);
      leftDistance1 = leftDuration1*0.034/2;
  
      if(leftDistance1 < 2){leftDistance1 = 0;}         // Remove small outlier measurements
      if(leftDistance1 > 400){leftDistance1 = 400;}       // Remove large outlier measurements
      
      if(leftMax1 < leftDistance1)
        leftMax1 = leftDistance1;
    }

    minLeftW = leftMax1;
  }
  else if(side == RIGHT)
  {
    for(i = 0; i <= samples; i++)
    {
      digitalWrite(rightTrig1, LOW);
      delayMicroseconds(2);
    
      digitalWrite(rightTrig1, HIGH);
      delayMicroseconds(5);
      digitalWrite(rightTrig1, LOW);
    
      rightDuration1 = pulseIn(rightEcho1, HIGH);
      rightDistance1 = rightDuration1*0.034/2;
  
      if(rightDistance1 < 2){rightDistance1 = 0;}         // Remove small outlier measurements
      if(rightDistance1 > 400){rightDistance1 = 400;}       // Remove large outlier measurements
      
      if(rightMax1 < rightDistance1)
        rightMax1 = rightDistance1;
    }

    minRightW = rightMax1;
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

