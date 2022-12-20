// Damon Tregear
// 8/29/2017
// Senior Design
// Autonomous Delivery Robot

#include "Motor.h"

#define LEFT_MTR 10
#define LEFT_DIR A0
#define RIGHT_MTR 9
#define RIGHT_DIR A1

#define LEFT_ENC_A 4    
#define LEFT_ENC_B 3    // Tie to interrupt 1
#define RIGHT_ENC_A 2   // Tie to interrupt 0
#define RIGHT_ENC_B 5   

Motor leftMotor;
Motor rightMotor;

int pwmLeft;
int pwmRight;

void setup()
{
  Serial.begin(115200);
  
  leftMotor.Attach(LEFT_MTR, LEFT_DIR, LEFT_ENC_A, LEFT_ENC_B);
  rightMotor.Attach(RIGHT_MTR, RIGHT_DIR, RIGHT_ENC_A, RIGHT_ENC_B);
 
  attachInterrupt(0, rightEncoder, CHANGE);
  attachInterrupt(1, leftEncoder, CHANGE);
}

void loop()
{
  
}

void leftEncoder()  {leftMotor.Encoder();}
void rightEncoder() {rightMotor.Encoder();}

void motorFwd(float setSpd, int Kp, int Ki, int Kd) {
  analogWrite(LEFT_MTR, pwmLeft);
  digitalWrite(LEFT_DIR, LEFT_FWD);
  analogWrite(RIGHT_MTR, pwmRight);
  digitalWrite(RIGHT_DIR, RIGHT_FWD);

  leftMotor.Speed();
  rightMotor.Speed();
  leftMotor.PIDControl(LEFT, setSpd, Kp, Ki, Kd);
  rightMotor.PIDControl(RIGHT, setSpd, Kp, Ki, Kd);
  
  delay(100);
}

