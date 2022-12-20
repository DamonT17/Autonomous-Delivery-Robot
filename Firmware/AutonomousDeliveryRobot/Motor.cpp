#include "Arduino.h"
#include "Motor.h"

Motor::Motor(){}

int Motor::Attach(uint8_t mtrPin, uint8_t mtrDir)
{
  _mtrPin = mtrPin;
  _mtrDir = mtrDir;
  
  pinMode(_mtrPin, OUTPUT);  // Digital PWM pin
  pinMode(_mtrDir, OUTPUT);  // Analog pin (controls direction)
}

int Motor::Attach(uint8_t mtrPin, uint8_t mtrDir, uint8_t encA, uint8_t encB)
{
  _mtrPin = mtrPin;
  _mtrDir = mtrDir;
  _encA = encA;
  _encB = encB;
  
  pinMode(_mtrPin, OUTPUT);  // Digital PWM pin
  pinMode(_mtrDir, OUTPUT);  // Analog pin (controls direction)

  pinMode(_encA,INPUT);      // Encoder pin A
  digitalWrite(_encA, HIGH);
  pinMode(_encB,INPUT);      // Encoder pin B
  digitalWrite(_encB, HIGH);
}

void Motor::Stop()
{
  digitalWrite(_mtrPin, LOW);
  digitalWrite(_mtrDir, LOW);
}

void Motor::Forward(int val, uint8_t dir)
{
  analogWrite(_mtrPin, val);
  digitalWrite(_mtrDir, dir);
}

void Motor::Reverse(int val, uint8_t dir)
{
  analogWrite(_mtrPin, val);
  digitalWrite(_mtrDir, dir);
}

void Motor::Encoder() {
  n = digitalRead(_encA);   // Read status of Pin A
  
  if((lastPos == LOW) && (n == HIGH)) 
  {
    if(digitalRead(_encB) == LOW) 
    {
      pos--;   // Decrement position counter if reverse  
    } 
    else 
    {
      pos++;  // Increment position counter if forward
    }

    deg = pos*0.17143;         // Degrees conversion

    circum = PI*diam;
    dis = -((deg*circum)/360);  // Distance [in]
    // Distance = (Degrees turned*Wheel Circumfrence)/360
    // Wheel diameter = 6 inches
  }  
  lastPos = n;  // Set the last value of Pin A to state of 'n' 
}

void Motor::Speed() {
  newPos = dis;
  
  newTime = millis(); 

  spd = 1000*(newPos - oldPos)/(newTime - oldTime);

  oldPos = newPos;  
  oldTime = newTime;
}

int Motor::PControl(int mtrCnt, float setSpd, float Kp) {
  if(mtrCnt == LEFT)
  {
    do
    {
      measSpd = spd;
      error = setSpd - measSpd;
      
      control += Kp*error;  // P control
  
      pwmLeft = abs(control);         // Control left motor PWM
  
      Limits(LEFT, PWM_MIN, PWM_MAX);
  
      pwmRight = pwmLeft;     // Set right motor PWM to left motor PWM
      lastErr = error;        // Set error as previous error
      
    } while(error > 0.1 && error < -0.1);
  }
  else if(mtrCnt == RIGHT)
  {
    do
    {
      measSpd = spd;
      error = setSpd - measSpd;
  
      control += Kp*error;  // P control
  
      pwmRight = abs(control);         // Control left motor PWM
  
      Limits(RIGHT, PWM_MIN, PWM_MAX);
  
      pwmLeft = pwmRight;     // Set right motor PWM to left motor PWM
      lastErr = error;        // Set error as previous error
      
    } while(error > 0.1 && error < -0.1);
  }
  else
    return false;
}

int Motor::PIControl(int mtrCnt, float setSpd, float Kp, float Ki) {
  if(mtrCnt == LEFT)
  {
    do
    {
      measSpd = spd;
      error = setSpd - measSpd;
      errSum += error;
      
      control += Kp*error + Ki*errSum;  // PI control
  
      pwmLeft = abs(control);         // Control left motor PWM
  
      Limits(LEFT, PWM_MIN, PWM_MAX);
  
      pwmRight = pwmLeft;     // Set right motor PWM to left motor PWM
      lastErr = error;        // Set error as previous error
      
    } while(error > 0.1 && error < -0.1);
  }
  else if(mtrCnt == RIGHT)
  {
    do
    {
      measSpd = spd;
      error = setSpd - measSpd;
      errSum += error;
  
      control += Kp*error + Ki*errSum;  // PI control
  
      pwmRight = abs(control);         // Control left motor PWM
  
      Limits(RIGHT, PWM_MIN, PWM_MAX);
  
      pwmLeft = pwmRight;     // Set right motor PWM to left motor PWM
      lastErr = error;        // Set error as previous error
      
    } while(error > 0.1 && error < -0.1);
  }
  else
    return false;
}

int Motor::PIDControl(int mtrCnt, float setSpd, float Kp, float Ki, float Kd)
{
  if(mtrCnt == LEFT)
  {
    do
    {
      measSpd = spd;
      error = setSpd - measSpd;
      errSum += error;            // Integral error
      dError = error - lastErr;   // Derivative error
      
      control += Kp*error + Ki*errSum + Kd*dError;  // PID control
  
      pwmLeft = abs(control);         // Control left motor PWM
  
      Limits(LEFT, PWM_MIN, PWM_MAX);
  
      pwmRight = pwmLeft;     // Set right motor PWM to left motor PWM
      lastErr = error;        // Set error as previous error
      
    } while(error > 0.1 && error < -0.1);
  }
  else if(mtrCnt == RIGHT)
  {
    do
    {
      measSpd = spd;
      error = setSpd - measSpd;
      errSum += error;            // Integral error
      dError = error - lastErr;   // Derivative error
  
      control += Kp*error + Ki*errSum + Kd*dError;  // PID control
  
      pwmRight = abs(control);         // Control left motor PWM
  
      Limits(RIGHT, PWM_MIN, PWM_MAX);
  
      pwmLeft = pwmRight;     // Set right motor PWM to left motor PWM
      lastErr = error;        // Set error as previous error
    } while(error > 0.1 && error < -0.1);
  }
  else
    return false;
}

void Motor::Limits(int mtrCnt, int pwmMin, int pwmMax) {
  if(mtrCnt == LEFT)
  { 
    if(pwmLeft >= pwmMax)
      pwmLeft = pwmMax;
    if(pwmLeft <= pwmMin)
      pwmLeft = pwmMin;  
  }
  else if(mtrCnt == RIGHT)
  {
    if(pwmRight >= pwmMax)
      pwmRight = pwmMax;
    if(pwmRight <= pwmMin)
      pwmRight = pwmMin;
  }
  else
    return false;
}
