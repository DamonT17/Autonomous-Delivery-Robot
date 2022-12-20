#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor
{
  public:
    /**** CONSTRUCTOR ****/
    Motor();

    /**** DEFINITIONS ****/
    #define LEFT_MTR 10
    #define LEFT_DIR A0
    #define RIGHT_MTR 9
    #define RIGHT_DIR A1

    #define LEFT  1
    #define RIGHT 2

    #define PWM_MIN 40
    #define PWM_MAX 255

    #define LEFT_FWD HIGH
    #define LEFT_REV LOW
    #define RIGHT_FWD LOW
    #define RIGHT_REV HIGH

    /**** Encoder Variables ****/  
    long pos = 0;
    int lastPos = LOW;
    long deg;
    float dis;
    int n = LOW;
    
    float diam = 6; // Wheel diameter [in]
    float circum;

    /**** Speed Variables ****/
    float newTime;
    float oldTime;
    float newPos;
    float oldPos;
    float spd;

    /**** Motor Control Variables ****/
    int pwmLeft;
    int pwmRight;
    float measSpd;  // Measured speed [cm/s]
    
    float error;    // Proportional Error
    float lastErr;  // Previous error
    int errSum;     // Sum of errors (Integral)
    float dError;   // dError = error - lastErr;
    float control;  // Controller
    
    /**** METHODS ****/
    int Attach(uint8_t mtrPin, uint8_t mtrDir);
    int Attach(uint8_t mtrPin, uint8_t mtrDir, uint8_t encA, uint8_t encB);

    void Stop();
    void Forward(int val, uint8_t dir);
    void Reverse(int val, uint8_t dir);

    void Encoder();
    void Speed();

    int PControl(int mtrCnt, float setSpd, float Kp);
    int PIControl(int mtrCnt, float setSpd, float Kp, float Ki);
    int PIDControl(int mtrCnt, float setSpd, float Kp, float Ki, float Kd);

    void Limits(int mtrCnt, int pwmMin, int pwmMax);

  private:
    // Variables
    int _mtrPin;
    int _mtrDir;
    int _encA;
    int _encB;
    
};

#endif
