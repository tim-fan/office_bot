/*
  MotorControl.h - Library for controlling motors through a motor driver.
  motors are controlled by the driver through two direction pins and one enable pin
  Note the enable pin must support PWM.
  I'm testing this with motor driver L298N
*/

#ifndef MotorControl_h
#define MotorControl_h
#include "Arduino.h"


class MotorController
{
  public:
    MotorController(int ctrlPin1, int ctrlPin2, int enablePin);
    void setSpeed(int speed); //set speed (percentage, -100 tp 100)
  private:
    int _ctrlPin1;
    int _ctrlPin2;
    int _enablePin;
};

#endif
