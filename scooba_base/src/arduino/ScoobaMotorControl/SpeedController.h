/*
  SpeedController.h - Library for speed control of a motor
  Written for control of Scooba motors, which provide feedback on
  motor speed, but not direction.
*/

#ifndef SpeedController_h
#define SpeedController_h

#include "SpeedEncoder.h"
#include "MotorControl.h"
#include <PID_v1.h>

class SpeedController
{
  public:
    SpeedController(SpeedEncoder& speedEncoder, MotorController& motorController);
    void setPidParams(double Kp, double Ki, double Kd);
    void setTicksToRadiansParam(double ticksToRadians);
    void setSpeed(double speed); //set speed, radians per second
    double getSpeed(); //return current speed, radians per second
    double getPower(); //return current motor commanded power (-100% to 100%)
    double getSetpoint(); //return current motor desired speed, radians per second
    void update(); //call as fast as possible - updates the PID loop
    
    SpeedEncoder& _encoder;
    MotorController& _motor;
    double _pidConstants[3];
    double _ticksToRadians;
    
    double _setpointSpeed, _currentSpeed, _motorPower; //Variables to connect to PID
    PID _pidController;
};

#endif 
