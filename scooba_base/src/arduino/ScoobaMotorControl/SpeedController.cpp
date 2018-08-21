#include "SpeedController.h"

SpeedController::SpeedController(SpeedEncoder& speedEncoder, MotorController& motorController) : 
  _encoder(speedEncoder),
  _motor(motorController),
  _pidConstants{0,0,0},
  _currentSpeed(0),
  _motorPower(0),
  _setpointSpeed(0),
  _pidController(&_currentSpeed, &_motorPower, &_setpointSpeed, _pidConstants[0],_pidConstants[1],_pidConstants[2], DIRECT)
{
  _pidController.SetMode(AUTOMATIC);
  _pidController.SetOutputLimits(-100, 100); //output is motor power, which varies between +/- 100
}

void SpeedController::setPidParams(double Kp, double Ki, double Kd){
  _pidConstants[0] = Kp;
  _pidConstants[1] = Ki;
  _pidConstants[2] = Kd;
  _pidController.SetTunings(Kp, Ki, Kd);
}

double SpeedController::getSpeed(){
  return _encoder.getSpeed();
}

double SpeedController::getPower(){
  return _motorPower;
}

double SpeedController::getSetpoint(){
  return _setpointSpeed;
}
void SpeedController::setSpeed(double speed){
  _setpointSpeed = speed;
}

void SpeedController::update(){
  //encoder does not detect motor spin direction (forward or backward). 
  //Simplest work around - assume direction is equal 
  //to the direction of the current motor command.
  //This may cause issues with the PID controller, especially
  //if the setpoint direction is flipped suddenly. Will
  //test and attempt cleaner workaround if necessary.
  _encoder.setDirection(_motor.getPower() >= 0 ? 1 : -1);
  
  _currentSpeed = getSpeed();
  _pidController.Compute(); 
  
  //when setpoint is zero, send zero power
  //(workaround for annoying pwm hum when stationary)
  //ToDo: set pwm freq. out of audible region
  if (_setpointSpeed == 0)
  {
    _pidController.SetMode(MANUAL);
    _motor.setPower(0);
  }
  else {
    _pidController.SetMode(AUTOMATIC);
    _motor.setPower(_motorPower);
  }

}
