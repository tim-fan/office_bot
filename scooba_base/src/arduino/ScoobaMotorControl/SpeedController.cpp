#include "SpeedController.h"

SpeedController::SpeedController(SpeedEncoder& speedEncoder, MotorController& motorController) : 
  _encoder(speedEncoder),
  _motor(motorController),
  _pidConstants{0,0,0},
  _currentSpeed(0),
  _motorPower(0),
  _setpointSpeed(0),
  _pidController(&_currentSpeed, &_motorPower, &_setpointSpeed, _pidConstants[0],_pidConstants[1],_pidConstants[2], DIRECT),
  _ticksToRadians(1)
{
  _pidController.SetMode(AUTOMATIC);
  _pidController.SetOutputLimits(-100, 100); //output is motor power, which varies between +/- 100
}

void SpeedController::setTicksPerRevolution(double ticksPerRevolution){
  _ticksToRadians = 2 * PI / ticksPerRevolution;
}

void SpeedController::setPidParams(double Kp, double Ki, double Kd){
  _pidConstants[0] = Kp;
  _pidConstants[1] = Ki;
  _pidConstants[2] = Kd;
  _pidController.SetTunings(Kp, Ki, Kd);
}

double SpeedController::getSpeed(){
  //encoder does not detect motor spin direction (forward or backward). 
  //Simplest work around - assume direction is equal 
  //to the direction of the current motor command.
  //This may cause issues with the PID controller, especially
  //if the setpoint direction is flipped suddenly. Will
  //test and attempt cleaner workaround if necessary.
  double motorDir = _motor.getPower() >= 0 ? 1 : -1;
  return _encoder.getSpeed() * _ticksToRadians * motorDir;
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
  _currentSpeed = getSpeed();
  _pidController.Compute();
  _motor.setPower(_motorPower);
//  Serial.println(String(_pidController.GetKp()));
}
