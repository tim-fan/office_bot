
#include "MotorControl.h"
#include "Arduino.h"

MotorController::MotorController(int ctrlPin1, int ctrlPin2, int enablePin){
  _ctrlPin1 = ctrlPin1;
  _ctrlPin2 = ctrlPin2;
  _enablePin = enablePin;
  pinMode(_ctrlPin1, OUTPUT);
  pinMode(_ctrlPin2, OUTPUT);
  pinMode(_enablePin, OUTPUT);
  setPower(0);
}

void MotorController::setPower(int power){

    _currentPower = power;

    //map from desired power (-100 to 100) to pwm value (0 to 255)
    int pwmVal = map(abs(power), 0, 100, 0, 255);
    pwmVal = constrain(pwmVal, 0, 255);

    //ctrl pins set motor direction
    bool ctrl1 = true;
    bool ctrl2 = false;
    if (power < 0) {
        ctrl1 = !ctrl1;
        ctrl2 = !ctrl2;
    }

    digitalWrite(_ctrlPin1, ctrl1);
    digitalWrite(_ctrlPin2, ctrl2);
    analogWrite(_enablePin, pwmVal);
}

double MotorController::getPower(void){
    return _currentPower;
}
