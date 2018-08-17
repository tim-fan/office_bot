#include <TimerOne.h>

#include "Arduino.h"
#include "SpeedEncoder.h"

SpeedEncoder::SpeedEncoder(int interruptPin)
{
  _interruptPin = interruptPin;
  _tickCountSinceStart = 0;
  _tickCountSinceUpdate = 0;
  _direction = 1;
}

double SpeedEncoder::getSpeed()
{
  return _speed;
}

long SpeedEncoder::getTicksSinceStart()
{
  return _tickCountSinceStart;
}

void SpeedEncoder::incrementCount()
{
  _tickCountSinceStart += _direction;
  _tickCountSinceUpdate += _direction;
 
}

void SpeedEncoder::computeSpeed(int updatePeriod){
  _speed = double(_tickCountSinceUpdate) / updatePeriod * 1e6;
  _tickCountSinceUpdate = 0;
}

void SpeedEncoder::setDirection(int direction){
  _direction = direction;  
}

extern int speedCalcPeriod = 50 * 1e3; //50,000 micros (50 millis, 20Hz)
extern SpeedEncoder leftEncoder;
extern SpeedEncoder rightEncoder;

void incrementLeftEncoder(void){
  noInterrupts();
  leftEncoder.incrementCount();
  interrupts();
}

void incrementRightEncoder(void){
  noInterrupts();
  rightEncoder.incrementCount();
  interrupts();
}

void computeSpeed(void){
  noInterrupts();  
  leftEncoder.computeSpeed(speedCalcPeriod);
  rightEncoder.computeSpeed(speedCalcPeriod);
//  Serial.println(String(leftEncoder.getSpeed(),2) + "\t" + String(rightEncoder.getSpeed()));
  interrupts();
}

void attachEncoderInterrupts(){
  int leftEncoderPin = leftEncoder._interruptPin;
  int rightEncoderPin = rightEncoder._interruptPin;
  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), incrementLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), incrementRightEncoder, RISING);
  Timer1.initialize(speedCalcPeriod);
  Timer1.attachInterrupt(computeSpeed);
}
