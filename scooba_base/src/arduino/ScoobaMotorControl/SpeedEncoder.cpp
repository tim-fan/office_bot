#include <TimerOne.h>

#include "Arduino.h"
#include "SpeedEncoder.h"

SpeedEncoder::SpeedEncoder(int interruptPin)
{
  _interruptPin = interruptPin;
  _tickCountSinceStart = 0;
  _tickCountSinceUpdate = 0;
}

double SpeedEncoder::getSpeed()
{
  return _speed;
}

unsigned long SpeedEncoder::getTicksSinceStart()
{
  return _tickCountSinceStart;
}

void SpeedEncoder::incrementCount()
{
  _tickCountSinceStart++;
  _tickCountSinceUpdate++;
 
}

void SpeedEncoder::computeSpeed(int updatePeriod){
  _speed = double(_tickCountSinceUpdate) / updatePeriod * 1e6;
  _tickCountSinceUpdate = 0;
}

extern int speedCalcPeriod = 50 * 1e3; //50,000 micros (50 millis, 20Hz)
extern SpeedEncoder encoder1;
extern SpeedEncoder encoder2;

void incrementEncoder1(void){
  noInterrupts();
  encoder1.incrementCount();
  interrupts();
}

void incrementEncoder2(void){
  noInterrupts();
  encoder2.incrementCount();
  interrupts();
}

void computeSpeed(void){
  noInterrupts();  
  encoder1.computeSpeed(speedCalcPeriod);
  encoder2.computeSpeed(speedCalcPeriod);
//  Serial.println(String(encoder1.getSpeed(),2) + "\t" + String(encoder2.getSpeed()));
  interrupts();
}

void attachEncoderInterrupts(){
  int encoder1Pin = encoder1._interruptPin;
  int encoder2Pin = encoder2._interruptPin;
  pinMode(encoder1Pin, INPUT_PULLUP);
  pinMode(encoder2Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1Pin), incrementEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2Pin), incrementEncoder2, RISING);
  Timer1.initialize(speedCalcPeriod);
  Timer1.attachInterrupt(computeSpeed);
}
