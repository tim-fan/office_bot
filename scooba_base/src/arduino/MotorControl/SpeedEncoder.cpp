#include <TimerOne.h>

#include "Arduino.h"
#include "SpeedEncoder.h"

SpeedEncoder::SpeedEncoder()
{
  _tickCount = 0;
}

double SpeedEncoder::getSpeed()
{
  return _speed;
}

void SpeedEncoder::incrementCount()
{
  _tickCount++;
 
}

void SpeedEncoder::computeSpeed(int updatePeriod){
  _speed = double(_tickCount) / updatePeriod * 1e6;
  _tickCount = 0;
}

extern int speedCalcPeriod = 50 * 1e3; //50,000 micros (50 millis, 20Hz)
extern int encoder1Pin = 2;
extern int encoder2Pin = 3;
SpeedEncoder encoder1 = SpeedEncoder();
SpeedEncoder encoder2 = SpeedEncoder();

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
  Serial.println(String(encoder1.getSpeed()) + "\t" + String(encoder2.getSpeed()));
  interrupts();
}

void attachEncoderInterrupts(void){
  pinMode(encoder1Pin, INPUT_PULLUP);
  pinMode(encoder2Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1Pin), incrementEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2Pin), incrementEncoder2, RISING);
  Timer1.initialize(speedCalcPeriod);
  Timer1.attachInterrupt(computeSpeed);
}
