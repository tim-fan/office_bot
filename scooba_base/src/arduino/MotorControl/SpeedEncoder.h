/*
  SpeedEncoder.h - Library for interrupt driven 
  reading of two optical speed encoders. 
  Assumes the encoders are attached to digital inputs 2 and 3
  (these are the two pins on the ATmega168/328 which support
  external interrupts)
  Updates current speed measurement in a time-based isr
*/

#ifndef SpeedEncoder_h
#define SpeedEncoder_h
#include "Arduino.h"


class SpeedEncoder
{
  public:
    SpeedEncoder();
    double getSpeed();
    void incrementCount();
    void computeSpeed(int updatePeriod);
  private:
    int _tickCount;
    double _speed;
};

extern int speedCalcPeriod;
extern int encoder1Pin;
extern int encoder2Pin;

extern SpeedEncoder encoder1;
extern SpeedEncoder encoder2;

void incrementEncoder1(void);
void incrementEncoder2(void);
void attachEncoderInterrupts(void);

#endif
