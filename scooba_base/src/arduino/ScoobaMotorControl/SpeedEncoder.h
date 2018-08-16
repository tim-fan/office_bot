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
    SpeedEncoder(int interruptPin);
    double getSpeed();
    unsigned long getTicksSinceStart();
    void incrementCount();
    void computeSpeed(int updatePeriod);    
    int _interruptPin;
    
  private:
    unsigned int _tickCountSinceUpdate; //used for speed calculation
    unsigned long _tickCountSinceStart; //used for determining ticks per rad param (count ticks over a few revolutions)
    double _speed;
};

extern int speedCalcPeriod;

extern SpeedEncoder encoder1;
extern SpeedEncoder encoder2;

void incrementEncoder1(void);
void incrementEncoder2(void);
void attachEncoderInterrupts();

#endif
