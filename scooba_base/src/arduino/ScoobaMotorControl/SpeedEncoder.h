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
    long getTicksSinceStart();
    void incrementCount();
    void computeSpeed(int updatePeriod);
    void setDirection(int direction);
    int _interruptPin;
    int _direction;
    
  private:
    int _tickCountSinceUpdate; //used for speed calculation
    long _tickCountSinceStart; //used for determining ticks per rad param (count ticks over a few revolutions)
    double _speed;
};

extern int speedCalcPeriod;

extern SpeedEncoder encoderLeft;
extern SpeedEncoder encoderRight;

void incrementLeftEncoder(void);
void incrementRightEncoder(void);
void attachEncoderInterrupts();

#endif
