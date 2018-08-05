#include "MotorControl.h"
#include "SpeedEncoder.h"

MotorController m1(12,A5,11);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  attachEncoderInterrupts();
}

void loop() {
  // put your main code here, to run repeatedly:
  m1.setSpeed(100);
  delay(2000);
  m1.setSpeed(0);
  delay(2000);
  m1.setSpeed(-50);
  delay(2000);
  m1.setSpeed(0);
  delay(2000);
}
