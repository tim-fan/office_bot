/*
 * Scooba motor controller
 * Provides a ROS serial interface to two scooba motors
 * 
 * NOTE to get memory size down, had to reduce num pubs/subs 
 * and buffer sizes in <sketchbook folder>/libraries/ros_lib/ros.h
 * Currently used settings:
 * #elif defined(__AVR_ATmega328P__)                                                                                                                                                                                                              
 * 
 * typedef NodeHandle_<ArduinoHardware, 2, 2, 150, 150> NodeHandle;
 * 
 * Note in my case, the memory issues were manifesting as failure to 
 * read PID parameters.
 */


#include <TimerOne.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include "MotorControl.h"
#include "SpeedEncoder.h"
#include "SpeedController.h"

//left wheel controller
MotorController motor1(7,4,5); 
extern SpeedEncoder encoder1 = SpeedEncoder(2);
SpeedController m1Controller(encoder1, motor1);

//right wheel controller
MotorController motor2(8,9,6); 
extern SpeedEncoder encoder2 = SpeedEncoder(3);
SpeedController m2Controller(encoder2, motor2);

ros::NodeHandle  nh;

//ROS subsribers for setting wheel speeds
void setLeftMotorSpeed( const std_msgs::Float32& motorSpeed){
  m1Controller.setSpeed(motorSpeed.data);
}
ros::Subscriber<std_msgs::Float32> subLeft("left_motor_speed", setLeftMotorSpeed );

void setRightMotorSpeed( const std_msgs::Float32& motorSpeed){
  m2Controller.setSpeed(motorSpeed.data);
}
ros::Subscriber<std_msgs::Float32> subRight("right_motor_speed", setRightMotorSpeed );


//params for speed control.
//Will be set as ROS params
float pid_constants[3];
float ticksPerRevolution;


//led flashing
bool ledState = false;
unsigned long lastToggleTime = millis();
void flashLed(unsigned long togglePeriod){
  digitalWrite(LED_BUILTIN, ledState);
  if (millis() - lastToggleTime > togglePeriod)
  {
    ledState = !ledState;
    lastToggleTime = millis();
  }
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  //start up the node handle
  nh.initNode();
  nh.subscribe(subLeft);
  nh.subscribe(subRight);
  while(!nh.connected()) {
    nh.spinOnce();
    flashLed(1000);
  }

  //get params
  while (!nh.getParam("~pid_constants", pid_constants,3)){ 
    nh.spinOnce();
    flashLed(500);
  }
  while (!nh.getParam("~ticks_per_revolution", &ticksPerRevolution )) {
    nh.spinOnce();
    flashLed(500);
  }

  //pass params to controllers
  m1Controller.setPidParams(pid_constants[0],pid_constants[1],pid_constants[2]);
  m2Controller.setPidParams(pid_constants[0],pid_constants[1],pid_constants[2]);  
  
  //for reference, ticks per rev for scooba motor = 402.7 calculated by averaging ticks over 10 revolutions
  m1Controller.setTicksPerRevolution(ticksPerRevolution);
  m2Controller.setTicksPerRevolution(ticksPerRevolution);

  //start encoders
  attachEncoderInterrupts();
}

void loop() {
  flashLed(30); //for making sure loop is still looping
  
  m1Controller.update();
  m2Controller.update();
  nh.spinOnce();
}
