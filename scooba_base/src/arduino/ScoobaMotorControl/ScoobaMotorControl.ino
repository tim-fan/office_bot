/*
 * Scooba motor controller
 * Provides a ROS serial interface to two scooba motors
 * 
 * NOTE to get memory size down, had to reduce num pubs/subs 
 * and buffer sizes in <sketchbook folder>/libraries/ros_lib/ros.h
 * Currently used settings:
 * #elif defined(__AVR_ATmega328P__)                                                                                                                                                                                                              
 * 
 * typedef NodeHandle_<ArduinoHardware, 2, 4, 150, 150> NodeHandle;
 * 
 * Note in my case, the memory issues were manifesting as failure to 
 * read PID parameters.
 */


#include <TimerOne.h>
#include <ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>


#include "MotorControl.h"
#include "SpeedEncoder.h"
#include "SpeedController.h"

//left wheel controller
MotorController leftMotor(7,4,5); 
extern SpeedEncoder leftEncoder = SpeedEncoder(2);
SpeedController leftMotorController(leftEncoder, leftMotor);

//right wheel controller
MotorController rightMotor(8,9,6); 
extern SpeedEncoder rightEncoder = SpeedEncoder(3);
SpeedController rightMotorController(rightEncoder, rightMotor);

ros::NodeHandle  nh;

//ROS subscribers for setting wheel speeds
void setLeftMotorSpeed( const std_msgs::Float32& motorSpeed){
  leftMotorController.setSpeed(motorSpeed.data);
}
ros::Subscriber<std_msgs::Float32> subLeft("left_wheel_speed", setLeftMotorSpeed );

void setRightMotorSpeed( const std_msgs::Float32& motorSpeed){
  rightMotorController.setSpeed(motorSpeed.data);
}
ros::Subscriber<std_msgs::Float32> subRight("right_wheel_speed", setRightMotorSpeed );

std_msgs::Int32 leftWheelTicksMsg;
ros::Publisher leftTicksPublisher("left_wheel_ticks", &leftWheelTicksMsg);
std_msgs::Int32 rightWheelTicksMsg;
ros::Publisher rightTicksPublisher("right_wheel_ticks", &rightWheelTicksMsg);
std_msgs::Float32 leftWheelSpeedMsg;
ros::Publisher leftSpeedPublisher("left_wheel_speed_feedback", &leftWheelSpeedMsg);
std_msgs::Float32 rightWheelSpeedMsg;
ros::Publisher rightSpeedPublisher("right_wheel_speed_feedback", &rightWheelSpeedMsg);

//params for speed control.
//Will be set as ROS params
float pid_constants[3];
float ticksPerRevolution;
int publishPeriod = 50; //ms
unsigned long nextPublishTime = millis();

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
  nh.advertise(leftTicksPublisher);
  nh.advertise(rightTicksPublisher);
  nh.advertise(leftSpeedPublisher);
  nh.advertise(rightSpeedPublisher);
  while(!nh.connected()) {
    nh.spinOnce();
    flashLed(1000);
  }

  //get params
  while (!nh.getParam("~pid_constants", pid_constants,3)){ 
    nh.spinOnce();
    flashLed(500);
  }

  //pass params to controllers
  leftMotorController.setPidParams(pid_constants[0],pid_constants[1],pid_constants[2]);
  rightMotorController.setPidParams(pid_constants[0],pid_constants[1],pid_constants[2]);  

  //start encoders
  attachEncoderInterrupts();
}

void loop() {
  flashLed(30); //for making sure loop is still looping
  
  leftMotorController.update();
  rightMotorController.update();

  if (millis() > nextPublishTime){
    nextPublishTime += publishPeriod;
    leftWheelTicksMsg.data = leftEncoder.getTicksSinceStart();    
    rightWheelTicksMsg.data = rightEncoder.getTicksSinceStart();
    leftWheelSpeedMsg.data = leftEncoder.getSpeed();
    rightWheelSpeedMsg.data = rightEncoder.getSpeed();
    
    leftTicksPublisher.publish(&leftWheelTicksMsg);
    rightTicksPublisher.publish(&rightWheelTicksMsg);
    leftSpeedPublisher.publish(&leftWheelSpeedMsg);
    rightSpeedPublisher.publish(&rightWheelSpeedMsg);
  }
  
  nh.spinOnce();
}
