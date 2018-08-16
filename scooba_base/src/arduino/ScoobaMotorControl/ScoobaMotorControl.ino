#include <TimerOne.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include "MotorControl.h"
#include "SpeedEncoder.h"
#include "SpeedController.h"

MotorController motor1(7,4,5); //left wheel
extern SpeedEncoder encoder1 = SpeedEncoder(2);
SpeedController m1Controller(encoder1, motor1);

MotorController motor2(8,9,6); //right wheel
extern SpeedEncoder encoder2 = SpeedEncoder(3);
SpeedController m2Controller(encoder2, motor2);

//Create ROS subsribers for setting wheel speeds
ros::NodeHandle  nh;
void setLeftMotorSpeed( const std_msgs::Float32& motorSpeed){
  m1Controller.setSpeed(motorSpeed.data);
}
ros::Subscriber<std_msgs::Float32> subLeft("left_motor_speed", setLeftMotorSpeed );

void setRightMotorSpeed( const std_msgs::Float32& motorSpeed){
  m2Controller.setSpeed(motorSpeed.data);
}
ros::Subscriber<std_msgs::Float32> subRight("right_motor_speed", setRightMotorSpeed );


void setup() {
//  Serial.begin(9600);
  attachEncoderInterrupts();
  m1Controller.setTicksToRadiansParam(float(1) / 64.092); //calculated by averaging ticks over 10 revolutions
  m1Controller.setPidParams(0,10,0);
  m2Controller.setTicksToRadiansParam(float(1) / 64.092); //calculated by averaging ticks over 10 revolutions
  m2Controller.setPidParams(0,10,0);

  nh.initNode();
  nh.subscribe(subLeft);
  nh.subscribe(subRight);
}

void loop() {
  m1Controller.update();
  m2Controller.update();
  nh.spinOnce();
//  Serial.println(String(m1Controller.getSpeed()) + '\t' + String(m1Controller.getPower()) + '\t' + String(m1Controller.getSetpoint()));
//  Serial.println(String(encoder1.getTicksSinceStart()));
}
