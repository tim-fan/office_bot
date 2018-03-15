#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "MotorControl.h"


class BaseController
{
		SerialMotors motors;
	public:
		//BaseController(): motors("/dev/pts/5") {}; //connect to specific serial port
		void setVel(const geometry_msgs::Twist::ConstPtr& velCmd);
};


void BaseController::setVel(const geometry_msgs::Twist::ConstPtr& velCmd)
{
	ROS_INFO("I heard: [%4.2f]", velCmd->linear.x);
	int runTimeMs = 1000;

  //TODO: implement interface as standard JointVelocityInterface, then use diff-drive controller	
	float wheelDist = 0.15;
	float speedRight = (velCmd->angular.z*wheelDist)/2 + velCmd->linear.x;
	float speedLeft = velCmd->linear.x*2-speedRight;	

	motors.SetMotors(speedLeft,speedRight,runTimeMs*10);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "thjc_bot_base");

  ros::NodeHandle n;
  
  BaseController controller;
  
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, &BaseController::setVel, &controller);
  ros::spin();

  return 0;
}
