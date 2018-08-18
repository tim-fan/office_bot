
#include <math.h>       /* fabs */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


class Scooba : public hardware_interface::RobotHW
{
public:
  Scooba(ros::NodeHandle nh){

    pos_[0] = 0.0; pos_[1] = 0.0;
    vel_[0] = 0.0; vel_[1] = 0.0;
    eff_[0] = 0.0; eff_[1] = 0.0;
    cmd_[0] = 0.0; cmd_[1] = 0.0;

    hardware_interface::JointStateHandle state_handle_1("right_wheel_joint", &pos_[0], &vel_[0], &eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("left_wheel_joint", &pos_[1], &vel_[1], &eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_2);

    registerInterface(&jnt_state_interface_);

    hardware_interface::JointHandle vel_handle_left(jnt_state_interface_.getHandle("right_wheel_joint"), &cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_left);

    hardware_interface::JointHandle vel_handle_right(jnt_state_interface_.getHandle("left_wheel_joint"), &cmd_[1]);
    jnt_vel_interface_.registerHandle(vel_handle_right);

    registerInterface(&jnt_vel_interface_);

    //create publisher to send motor commands to motors
    left_motor_pub = nh.advertise<std_msgs::Float32>("left_wheel_speed", 100);
    right_motor_pub = nh.advertise<std_msgs::Float32>("right_wheel_speed", 100);
    
    //Ticks to radians conversion: determined by rotating wheel 10 times, 
    //and watching published tick count
    ticksToRadians = 2*M_PI * 10 / 4014; 
    radiansToTicks = 1/ticksToRadians;
  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.1);}

  std_msgs::Float32 publishVel(float vel, ros::Publisher pub){
    std_msgs::Float32 msg;
    msg.data = vel;
    pub.publish(msg);
  }

  void read(){
    ROS_INFO_STREAM("Commands for joints: " << cmd_[0] << ", " << cmd_[1]);
    
    //send commands to robot through motor topics
    publishVel(cmd_[0] * radiansToTicks, right_motor_pub);
    publishVel(cmd_[1] * radiansToTicks, left_motor_pub); 
  }
  
  void setLeftPos(std_msgs::Int32 ticksMsg){
    pos_[1] = ticksToRadians * ticksMsg.data;
  }
  void setRightPos(std_msgs::Int32 ticksMsg){
    pos_[0] = ticksToRadians * ticksMsg.data;
  }
  void setLeftVel(std_msgs::Float32 ticksPerSecondMsg){
    vel_[1] = ticksToRadians * ticksPerSecondMsg.data;
  }
  void setRightVel(std_msgs::Float32 ticksPerSecondMsg){
    vel_[0] = ticksToRadians * ticksPerSecondMsg.data;
  }

private:
  ros::Publisher left_motor_pub;
  ros::Publisher right_motor_pub;
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];
  double ticksToRadians;
  double radiansToTicks;
};

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "scooba_base_driver");
  ros::NodeHandle nh;
    
  Scooba robot(nh);
  ROS_INFO_STREAM("period: " << robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::Subscriber leftPosSub = nh.subscribe("left_wheel_ticks", 1000, &Scooba::setLeftPos, &robot);
  ros::Subscriber rightPosSub = nh.subscribe("right_wheel_ticks", 1000, &Scooba::setRightPos, &robot);
  ros::Subscriber leftVelSub = nh.subscribe("left_wheel_speed_feedback", 1000, &Scooba::setLeftVel, &robot);
  ros::Subscriber rightVelSub = nh.subscribe("right_wheel_speed_feedback", 1000, &Scooba::setRightVel, &robot);

  while(ros::ok())
  {
    robot.read();   
    cm.update(robot.getTime(), robot.getPeriod());
    rate.sleep();
  }
  spinner.stop();

  return 0;
}

