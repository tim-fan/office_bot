
#include <math.h>       /* fabs */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


class RedRobot : public hardware_interface::RobotHW
{
public:
  RedRobot(ros::NodeHandle nh){

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
    left_motor_pub = nh.advertise<std_msgs::Float32>("left_motor_speed", 100);
    right_motor_pub = nh.advertise<std_msgs::Float32>("right_motor_speed", 100);
  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.1);}

  std_msgs::Float32 publishVel(float vel, ros::Publisher pub){
    std_msgs::Float32 msg;
    //deadzone scaling:
    float maxSpeed = 16;
    float deadZoneCutoff = 10;
    float correctedSpeed = deadZoneCutoff + fabs(vel) * (maxSpeed - deadZoneCutoff) / maxSpeed;
    float direction = (vel == 0) ? 0 : ((vel > 0) ? 1 : -1);
    float correctedVel = direction * correctedSpeed;

    msg.data = correctedVel;
    pub.publish(msg);
  }

  void read(){
    ROS_INFO_STREAM("Commands for joints: " << cmd_[0] << ", " << cmd_[1]);
    
    //send commands to robot through motor topics
    publishVel(cmd_[0], right_motor_pub);
    publishVel(cmd_[1], left_motor_pub); 
  }
  

  void write(){
    double cur_vel[2];

    
    //just use commands as feedback vel (don't have any encoders fitted)
    cur_vel[0] = cmd_[0];
    cur_vel[1] = cmd_[1];
    ROS_INFO_STREAM("Red Robot vel: " << cur_vel[0] << ", " << cur_vel[1]);

    for (unsigned int i = 0; i < 2; ++i)
    {
      pos_[i] += cur_vel[i] * getPeriod().toSec();
      vel_[i] = cur_vel[i];
    }
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
  float scaleFactorVelToMotorSpeed;
};

int main(int argc, char **argv)
{
  double x, y, theta;
  
  ros::init(argc, argv, "red_robot");
  ros::NodeHandle nh;
    
  RedRobot robot(nh);
  ROS_INFO_STREAM("period: " << robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    robot.read();
    robot.write();    
    cm.update(robot.getTime(), robot.getPeriod());
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
