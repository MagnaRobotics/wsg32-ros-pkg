#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <ros/ros.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "controller_manager/controller_manager.h"

#include "wsg_32_ros_interface.h"


namespace wsg_gripper_driver
{


class Wsg32HardwareInterface : public hardware_interface::RobotHW
{


private:

  Wsg32ROSInterface gripper_;

  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;

  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];

public:

  Wsg32HardwareInterface(ros::NodeHandle &nh);

  double rate_;
 
  // helper functions
  bool init(std::string ip, int port, std::string prefix);
  void shutdown();

  void setRunning(bool b) { is_running_ = b; }

  // ros control update functions
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);
  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(1.0/rate_); }
  
  ros::NodeHandle nh_;

  bool is_running_;
  double gripper_speed_;

};

}
