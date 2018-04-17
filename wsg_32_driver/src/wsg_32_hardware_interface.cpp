#include <wsg_32/wsg_32_hardware_interface.h>

namespace wsg_gripper_driver
{


Wsg32HardwareInterface::Wsg32HardwareInterface(ros::NodeHandle &nh) : 
  nh_(nh),
  gripper_(nh),
  is_running_(false),
  rate_(30.0)
{
  cmd_[0] = 0.0;
  cmd_[1] = 0.0;
}

bool Wsg32HardwareInterface::init(std::string ip, int port) {

  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_gripper_left_joint("gripper_left_joint", &pos_[0], &vel_[0], &eff_[0]);
  jnt_state_interface_.registerHandle(state_handle_gripper_left_joint);

  hardware_interface::JointStateHandle state_handle_gripper_right_joint("gripper_right_joint", &pos_[1], &vel_[1], &eff_[1]);
  jnt_state_interface_.registerHandle(state_handle_gripper_right_joint);

  registerInterface(&jnt_state_interface_);

  // connect and register the joint position interface
  hardware_interface::JointHandle pos_handle_gripper_left_joint(jnt_state_interface_.getHandle("gripper_left_joint"), &cmd_[0]);
  jnt_pos_interface_.registerHandle(pos_handle_gripper_left_joint);

  hardware_interface::JointHandle pos_handle_gripper_right_joint(jnt_state_interface_.getHandle("gripper_right_joint"), &cmd_[1]);
  jnt_pos_interface_.registerHandle(pos_handle_gripper_right_joint);

  registerInterface(&jnt_pos_interface_);

  cmd_[0] = 0.0;
  cmd_[1] = 0.0;
  
  return gripper_.init(ip,port);

}

void Wsg32HardwareInterface::shutdown() {
  setRunning(false);
  gripper_.shutdown();
}


void Wsg32HardwareInterface::read(const ros::Time& time, const ros::Duration& period) {

  double current_opening_;
  gripper_.update(current_opening_);

  double val = current_opening_/1000.0;

  pos_[0] = val/2.0;
  pos_[1] = val/2.0;

  vel_[0] = 0.0;
  vel_[1] = 0.0;

  eff_[0] = 0.0;
  eff_[1] = 0.0;

}


void Wsg32HardwareInterface::write(const ros::Time& time, const ros::Duration& period) {

  double gripper_cmd = cmd_[0]+cmd_[1];
  // ROS_INFO_STREAM("Wsg32HardwareInterface::write() -- " << gripper_cmd);
  if(!gripper_.setPosition(gripper_cmd,-1)) {
    ROS_ERROR_STREAM("Wsg32HardwareInterface::write() -- could not set gripper position to be: " << gripper_cmd);
  }
  
}


} // end namespace



/**
 * The main function
 */
int main( int argc, char **argv )
{
  ros::init(argc, argv, "wsg_hardware_interface");
  ros::NodeHandle nh("/gripper_controller");

  std::string ip;
  int port;

  ROS_DEBUG("WSG 32 - ROS NODE");
  nh.param("ip", ip, std::string("192.168.1.53"));
  nh.param("port", port, 1000);

  boost::shared_ptr<wsg_gripper_driver::Wsg32HardwareInterface> gripper;
  boost::shared_ptr<ros::AsyncSpinner> spinner;

  gripper = boost::make_shared<wsg_gripper_driver::Wsg32HardwareInterface>(nh);
  controller_manager::ControllerManager cm(&(*(gripper.get())), nh);

  spinner = boost::make_shared<ros::AsyncSpinner>(1);
  spinner->start();

  ros::Rate rate(1.0 / gripper->getPeriod().toSec());
  if(!gripper->init(ip, port)) {
    ROS_ERROR("Unable to initialize gripper ros control node");
  }

  gripper->setRunning(true);
  while (ros::ok())
  {
    ros::Time now = gripper->getTime();
    ros::Duration dt = gripper->getPeriod();
    gripper->read(now, dt);
    cm.update(now, dt);
    gripper->write(now, dt);
    rate.sleep();
    ros::spinOnce();
  }
  gripper->shutdown();
  
  ros::shutdown();
  return 0;
}
