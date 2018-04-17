/*
 * WSG 50 ROS NODE
 * Copyright (c) 2012, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Marc Benetó (mbeneto@robotnik.es)
 * \author Sachin Chitta (robot.moveit@gmail.com)
 * \brief WSG-32 ROS driver.
 */


//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------

#include <wsg_32/gripper_action_server.h>

namespace wsg_gripper_driver
{

static const double MAX_OPENING = 0.07;
static const double MAX_SPEED = 0.42;

GripperActionServer::GripperActionServer(ros::NodeHandle &nh) : 
  nh_(nh),
  is_running_(false)
{

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

  last_cmd_ = 0.0;
  new_cmd_ = 0.0;
  max_effort_ = 5.0;

  cmd_[0] = 0.0;
  cmd_[1] = 0.0;

}

bool GripperActionServer::preemptActiveCallback()
{
  if(current_active_goal_ && current_active_goal_->getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
    current_active_goal_->setCanceled();
  current_active_goal_.reset();
  return true;
}

bool GripperActionServer::initialize()
{

  // Services
  moveSS = nh_.advertiseService("/wsg_gripper_driver/move", &GripperActionServer::moveSrv, this);
  graspSS = nh_.advertiseService("/wsg_gripper_driver/grasp", &GripperActionServer::graspSrv, this);
  releaseSS = nh_.advertiseService("/wsg_gripper_driver/release", &GripperActionServer::releaseSrv, this);
  homingSS = nh_.advertiseService("/wsg_gripper_driver/homing", &GripperActionServer::homingSrv, this);
  stopSS = nh_.advertiseService("/wsg_gripper_driver/stop", &GripperActionServer::stopSrv, this);
  ackSS = nh_.advertiseService("/wsg_gripper_driver/ack", &GripperActionServer::ackSrv, this);
  incrementSS = nh_.advertiseService("/wsg_gripper_driver/move_incrementally", &GripperActionServer::incrementSrv, this);
  setAccSS = nh_.advertiseService("/wsg_gripper_driver/set_acceleration", &GripperActionServer::setAccSrv, this);
  setForceSS = nh_.advertiseService("/wsg_gripper_driver/set_force", &GripperActionServer::setForceSrv, this);

  // ROS API: Action interface
  action_server_.reset(new ActionServer(nh_, "gripper_cmd",
					boost::bind(&GripperActionServer::goalCB, this, _1),
					boost::bind(&GripperActionServer::cancelCB, this, _1),
					false));
  action_server_->start();

  // Publishers
  state_pub_ = nh_.advertise<wsg_32_common::Status>("/wsg_gripper_driver/status", 1000);
  joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
  ack_fault();
  homing();
  ROS_INFO("WSG 32 ready to use.");
  setGraspingForceLimit(15);
  ROS_INFO_STREAM("WSG using force "<<getGraspingForceLimit());

  return true;
}

bool GripperActionServer::moveSrv(wsg_32_common::Move::Request &req,
                                  wsg_32_common::Move::Response &res)
{
  if ( (req.width >= 0.0 && req.width <= MAX_OPENING) && (req.speed > 0.0 && req.speed <= MAX_SPEED) )
  {
    ROS_INFO("Moving to %f position at %f mm/s.", req.width, req.speed);
    res.error = move(req.width * 1000, req.speed * 1000, false, false);
  }
  else if (req.width < 0.0 || req.width > MAX_OPENING)
  {
    ROS_ERROR("Impossible to move to this position. (Width values: [0.0 - %f] m", MAX_OPENING);
    res.error = 255;
    return true;
  }
  else
  {
    ROS_WARN("Speed values are outside the gripper's physical limits ([0.0001 - %f m/s])  Using clamped values.", MAX_SPEED);
    res.error = move(req.width * 1000, req.speed * 1000, false, false);
  }
  if (res.error == 255) {
    ack_fault();
  }
  ROS_DEBUG("Target position reached.");
  return true;
}

bool GripperActionServer::graspSrv(wsg_32_common::Move::Request &req,
				   wsg_32_common::Move::Response &res)
{
  if ( (req.width >= 0.0 && req.width <= MAX_OPENING) && (req.speed > 0.0 && req.speed <= MAX_SPEED) )
  {
    ROS_DEBUG("Grasping object of width %f at %f mm/s.", req.width, req.speed);
    res.error = grasp(req.width*1000, req.speed*1000);
  }
  else if (req.width < 0.0 || req.width > MAX_OPENING)
  {
    ROS_ERROR("Impossible to move to position %lf. (Width values: [0.0 - %f] ", req.width, MAX_OPENING);
    res.error = 255;
    return false;
  }
  else
  {
    ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: [0.1 - %f])  Using clamped values.", MAX_SPEED);
    res.error = grasp(req.width, req.speed);
  }
  ROS_DEBUG("Object grasped correctly.");
  objectGraspped=true;
  return true;
}

bool GripperActionServer::incrementSrv(wsg_32_common::Incr::Request &req,
				       wsg_32_common::Incr::Response &res)
{
  if (req.direction == "open"){

    if (!objectGraspped){

      float currentWidth = getOpening();
      float nextWidth = currentWidth + req.increment*1000;
      if ( (currentWidth < GRIPPER_MAX_OPEN) && nextWidth < GRIPPER_MAX_OPEN ){
      	//grasp(nextWidth, 1);
      	move(nextWidth,20, false, false);
      	currentWidth = nextWidth;
      }else if( nextWidth >= GRIPPER_MAX_OPEN){
      	//grasp(GRIPPER_MAX_OPEN, 1);
      	move(GRIPPER_MAX_OPEN,1, false, false);
      	currentWidth = GRIPPER_MAX_OPEN;
      }
    }else{
      ROS_DEBUG("Releasing object...");
      release(GRIPPER_MAX_OPEN, 20);
      objectGraspped = false;
    }
  }else if (req.direction == "close"){

    if (!objectGraspped){

      float currentWidth = getOpening();
      float nextWidth = currentWidth - req.increment;

      if ( (currentWidth > GRIPPER_MIN_OPEN) && nextWidth > GRIPPER_MIN_OPEN ){
      	//grasp(nextWidth, 1);
      	move(nextWidth,20, false, false);
      	currentWidth = nextWidth;
      } else if( nextWidth <= GRIPPER_MIN_OPEN){
      	//grasp(GRIPPER_MIN_OPEN, 1);
      	move(GRIPPER_MIN_OPEN,1, false, false);
      	currentWidth = GRIPPER_MIN_OPEN;
      }
    }
  }
}

bool GripperActionServer::releaseSrv(wsg_32_common::Move::Request &req,
				     wsg_32_common::Move::Response &res)
{
  if ( (req.width >= 0.0 && req.width <= MAX_OPENING) && (req.speed > 0.0 && req.speed <= MAX_SPEED) ){
    ROS_DEBUG("Releasing to %f position at %f mm/s.", req.width, req.speed);
    res.error = release(req.width*1000, req.speed*1000);
  }else if (req.width < 0.0 || req.width > MAX_OPENING){
    ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - %f] ", MAX_OPENING);
    res.error = 255;
    return false;
  }else{
    ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - %f] / Speed: [0.0001 - %f])  Using clamped values.", MAX_OPENING, MAX_SPEED);
    res.error = release(req.width*1000, req.speed*1000);
  }
  ROS_DEBUG("Object released correctly.");
  return true;
}

bool GripperActionServer::homingSrv(std_srvs::Empty::Request &req,
				    std_srvs::Empty::Request &res)
{
  ROS_DEBUG("Homing...");
  homing();
  ROS_DEBUG("Home position reached.");
  return true;
}

bool GripperActionServer::stopSrv(std_srvs::Empty::Request &req,
				  std_srvs::Empty::Request &res)
{
  ROS_WARN("Stop!");
  stop();
  ROS_WARN("Stopped.");
  return true;
}

bool GripperActionServer::setAccSrv(wsg_32_common::Conf::Request &req,
				    wsg_32_common::Conf::Response &res)
{
  setAcceleration(req.val*1000);
  return true;
}

bool GripperActionServer::setForceSrv(wsg_32_common::Conf::Request &req,
				      wsg_32_common::Conf::Response &res)
{
  setGraspingForceLimit(req.val);
  return true;
}

bool GripperActionServer::ackSrv(std_srvs::Empty::Request &req,
				 std_srvs::Empty::Request &res)
{
  stop();
  ack_fault();
  return true;
}

void GripperActionServer::goalCB(GoalHandle gh)
{
  ROS_INFO("Gripper action Received new action goal");
  // Precondition: Running controller
  if (!is_running_)
  {
    ROS_ERROR("Can't accept new action goals. Controller is not running.");
    control_msgs::GripperCommandResult result;
    gh.setRejected(result);
    return;
  }
  // Accept new goal
  preemptActiveCallback();
  gh.setAccepted();
  current_active_goal_.reset(new GoalHandle(gh));
  ROS_INFO("GOT A GOAL: EFFORT=%f POSITION=%f", gh.getGoal()->command.max_effort, gh.getGoal()->command.position);

  if (!nh_.hasParam("gripper_speed"))
    nh_.setParam("gripper_speed",MAX_SPEED);
  
  nh_.getParamCached("gripper_speed", gripper_speed_);
  
  new_cmd_ = gh.getGoal()->command.position;
  max_effort_ = gh.getGoal()->command.max_effort;

  cmd_[0] = new_cmd_/2.0;
  cmd_[1] = new_cmd_/2.0;

}

void GripperActionServer::cancelCB(GoalHandle gh)
{
  // Check that cancel request refers to currently active goal (if any)
  if (current_active_goal_ && *current_active_goal_ == gh)
  {
    ROS_DEBUG("Canceling active action goal because cancel callback recieved from actionlib.");
    // Mark the current goal as canceled
    current_active_goal_->setCanceled();
  }
}

void GripperActionServer::read(const ros::Time& time, const ros::Duration& period) {

  //Get state values
  const char * aux;
  aux = systemState();
  float op = getOpening();
  float acc = getAcceleration();
  float force = getGraspingForceLimit();

  std::stringstream ss;
  ss << aux;

  wsg_32_common::Status status_msg;
  status_msg.status = ss.str();
  status_msg.width = (int) op;
  status_msg.acc = (int) acc;
  status_msg.force = (int) force;

  state_pub_.publish(status_msg);

  double val = ((double) op)/1000.0;

  pos_[0] = val/2.0;
  pos_[1] = val/2.0;

  vel_[0] = 0.0;
  vel_[1] = 0.0;

  eff_[0] = 0.0;
  eff_[1] = 0.0;
  
}

void GripperActionServer::write(const ros::Time& time, const ros::Duration& period) {

  new_cmd_ = cmd_[0]+cmd_[1];

  if(fabs(new_cmd_-last_cmd_) < 1e-5) {
    return;
  }
  last_cmd_ = new_cmd_;

  // ROS_INFO_STREAM("GripperActionServer::write() -- " << new_cmd_);

  wsg_32_common::Move::Request req;
  wsg_32_common::Move::Response res;
   
  nh_.getParamCached("gripper_speed", gripper_speed_);
 
  req.width = new_cmd_;
  req.speed = gripper_speed_;

  if(!moveSrv(req,res)) {
    ROS_ERROR("Could not service request");
    if(current_active_goal_) {
      if(current_active_goal_->getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE) {
        current_active_goal_->setCanceled();
      }
    }
  }

  // ROS_INFO_STREAM("In goal CB "<<res.error);

  if(res.error != 255) {
    control_msgs::GripperCommandResult result;
    result.effort = getGraspingForceLimit()/1000.0;
    result.position = getOpening()/1000.0;
    result.reached_goal = true;
    result.stalled = false;
    
    if(current_active_goal_)
      current_active_goal_->setSucceeded(result);
  } else {
    if(current_active_goal_) {
      if(current_active_goal_->getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE) {
        current_active_goal_->setAborted();
      }
    }
  }

}


void GripperActionServer::run()
{
  ROS_DEBUG("TCP connection stablished");
  ros::Rate loop_rate(30); // loop at 1Hz

  // Setup joint states
  sensor_msgs::JointState joint_state;
  joint_state.position.resize(2);
  joint_state.name.resize(2);

  nh_.getParamCached("gripper_speed", gripper_speed_);

  joint_state.name[1] = "gripper_left_joint";
  joint_state.name[2] = "gripper_right_joint";

  //Loop waiting for orders and updating the state
  //Create the msg to send
  wsg_32_common::Status status_msg;
  setRunning(true);
  while( ros::ok() )
  {
    //Get state values
    const char * aux;
    aux = systemState();
    float op = getOpening();
    float acc = getAcceleration();
    float force = getGraspingForceLimit();

    std::stringstream ss;

    ss << aux;

    status_msg.status = ss.str();
    status_msg.width = (int) op;
    status_msg.acc = (int) acc;
    status_msg.force = (int) force;

    state_pub_.publish(status_msg);

    joint_state.header.stamp = ros::Time::now();
    double val = ((double) op)/1000.0;
    joint_state.position[0] = val/2;
    joint_state.position[1] = val/2;
    joint_states_pub_.publish(joint_state);

    pos_[0] = val/2.0;
    pos_[1] = val/2.0;

    ros::spinOnce();
    loop_rate.sleep();
  }
}
}

/**
 * The main function
 */
int main( int argc, char **argv )
{
  ros::init(argc, argv, "wsg_gripper_driver");
  ros::NodeHandle nh("/gripper_controller");

  std::string ip;
  int port;

  ROS_DEBUG("WSG 32 - ROS NODE");
  nh.param("ip", ip, std::string("192.168.1.53"));
  nh.param("port", port, 1000);

  boost::shared_ptr<wsg_gripper_driver::GripperActionServer> gripper;
  boost::shared_ptr<ros::AsyncSpinner> spinner;

  gripper = boost::make_shared<wsg_gripper_driver::GripperActionServer>(nh);
  controller_manager::ControllerManager cm(&(*(gripper.get())), nh);

  spinner = boost::make_shared<ros::AsyncSpinner>(1);
  spinner->start();

  ros::Rate rate(1.0 / gripper->getPeriod().toSec());
  if( cmd_connect_tcp( ip.c_str(), port ) == 0 ) {
    if(!gripper->initialize()) {
      ROS_ERROR("Unable to initialize gripper action server");
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

  } else
  {
    ROS_ERROR("Unable to connect via TCP, please check the port and address used.");
  }
  gripper->setRunning(false);

  cmd_disconnect();
  ros::shutdown();
  return 0;
}
