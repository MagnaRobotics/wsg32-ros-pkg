#include <wsg_32/wsg_32_ros_interface.h>

namespace wsg_gripper_driver
{

static const double MAX_OPENING = 0.07;
static const double MAX_SPEED = 10.0;//0.42;

Wsg32ROSInterface::Wsg32ROSInterface(ros::NodeHandle &nh) : 
  nh_(nh)
{

  // Services
  moveSS = nh_.advertiseService("/wsg_gripper_driver/move", &Wsg32ROSInterface::moveSrv, this);
  graspSS = nh_.advertiseService("/wsg_gripper_driver/grasp", &Wsg32ROSInterface::graspSrv, this);
  releaseSS = nh_.advertiseService("/wsg_gripper_driver/release", &Wsg32ROSInterface::releaseSrv, this);
  homingSS = nh_.advertiseService("/wsg_gripper_driver/homing", &Wsg32ROSInterface::homingSrv, this);
  stopSS = nh_.advertiseService("/wsg_gripper_driver/stop", &Wsg32ROSInterface::stopSrv, this);
  ackSS = nh_.advertiseService("/wsg_gripper_driver/ack", &Wsg32ROSInterface::ackSrv, this);
  incrementSS = nh_.advertiseService("/wsg_gripper_driver/move_incrementally", &Wsg32ROSInterface::incrementSrv, this);
  setAccSS = nh_.advertiseService("/wsg_gripper_driver/set_acceleration", &Wsg32ROSInterface::setAccSrv, this);
  setForceSS = nh_.advertiseService("/wsg_gripper_driver/set_force", &Wsg32ROSInterface::setForceSrv, this);

  // Publishers
  state_pub_ = nh_.advertise<wsg_32_common::Status>("/wsg_gripper_driver/status", 10);

  // home the gripper
  ack_fault();
  homing();

  ROS_INFO_STREAM("Wsg32ROSInterface() -- using force: " << getGraspingForceLimit());
  setGraspingForceLimit(15);

  ROS_INFO("Wsg32ROSInterface() -- ready");

}


bool Wsg32ROSInterface::init(std::string ip, int port) {
  return (cmd_connect_tcp( ip.c_str(), port ) == 0);
}


void Wsg32ROSInterface::shutdown() {

  cmd_disconnect();

  moveSS.shutdown();
  graspSS.shutdown();
  releaseSS.shutdown();
  homingSS.shutdown();
  stopSS.shutdown();
  ackSS.shutdown();
  incrementSS.shutdown();
  setAccSS.shutdown();
  setForceSS.shutdown();

}


bool Wsg32ROSInterface::update(double &current_opening) {

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

  current_opening = (double)op;

  return true;
}


bool Wsg32ROSInterface::setPosition(double val, double speed)  {

  wsg_32_common::Move::Request req;
  wsg_32_common::Move::Response res;

  req.width = val;
  req.speed = speed;

  if(!moveSrv(req,res)) {
    ROS_ERROR_STREAM("Wsg32ROSInterface::setPosition() -- could not service request");
    return false;
  }
  
  return true;
}
  

bool Wsg32ROSInterface::moveSrv(wsg_32_common::Move::Request &req, wsg_32_common::Move::Response &res) {

  if ( (req.width >= 0.0 && req.width <= MAX_OPENING) && (req.speed > 0.0 && req.speed <= MAX_SPEED) ) {
    ROS_INFO("Wsg32ROSInterface::moveSrv() -- Moving to %f position at %f mm/s.", req.width, req.speed);
    res.error = move(req.width * 1000, req.speed * 1000, false, false);
  } else if (req.width < 0.0 || req.width > MAX_OPENING) {
    ROS_ERROR("Wsg32ROSInterface::moveSrv() -- Impossible to move to this position. (Width values: [0.0 - %f] m", MAX_OPENING);
    res.error = 255;
    return true;
  }
  else {
    ROS_WARN("Wsg32ROSInterface::moveSrv() -- Speed values are outside the gripper's physical limits ([0.0001 - %f m/s])  Using clamped values.", MAX_SPEED);
    res.error = move(req.width * 1000, req.speed * 1000, false, false);
  }
  if (res.error == 255) {
    ack_fault();
  }
  ROS_DEBUG("Target position reached.");
  return true;

}


bool Wsg32ROSInterface::graspSrv(wsg_32_common::Move::Request &req, wsg_32_common::Move::Response &res) {

  if ( (req.width >= 0.0 && req.width <= MAX_OPENING) && (req.speed > 0.0 && req.speed <= MAX_SPEED) ) {
    ROS_DEBUG("Grasping object of width %f at %f mm/s.", req.width, req.speed);
    res.error = grasp(req.width*1000, req.speed*1000);
  } else if (req.width < 0.0 || req.width > MAX_OPENING) {
    ROS_ERROR("Wsg32ROSInterface::graspSrv() -- Impossible to move to position %lf. (Width values: [0.0 - %f] ", req.width, MAX_OPENING);
    res.error = 255;
    return false;
  } else {
    ROS_WARN("Wsg32ROSInterface::graspSrv() -- Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: [0.1 - %f])  Using clamped values.", MAX_SPEED);
    res.error = grasp(req.width, req.speed);
  }

  ROS_DEBUG("Object grasped correctly.");
  objectGraspped=true;
  return true;

}


bool Wsg32ROSInterface::incrementSrv(wsg_32_common::Incr::Request &req, wsg_32_common::Incr::Response &res) {
  
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
  
  } else if (req.direction == "close"){

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


bool Wsg32ROSInterface::releaseSrv(wsg_32_common::Move::Request &req, wsg_32_common::Move::Response &res) {

  if ( (req.width >= 0.0 && req.width <= MAX_OPENING) && (req.speed > 0.0 && req.speed <= MAX_SPEED) ){
    ROS_DEBUG("Releasing to %f position at %f mm/s.", req.width, req.speed);
    res.error = release(req.width*1000, req.speed*1000);
  }else if (req.width < 0.0 || req.width > MAX_OPENING){
    ROS_ERROR("Wsg32ROSInterface::releaseSrv() -- Imposible to move to this position. (Width values: [0.0 - %f] ", MAX_OPENING);
    res.error = 255;
    return false;
  }else{
    ROS_WARN("Wsg32ROSInterface::releaseSrv() -- Speed or position values are outside the gripper's physical limits (Position: [0.0 - %f] / Speed: [0.0001 - %f])  Using clamped values.", MAX_OPENING, MAX_SPEED);
    res.error = release(req.width*1000, req.speed*1000);
  }
  ROS_DEBUG("Object released correctly.");
  return true;

}


bool Wsg32ROSInterface::homingSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
  ROS_DEBUG("Homing...");
  homing();
  ROS_DEBUG("Home position reached.");
  return true;
}


bool Wsg32ROSInterface::stopSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
  ROS_WARN("Wsg32ROSInterface::stopSrv() -- Stop!");
  stop();
  ROS_WARN("Wsg32ROSInterface::stopSrv() -- Stopped.");
  return true;
}


bool Wsg32ROSInterface::setAccSrv(wsg_32_common::Conf::Request &req, wsg_32_common::Conf::Response &res) {
  setAcceleration(req.val*1000);
  return true;
}


bool Wsg32ROSInterface::setForceSrv(wsg_32_common::Conf::Request &req, wsg_32_common::Conf::Response &res) {
  setGraspingForceLimit(req.val);
  return true;
}


bool Wsg32ROSInterface::ackSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
  stop();
  ack_fault();
  return true;
}


}

