#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "common.h"
#include "cmd.h"
#include "functions.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include "wsg_32_common/Status.h"
#include "wsg_32_common/Move.h"
#include "wsg_32_common/Conf.h"
#include "wsg_32_common/Incr.h"


namespace wsg_gripper_driver
{

#define GRIPPER_MAX_OPEN 110.0
#define GRIPPER_MIN_OPEN 0.0

// hack to get current actionserver to accept 3 command inputs
#define GRIPPER_RELEASE 1000000.0

class Wsg32ROSInterface 
{


public:

  Wsg32ROSInterface(ros::NodeHandle &nh);

  bool init(std::string ip, int port);
  void shutdown();

  bool update(double &current_opening);

  bool setPosition(double val, double speed);

private:

  // driver services
  bool moveSrv(wsg_32_common::Move::Request &req, wsg_32_common::Move::Response &res);
  bool graspSrv(wsg_32_common::Move::Request &req, wsg_32_common::Move::Response &res);
  bool incrementSrv(wsg_32_common::Incr::Request &req, wsg_32_common::Incr::Response &res);
  bool releaseSrv(wsg_32_common::Move::Request &req, wsg_32_common::Move::Response &res);
  bool homingSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);
  bool stopSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);
  bool setAccSrv(wsg_32_common::Conf::Request &req, wsg_32_common::Conf::Response &res);
  bool setForceSrv(wsg_32_common::Conf::Request &req, wsg_32_common::Conf::Response &res);
  bool ackSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);

  // Services
  ros::ServiceServer moveSS, graspSS, releaseSS, homingSS, stopSS, ackSS, incrementSS, setAccSS, setForceSS;

  // ROS API: Action interface
  ros::Publisher state_pub_;
  
  ros::NodeHandle nh_;

  bool objectGraspped;

};

}
