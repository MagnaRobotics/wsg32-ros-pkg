/*
 * WSG 32 ROS NODE
 * Copyright (c) 2015, SRI International
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
 * \author Sachin Chitta (robot.moveit@gmail.com)
 * \brief WSG-32 ROS gripper action server.
 */


//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "common.h"
#include "cmd.h"
#include "functions.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "std_srvs/Empty.h"

#include "wsg_32_common/Status.h"
#include "wsg_32_common/Move.h"
#include "wsg_32_common/Conf.h"
#include "wsg_32_common/Incr.h"

// ROS messages
#include <control_msgs/GripperCommandAction.h>
// actionlib
#include <actionlib/server/action_server.h>

namespace wsg_gripper_driver
{

typedef actionlib::ActionServer<control_msgs::GripperCommandAction> ActionServer;
typedef boost::shared_ptr<ActionServer> ActionServerPtr;
typedef ActionServer::GoalHandle GoalHandle;

//------------------------------------------------------------------------
// Local macros
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------

#define GRIPPER_MAX_OPEN 110.0
#define GRIPPER_MIN_OPEN 0.0

// hack to get current actionserver to accept 3 command inputs
#define GRIPPER_RELEASE 1000000.0

class GripperActionServer
{

public:

GripperActionServer(ros::NodeHandle &nh);

//------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------

float increment;
bool objectGraspped;
//------------------------------------------------------------------------
// Unit testing
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Local function prototypes
//------------------------------------------------------------------------


//------------------------------------------------------------------------
// Function implementation
//------------------------------------------------------------------------

 void run();

 void cancelCB(GoalHandle gh);

 void goalCB(GoalHandle gh);

 bool initialize();

 bool moveSrv(wsg_32_common::Move::Request &req,
	      wsg_32_common::Move::Response &res);

 bool graspSrv(wsg_32_common::Move::Request &req,
	       wsg_32_common::Move::Response &res);

 bool incrementSrv(wsg_32_common::Incr::Request &req,
		   wsg_32_common::Incr::Response &res);

 bool releaseSrv(wsg_32_common::Move::Request &req,
		 wsg_32_common::Move::Response &res);

 bool homingSrv(std_srvs::Empty::Request &req,
		std_srvs::Empty::Request &res);

 bool stopSrv(std_srvs::Empty::Request &req,
	      std_srvs::Empty::Request &res);

 bool setAccSrv(wsg_32_common::Conf::Request &req,
	       wsg_32_common::Conf::Response &res);

 bool setForceSrv(wsg_32_common::Conf::Request &req,
		  wsg_32_common::Conf::Response &res);

 bool ackSrv(std_srvs::Empty::Request &req,
	     std_srvs::Empty::Request &res);

 bool preemptActiveCallback();

 // Services
 ros::ServiceServer moveSS, graspSS, releaseSS, homingSS, stopSS, ackSS, incrementSS, setAccSS, setForceSS;

 // ROS API: Action interface
 ActionServerPtr action_server_;
 ros::Publisher state_pub_;
 ros::Publisher joint_states_pub_;

 ros::NodeHandle nh_;

 boost::shared_ptr<GoalHandle> current_active_goal_;
 bool is_running_;
 double gripper_speed_;
};

}