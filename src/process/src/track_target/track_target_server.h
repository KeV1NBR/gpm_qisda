#pragma once
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include "arm.h"
#include "process/fsmAction.h"
#include "process/fsmGoal.h"
#include "tm.h"
class TrackTargetServer {
   public:
    TrackTargetServer(std::string actionName);
    ~TrackTargetServer();
    void executeCallBack(const process::fsmGoalConstPtr& goal);

   private:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<process::fsmAction> server;
    std::string name;
    Arm arm;
    Tm tm;
};
