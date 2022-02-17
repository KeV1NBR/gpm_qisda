#pragma once
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include "arm.h"
#include "fsm_struct/fsmAction.h"
#include "fsm_struct/fsmGoal.h"
#include "tm.h"
class TrackTargetServer {
   public:
    TrackTargetServer(std::string actionName);
    ~TrackTargetServer();
    void executeCallBack(const fsm_struct::fsmGoalConstPtr& goal);

   private:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<fsm_struct::fsmAction> server;
    std::string name;
    Arm arm;
    Tm tm;
};
