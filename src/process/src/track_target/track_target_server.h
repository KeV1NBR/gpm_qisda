#pragma once
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <vector>

#include "arm.h"
#include "process/fsmAction.h"
#include "process/fsmGoal.h"
#include "tm.h"

enum STATE { INIT, TARGET_ESTIMATE, TRACKING, GRIP, FINISH, ABORTED };

class TrackTargetServer {
   public:
    TrackTargetServer(std::string actionName);
    ~TrackTargetServer();

   private:
    void executeCallBack(const process::fsmGoalConstPtr& goal);

    int init();
    int targerEstimate();
    int tracking();
    int grip();
    int finish();
    int aborted();

    STATE state;
    std::vector<double> error;
    bool isFinish;

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<process::fsmAction> server;
    std::string name;
    Arm arm;
    Tm tm;
};
