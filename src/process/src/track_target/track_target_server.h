#pragma once
#include <Poco/Net/SocketStream.h>
#include <Poco/Net/StreamSocket.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <vector>

#include "arm.h"
#include "object_degree.h"
#include "process/fsmAction.h"
#include "process/fsmGoal.h"
#include "realsense.h"
#include "tm.h"

enum STATE { INIT, TARGET_ESTIMATE, TRACKING, GRIP, FINISH, ABORTED };

class TrackTargetServer {
   public:
    TrackTargetServer(std::string actionName);
    ~TrackTargetServer();

   private:
    void executeCallBack(const process::fsmGoalConstPtr& goal);

    int init();
    int targetEstimate();
    int tracking();
    int grip();
    int finish();
    int aborted();

    STATE state;
    std::vector<double> error;
    double orientation;
    bool isFinish;

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<process::fsmAction> server;
    ros::ServiceClient yoloClient;

    std::string name;
    Arm arm;
    Tm tm;

    realsense::RealSense rs;

    Detector_deg detector;
};
