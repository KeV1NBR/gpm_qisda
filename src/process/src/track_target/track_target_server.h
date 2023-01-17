#pragma once
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <vector>

#include "arm.h"
#include "object_degree.h"
#include "process/fsmAction.h"
#include "process/fsmGoal.h"
#include "process/mission.h"
#include "process/store.h"
#include "realsense.h"
#include "tm.h"
#include "yolo_v2_class.hpp"

enum STATE { INIT, TARGET_ESTIMATE, TRACKING, GRIP, PUT, FINISH, ABORTED };

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
    int put();

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

    process::store goodsList;
    ros::Subscriber goodsSubscriber;
    ros::Publisher goodsPublisher;

    ros::Subscriber missionSubscriber;
    void missionCallBack(const process::mission::ConstPtr& mission);

    void goodsCallBack(const process::store::ConstPtr& goods);
    int targetNum;
    int objId;
    int agvPos;

};
