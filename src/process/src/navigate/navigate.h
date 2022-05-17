#pragma once
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <process/fsmAction.h>
#include <ros/ros.h>
#include <ros/subscriber.h>

#include <vector>

#include "amr/amr_info.h"
#include "process/mission.h"
class NavigateServer {
   public:
    NavigateServer(std::string actionName);
    ~NavigateServer();

   private:
    void executeCallBack(const process::fsmGoalConstPtr& goal);
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<process::fsmAction> server;

    ros::Subscriber missionSubscriber;
    void missionCallBack(const process::mission::ConstPtr& mission);

    ros::Publisher navigatePublisher;
    ros::Publisher movePublisher;
    ros::Subscriber agvSubscriber;
    void agvCallBack(const amr::amr_info::ConstPtr& info);

    void move(std::vector<double> pos);
    void nav(int posNum);
    void waitForIdle();
    bool isMoving;

    int agvPos;
    int agvPrePos;

    std::vector<float> agvXYZ;
    std::vector<double> rebaseValue;

    bool check;
};
