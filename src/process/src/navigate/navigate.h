#pragma once
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <process/fsmAction.h>
#include <ros/ros.h>
#include <ros/subscriber.h>

#include <vector>

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

    geometry_msgs::Twist goalPosition;
};
