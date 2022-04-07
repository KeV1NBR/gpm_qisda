#include "navigate.h"

#include <chrono>
#include <string>
#include <thread>

#include "geometry_msgs/Twist.h"
#include "process/mission.h"

using namespace std;
using namespace ros;

NavigateServer::NavigateServer(string actionName)
    : server(nh, actionName,
             boost::bind(&NavigateServer::executeCallBack, this, _1), false) {
    missionSubscriber = nh.subscribe<process::mission>(
        "/gpm/mission", 1, &NavigateServer::missionCallBack, this);
    navigatePublisher =
        nh.advertise<geometry_msgs::Twist>("/agv/navigate", 1000);

    server.start();
}

NavigateServer::~NavigateServer() { server.shutdown(); }

void NavigateServer::executeCallBack(const process::fsmGoalConstPtr& goal) {
    spinOnce();
    this_thread::sleep_for(chrono::milliseconds(100));
    ROS_INFO("position : x: %d, y: %d, rz: %d", int(goalPosition.linear.x),
             int(goalPosition.linear.y), int(goalPosition.angular.z));
    navigatePublisher.publish(this->goalPosition);

    server.setSucceeded();
}
void NavigateServer::missionCallBack(
    const process::mission::ConstPtr& mission) {
    int agv_pos = mission->agv_pos;
    string paramName = "/agv_pos/" + to_string(agv_pos) + "/";

    if (agv_pos != 999) {
        ROS_INFO("move agv to position %d", agv_pos);
    }

    nh.getParam(paramName + "x", goalPosition.linear.x);
    nh.getParam(paramName + "y", goalPosition.linear.y);
    nh.getParam(paramName + "rz", goalPosition.angular.z);
}
