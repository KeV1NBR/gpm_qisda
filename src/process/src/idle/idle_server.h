#pragma once
#include <Poco/Net/SocketStream.h>
#include <Poco/Net/StreamSocket.h>
#include <actionlib/server/simple_action_server.h>
#include <process/fsmAction.h>
#include <ros/ros.h>

#include <vector>

class IdleServer {
   public:
    IdleServer(std::string actionName);
    ~IdleServer();

   private:
    void executeCallBack(const process::fsmGoalConstPtr& goal);
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<process::fsmAction> server;
};
