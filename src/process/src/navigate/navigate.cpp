#include "navigate.h"

#include <chrono>
#include <string>
#include <thread>

#include "amr/amr_info.h"
#include "geometry_msgs/Twist.h"
#include "process/mission.h"

using namespace std;
using namespace ros;

NavigateServer::NavigateServer(string actionName)
    : server(nh, actionName,
             boost::bind(&NavigateServer::executeCallBack, this, _1), false) {
    missionSubscriber = nh.subscribe<process::mission>(
        "/gpm/mission", 1, &NavigateServer::missionCallBack, this);
    agvSubscriber = nh.subscribe<amr::amr_info>(
        "/amr/robot_state", 1, &NavigateServer::agvCallBack, this);

    navigatePublisher = nh.advertise<geometry_msgs::Twist>("/amr/nav", 1000);
    movePublisher = nh.advertise<geometry_msgs::Twist>("/amr/move", 1000);
    check = false;
    server.start();
    agvPos = 0;
    agvPrePos = 0;
    isMoving = false;

    agvXYZ.resize(0);
    rebaseValue.resize(6, .0);
}

NavigateServer::~NavigateServer() { server.shutdown(); }

void NavigateServer::executeCallBack(const process::fsmGoalConstPtr& goal) {
    spinOnce();
    this_thread::sleep_for(chrono::milliseconds(500));
    ROS_INFO("move agv to position %d", agvPos);
    if (check == true) {
        double sum = .0;
        for (uint i = 0; i < rebaseValue.size() - 1; i++) {
            rebaseValue[i] = -1 * rebaseValue[i];
            sum += rebaseValue[i];
        }
        if (abs(sum) >= 0.01) {
            move(rebaseValue);
            waitForIdle();
        }

        if(this->agvPos == 22){
            move({1.4, .0, .0, .0, .0, 180.});
            nav(22);
        } else if (this->agvPos >= 4) {
            // move to pos 21
            if (this->agvPrePos >= 4 && this->agvPrePos != 0 ) {
            } else if (this->agvPrePos == 0 ) {
                cout<<"first move"<<endl;
                nav(21);
            } else {
                move({1.4, .0, .0, .0, .0, 180.});
                move({.0, .0, .0, .0, .0, -90});
                move({2.1, .0, .0, .0, .0, -90.});
                move({.0, .0, .0, .0, .0, 0.});
                nav(21);
            }
        } else {
            // move to pos 20
            if (this->agvPrePos >= 4 && this->agvPrePos != 0) {
                move({1.4, .0, .0, .0, .0, .0});
                move({.0, .0, .0, .0, .0, 90.});
                move({2.1, .0, .0, .0, .0, 90.});
                move({.0, .0, .0, .0, .0, 180.});
                nav(20);
            } else {
            }
        }

        spinOnce();
        for (auto it : agvXYZ) {
            cout << it << ", ";
        }
        cout << endl;
        if (agvPos == 1 || agvPos == 4) {
            rebaseValue = {0.3, 0.45, .0, .0, .0, agvXYZ[2]};
            move(rebaseValue);
        } else if (agvPos == 2 || agvPos == 5) {
            rebaseValue = {-0.3, 0.45, .0, .0, .0, agvXYZ[2]};
            move(rebaseValue);
        } else if (agvPos == 3 || agvPos == 6) {
            rebaseValue = {.0, -0.45, .0, .0, .0, agvXYZ[2]};
            move(rebaseValue);
        } else {
            rebaseValue = {.0, .0, .0, .0, .0, .0};
        }

        server.setSucceeded();
    } else {
        ROS_ERROR("error query!!");
        server.setAborted();
    }
    check = false;
}

void NavigateServer::missionCallBack(
    const process::mission::ConstPtr& mission) {
    int agv_pos = mission->agv_pos;

    if (agv_pos != 999) {
        check = true;

        this->agvPrePos = this->agvPos;
        this->agvPos = agv_pos;
    }
}

void NavigateServer::move(vector<double> pos) {
    geometry_msgs::Twist goalPosition;
    goalPosition.linear.x = pos[0];
    goalPosition.linear.y = pos[1];
    goalPosition.angular.z = pos[5];

    movePublisher.publish(goalPosition);

    this->waitForIdle();
}

void NavigateServer::nav(int posNum) {
    geometry_msgs::Twist goalPosition;
    spinOnce();

    string paramName = "/agv_pos/" + to_string(posNum) + "/";
    nh.getParam(paramName + "x", goalPosition.linear.x);
    nh.getParam(paramName + "y", goalPosition.linear.y);
    nh.getParam(paramName + "rz", goalPosition.angular.z);

    navigatePublisher.publish(goalPosition);

    this->waitForIdle();

    spinOnce();
    if (posNum == 20) {
        move({-1 * goalPosition.linear.x + agvXYZ[0],
              -1 * goalPosition.linear.y + agvXYZ[1], 0, 0, 0,
              goalPosition.angular.z});
    } else {
        move({goalPosition.linear.x - agvXYZ[0],
              goalPosition.linear.y - agvXYZ[1], 0, 0, 0,
              goalPosition.angular.z});
    }
}

void NavigateServer::waitForIdle() {
    int count = 0;
    std::chrono::system_clock::time_point timeStamp =
        std::chrono::system_clock::now();

    spinOnce();
    while (isMoving == false) {
        std::chrono::system_clock::time_point now =
            std::chrono::system_clock::now();

        if ((now - timeStamp).count() / 1000000 > 5000) {
            cout << "timeout" << endl;
            break;
        }
        spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    while (true) {
        spinOnce();
        if (isMoving == false) {
            count++;
        } else {
            count = 0;
        }
        if (count > 10) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    cout << "move ok" << endl;
}

void NavigateServer::agvCallBack(const amr::amr_info::ConstPtr& info) {
    if (info->motion == "idle" || info->motion == "pause") {
        this->isMoving = false;
    } else {
        this->isMoving = true;
    }

    agvXYZ = info->position;
}
