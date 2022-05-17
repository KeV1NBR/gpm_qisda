#include "idle_server.h"

#include <chrono>
#include <regex>
#include <sstream>
#include <thread>

#include "process/mission.h"

using std::cin;
using std::cout;
using std::endl;
using std::flush;
using std::getline;
using std::stoi;
using std::string;

using Poco::Net::SocketAddress;
using Poco::Net::SocketStream;
using Poco::Net::StreamSocket;

IdleServer::IdleServer(string actionName)
    : server(nh, actionName,
             boost::bind(&IdleServer::executeCallBack, this, _1), false) {
    missionPublisher = nh.advertise<process::mission>("/gpm/mission", 1000);

    server.start();
}

IdleServer::~IdleServer() { server.shutdown(); }

void IdleServer::executeCallBack(const process::fsmGoalConstPtr &goal) {
    SocketAddress addr("127.0.0.1", "6666");
    StreamSocket socket(addr);
    string s, tmp;
    std::vector<int> receive;
    process::mission msg;
    msg.mission_name = "unknown";
    msg.object_id = 999;
    msg.agv_pos = 999;

    missionPublisher.publish(msg);

    SocketStream stream(socket);

    string info = "connected to " + addr.toString();
    ROS_INFO("%s", info.c_str());

    while (stream) {
        std::regex reg("\\$([\\d.]+), ([\\d.]+), ([\\d.]+)");

        stream << "request" << endl;
        getline(stream, s);
        if (regex_match(s, reg)) {
            break;
        } else {
            ROS_INFO("wait for mission");
        }
        sleep(1);
    }
    s.erase(s.begin());
    std::istringstream istr(s);
    while (getline(istr, tmp, ',')) {
        receive.push_back(stoi(tmp));
    }

    switch (receive[0]) {
        case 1:
            msg.mission_name = "pick";
            break;
        case 2:
            msg.mission_name = "put";
            break;
        case 3:
            msg.mission_name = "tmp";
            break;
        default:
            msg.mission_name = "unknown";
    }

    msg.object_id = receive[1];
    msg.agv_pos = receive[2];

    missionPublisher.publish(msg);
    ROS_INFO("get mission : %s, object:%d, agv_pos: %d",
             msg.mission_name.c_str(), msg.object_id, msg.agv_pos);

    //    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    server.setSucceeded();
}
