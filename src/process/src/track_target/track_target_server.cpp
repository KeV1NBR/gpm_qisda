#include "track_target_server.h"

#include "process/fsmAction.h"
#include "process/fsmActionResult.h"

using namespace std;

TrackTargetServer::TrackTargetServer(string actionName)
    : server(nh, actionName,
             boost::bind(&TrackTargetServer::executeCallBack, this, _1), false),
      name(actionName),
      arm("manipulator"),
      tm(nh) {
    arm.setAccel(50);
    arm.setSpeed(5);

    error.resize(2);
    state = INIT;
    isFinish = false;

    ROS_INFO("WAITING FOR REQUEST");
    server.start();
}

TrackTargetServer::~TrackTargetServer() { server.shutdown(); }

void TrackTargetServer::executeCallBack(const process::fsmGoalConstPtr& goal) {
    isFinish = false;
    int ret = 0;
    while (isFinish == false) {
        switch (state) {
            case INIT: {
                ret = init();
                ROS_INFO("STATE EXTRANGE: INIT");
                state = TARGET_ESTIMATE;
                break;
            }
            case TARGET_ESTIMATE: {
                ret = targerEstimate();
                ROS_INFO("STATE EXTRANGE: TARGET_ESTIMATE");
                state = TRACKING;
                break;
            }
            case TRACKING: {
                ret = tracking();
                ROS_INFO("STATE EXTRANGE: TRACKING");
                if (error[0] + error[1] > 5) {
                    state = TARGET_ESTIMATE;
                } else {
                    state = GRIP;
                }
                break;
            }
            case GRIP: {
                ret = grip();
                ROS_INFO("STATE EXTRANGE: GRIP");
                state = FINISH;
                break;
            }
            case FINISH: {
                ret = finish();
                isFinish = true;
                ROS_INFO("STATE EXTRANGE: FINISH");
                ROS_INFO("WAITING FOR REQUEST");
                server.setSucceeded();
                break;
            }
            case ABORTED: {
                ret = aborted();
                isFinish = true;
                ROS_ERROR("ABORTED");
                server.setAborted();

                break;
            }
            default: {
                isFinish = true;
                ROS_ERROR("error state entry");
                server.setAborted();
                break;
            }
        }
        if (ret < 0) {
            state = ABORTED;
        }
    }
}

int TrackTargetServer::init() {
    int ret = 0;
    vector<double> position;
    ret = arm.move(position, 100, Arm::MoveType::Absolute, Arm::CtrlType::PTP,
                   Arm::CoordType::JOINT);

    return ret;
}
int TrackTargetServer::targerEstimate() { return 0; }
int TrackTargetServer::tracking() {
    int ret = 0;
    vector<double> position = {error[0], 0, error[1], 0, 0, 0};
    ret = arm.move(position, 10, Arm::MoveType::Relative, Arm::CtrlType::PTP,
                   Arm::CoordType::CARTESIAN);
    return ret;
}
int TrackTargetServer::grip() { return 0; }

int TrackTargetServer::finish() {
    int ret = 0;
    vector<double> position;
    ret = arm.move(position, 10, Arm::MoveType::Absolute, Arm::CtrlType::PTP,
                   Arm::CoordType::JOINT);

    return ret;
}
int TrackTargetServer::aborted() { return 0; }