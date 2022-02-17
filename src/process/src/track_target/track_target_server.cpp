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
    //    arm.setAccel(100);
    //  arm.setSpeed(5);
    server.start();
}

TrackTargetServer::~TrackTargetServer() { server.shutdown(); }

void TrackTargetServer::executeCallBack(const process::fsmGoalConstPtr& goal) {
    vector<double> homeJ = {180, 0, 90, 0, 90, 0};
    vector<double> pos1 = {10, 0, 0, 0, 0, 0};
    vector<double> pos2 = {0, -10, 0, 0, 0, 0};
    vector<double> pos3 = {0, 0, -10, 0, 0, 0};
    vector<double> pos4 = {0, 0, 0, -10, 0, 0};
    vector<double> pos5 = {0, 0, 0, 0, 10, 0};
    vector<double> pos6 = {0, 0, 0, 0, 0, 10};
    arm.move(homeJ, 100, Arm::MoveType::Absolute, Arm::CtrlType::PTP,
             Arm::CoordType::JOINT);
    tm.waitForIdle();
    arm.move(pos1, 100, Arm::MoveType::Relative, Arm::CtrlType::PTP,
             Arm::CoordType::JOINT);
    tm.waitForIdle();
    arm.move(homeJ, 100, Arm::MoveType::Absolute, Arm::CtrlType::PTP,
             Arm::CoordType::JOINT);
    tm.waitForIdle();
    arm.move(pos2, 100, Arm::MoveType::Relative, Arm::CtrlType::PTP,
             Arm::CoordType::JOINT);
    tm.waitForIdle();
    arm.move(homeJ, 100, Arm::MoveType::Absolute, Arm::CtrlType::PTP,
             Arm::CoordType::JOINT);
    tm.waitForIdle();
    arm.move(pos3, 100, Arm::MoveType::Relative, Arm::CtrlType::PTP,
             Arm::CoordType::JOINT);
    tm.waitForIdle();
    arm.move(homeJ, 100, Arm::MoveType::Absolute, Arm::CtrlType::PTP,
             Arm::CoordType::JOINT);
    tm.waitForIdle();
    arm.move(pos4, 100, Arm::MoveType::Relative, Arm::CtrlType::PTP,
             Arm::CoordType::JOINT);
    tm.waitForIdle();
    arm.move(homeJ, 100, Arm::MoveType::Absolute, Arm::CtrlType::PTP,
             Arm::CoordType::JOINT);
    tm.waitForIdle();
    arm.move(pos5, 100, Arm::MoveType::Relative, Arm::CtrlType::PTP,
             Arm::CoordType::JOINT);
    tm.waitForIdle();
    arm.move(homeJ, 100, Arm::MoveType::Absolute, Arm::CtrlType::PTP,
             Arm::CoordType::JOINT);
    tm.waitForIdle();
    arm.move(pos6, 100, Arm::MoveType::Relative, Arm::CtrlType::PTP,
             Arm::CoordType::JOINT);
    tm.waitForIdle();

    server.setSucceeded();
}
