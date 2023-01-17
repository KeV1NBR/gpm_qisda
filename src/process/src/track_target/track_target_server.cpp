#include "track_target_server.h"

#include <omp.h>

#include <chrono>
#include <thread>

#include "object_degree.h"
#include "process/fsmAction.h"
#include "process/fsmActionResult.h"
#include "process/store.h"
#include "realsense.h"

using namespace std;
using namespace realsense;
using namespace cv;

TrackTargetServer::TrackTargetServer(string actionName)
    : server(nh, actionName,
             boost::bind(&TrackTargetServer::executeCallBack, this, _1), false),
      name(actionName),
      arm("manipulator"),
      tm(nh),
      detector("/home/qrobot/model/yolov4/yolov4-tiny.cfg",
               "/home/qrobot/model/yolov4/yolov4-tiny_20000.weights") {
    arm.setAccel(50);
    arm.setSpeed(50);

    Config config;
    rs.connect(config);

    rs.set_depth_option(RS2_OPTION_MIN_DISTANCE, 200);
    rs.set_colorizer_option(RS2_OPTION_MAX_DISTANCE, .4f);
    rs.set_colorizer_option(RS2_OPTION_MIN_DISTANCE, .3f);
    rs.set_colorizer_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, false);

    rs.set_align_stream(RS2_STREAM_COLOR);
    rs.set_colorizer_option(RS2_OPTION_COLOR_SCHEME, 3);
    rs.enable_hole_filling_filter(true);
    rs.set_hole_filling_filter_option(RS2_OPTION_HOLES_FILL, 1);

    goodsSubscriber = nh.subscribe<process::store>(
        "/gpm/goods", 10, &TrackTargetServer::goodsCallBack, this);

    goodsPublisher = nh.advertise<process::store>("/gpm/goods", 1000);

    missionSubscriber = nh.subscribe<process::mission>(
        "/gpm/mission", 1, &TrackTargetServer::missionCallBack, this);

    error.resize(3);
    orientation = .0;
    targetNum = -1;
    objId = -1;
    agvPos = -1;

    state = INIT;
    isFinish = false;

    vector<int> goods = {-1, -1, -1, -1, -1, -1};
    goodsList.goods = goods;

    this_thread::sleep_for(chrono::milliseconds(100));
    goodsPublisher.publish(goodsList);

    ROS_INFO("WAITING FOR REQUEST");
    server.start();
}

TrackTargetServer::~TrackTargetServer() { server.shutdown(); }

void TrackTargetServer::executeCallBack(const process::fsmGoalConstPtr& goal) {
    isFinish = false;
    int ret = 0;

    state = INIT;

    if (this->agvPos != 22) {
        while (isFinish == false) {
            switch (state) {
                case INIT: {
                    ROS_INFO("STATE EXTRANGE: INIT");
                    ret = init();
                    state = TARGET_ESTIMATE;
                    break;
                }
                case TARGET_ESTIMATE: {
                    ROS_INFO("STATE EXTRANGE: TARGET_ESTIMATE");
                    ret = targetEstimate();
                    if (fabs(error[0] + error[1]) > 0.005 || targetNum == -1) {
                        state = TRACKING;
                    } else {
                        state = GRIP;
                    }
                    break;
                }
                case TRACKING: {
                    ROS_INFO("STATE EXTRANGE: TRACKING");
                    ret = tracking();
                    state = TARGET_ESTIMATE;
                    break;
                }
                case GRIP: {
                    ROS_INFO("STATE EXTRANGE: GRIP");
                    ret = grip();
                    state = PUT;
                    break;
                }
                case PUT: {
                    ROS_INFO("STATE EXTRANGE: PUT");
                    ret = put();
                    state = FINISH;
                    break;
                }
                case FINISH: {
                    ROS_INFO("STATE EXTRANGE: FINISH");
                    ret = finish();
                    isFinish = true;
                    ROS_INFO("WAITING FOR REQUEST");
                    server.setSucceeded();
                    break;
                }
                case ABORTED: {
                    ROS_ERROR("ABORTED");
                    ret = aborted();
                    isFinish = true;
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
    server.setSucceeded();
}

int TrackTargetServer::init() {
    int ret = 0;

    tm.gripperOpen();

    cerr << agvPos;
    if (agvPos == 3 || agvPos == 6) {
        vector<double> position = {123.863, 8.81798, 63.5105,
                                   16.9489, 89.1952, 32.9309};

        ret = arm.move(position, 50, Arm::MoveType::Absolute,
                       Arm::CtrlType::PTP, Arm::CoordType::JOINT);

        tm.waitForIdle();

    } else {
        vector<double> position = {237.729, 5.10428, 61.8752,
                                   23.0111, 90.0007, -32.2791};

        ret = arm.move(position, 50, Arm::MoveType::Absolute,
                       Arm::CtrlType::PTP, Arm::CoordType::JOINT);

        tm.waitForIdle();
    }
    return ret;
}
int TrackTargetServer::targetEstimate() {
    Mat color;
    Mat depth_image;
    Mat pointCloud;

    rs.update();
    rs.retrieve_color_image(color);
    rs.retrieve_depth_image(depth_image);
    rs.retrieve_xyz_measure(pointCloud);

    std::vector<bbox_t_deg> predict =
        detector.detectWithDeg(color, depth_image);

    targetNum = -1;
    if (predict.size() != 0) {
        for (int i = 0; i < predict.size(); i++) {
            if (predict[i].obj_id == objId) {
                targetNum = i;
                break;
            }
        }
    }

    if (targetNum != -1) {
        int centerX = predict[targetNum].x + (predict[targetNum].w / 2);
        int centerY = predict[targetNum].y + (predict[targetNum].h / 2);

        this->error[0] = -1 * pointCloud.at<cv::Vec3f>(centerY, centerX)[0];
        this->error[1] = pointCloud.at<cv::Vec3f>(centerY, centerX)[1];
        this->error[2] = -1 * pointCloud.at<cv::Vec3f>(centerY, centerX)[2];
        this->orientation = predict[targetNum].degree;
    }
    ROS_INFO("track id: %d, num: %d, pos: %lf, %lf, %lf", objId, targetNum,
             error[0], error[1], orientation);
    return 0;
}
int TrackTargetServer::tracking() {
    int ret = 0;
    vector<double> position = {error[0], error[1], 0, 0, 0, 0};

    if (agvPos == 3 || agvPos == 6) {
        position[0] = -position[0];
        position[1] = -position[1];
    }
    ret = arm.move(position, 10, Arm::MoveType::Relative, Arm::CtrlType::PTP,
                   Arm::CoordType::CARTESIAN);
    tm.waitForIdle();
    return ret;
}
int TrackTargetServer::grip() {
    int ret = 0;

    vector<double> position = {0, -0.08, -0.203, 0, 0, orientation};
    if (agvPos == 3 || agvPos == 6) {
        position[0] = -position[0];
        position[1] = -position[1];
        position[2] = -0.175;
    }
    ret = arm.move(position, 10, Arm::MoveType::Relative, Arm::CtrlType::PTP,
                   Arm::CoordType::CARTESIAN);
    tm.waitForIdle();

    tm.gripperClose();
    this_thread::sleep_for(chrono::milliseconds(3000));

    position = {0, 0, 0.175, 0, 0, 0};
    ret = arm.move(position, 10, Arm::MoveType::Relative, Arm::CtrlType::PTP,
                   Arm::CoordType::CARTESIAN);
    tm.waitForIdle();

    vector<double> homeJ = {180, -33.8431, 105.578, -26.74, 90, 0};
    ret = arm.move(homeJ, 50, Arm::MoveType::Absolute, Arm::CtrlType::PTP,
                   Arm::CoordType::JOINT);
    tm.waitForIdle();

    return 0;
}

int TrackTargetServer::put() {
    ros::spinOnce();
    int goalNum = -1;
    int ret = 0;

    this_thread::sleep_for(chrono::milliseconds(100));
    for (int i = 0; i < goodsList.goods.size(); i++) {
        if (goodsList.goods[i] == -1) {
            goalNum = i;
            break;
        }
    }
    if (goalNum != -1) {
        std::vector<double> goalPosition = {0, 0, 0, 45, 0, 0};
        string paramName = "/arm_pos/put/" + to_string(goalNum) + "/";
        nh.getParam(paramName + "x", goalPosition[0]);
        nh.getParam(paramName + "y", goalPosition[1]);
        nh.getParam(paramName + "z", goalPosition[2]);
        nh.getParam(paramName + "rz", goalPosition[5]);
        cout << goalNum << endl;
        for (auto g : goalPosition) {
            cout << g << ", ";
        }
        goodsList.goods[goalNum] = this->objId;
        goodsPublisher.publish(goodsList);

        ret = arm.move(goalPosition, 50, Arm::MoveType::Relative,
                       Arm::CtrlType::PTP, Arm::CoordType::CARTESIAN);
        tm.waitForIdle();

        ret = arm.move({.0, .0, -0.05, .0, .0, .0}, 50, Arm::MoveType::Relative,
                       Arm::CtrlType::PTP, Arm::CoordType::CARTESIAN);
        tm.waitForIdle();

        tm.gripperOpen();
        this_thread::sleep_for(chrono::milliseconds(100));
        ret = arm.move({.0, .0, 0.05, .0, .0, .0}, 50, Arm::MoveType::Relative,
                       Arm::CtrlType::PTP, Arm::CoordType::CARTESIAN);
        tm.waitForIdle();

    } else {
        ROS_ERROR("none position to put");
        return -1;
    }
    return ret;
}

int TrackTargetServer::finish() {
    int ret = 0;

    vector<double> homeJ = {180, -33.8431, 105.578, -26.74, 90, 0};
    ret = arm.move(homeJ, 100, Arm::MoveType::Absolute, Arm::CtrlType::PTP,
                   Arm::CoordType::JOINT);
    tm.waitForIdle();

    return ret;
}
int TrackTargetServer::aborted() { return 0; }

void TrackTargetServer::goodsCallBack(const process::store::ConstPtr& goods) {
    this->goodsList = *goods;
}

void TrackTargetServer::missionCallBack(
    const process::mission::ConstPtr& mission) {
    objId = mission->object_id;
    agvPos = mission->agv_pos;
}
