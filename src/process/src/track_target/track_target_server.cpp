#include "track_target_server.h"

#include "object_degree.h"
#include "process/fsmAction.h"
#include "process/fsmActionResult.h"
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
      detector("~/model/yolov4/yolov4-tiny.cfg",
               "~/model/yolov4/yolov4-tiny.weights") {
    arm.setAccel(50);
    arm.setSpeed(5);

    Config config;
    rs.connect(config);

    if (rs.depth_supports(RS2_OPTION_MIN_DISTANCE))
        rs.set_depth_option(RS2_OPTION_MIN_DISTANCE, 200);
    rs.set_align_stream(RS2_STREAM_COLOR);
    rs.set_colorizer_option(RS2_OPTION_COLOR_SCHEME, 3);
    rs.enable_hole_filling_filter(true);
    rs.set_hole_filling_filter_option(RS2_OPTION_HOLES_FILL, 1);

    error.resize(3);
    orientation = .0;

    state = INIT;
    isFinish = false;

    ROS_INFO("WAITING FOR REQUEST");
    server.start();
}

TrackTargetServer::~TrackTargetServer() { server.shutdown(); }

void TrackTargetServer::executeCallBack(const process::fsmGoalConstPtr &goal) {
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
                ret = targetEstimate();
                ROS_INFO("STATE EXTRANGE: TARGET_ESTIMATE");
                if (error[0] + error[1] > 5) {
                    state = TRACKING;
                } else {
                    state = GRIP;
                }
                break;
            }
            case TRACKING: {
                ret = tracking();
                ROS_INFO("STATE EXTRANGE: TRACKING");
                state = TARGET_ESTIMATE;
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
    tm.waitForIdle();

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

    int centerX = predict[0].x + (predict[0].w / 2);
    int centerY = predict[0].y + (predict[0].h / 2);

    this->error[0] = 1000 * pointCloud.at<cv::Vec3f>(centerY, centerX)[0];
    this->error[1] = -1000 * pointCloud.at<cv::Vec3f>(centerY, centerX)[1];
    this->error[2] = -1000 * pointCloud.at<cv::Vec3f>(centerY, centerX)[2];
    this->orientation = predict[0].degree;

    return 0;
}
int TrackTargetServer::tracking() {
    int ret = 0;
    vector<double> position = {error[0], error[1], 0, 0, 0, 0};
    ret = arm.move(position, 10, Arm::MoveType::Relative, Arm::CtrlType::PTP,
                   Arm::CoordType::CARTESIAN);
    tm.waitForIdle();
    return ret;
}
int TrackTargetServer::grip() {
    int ret = 0;
    vector<double> position = {0, 0, -1 * error[3] + 20, 0, 0, orientation};
    ret = arm.move(position, 10, Arm::MoveType::Relative, Arm::CtrlType::PTP,
                   Arm::CoordType::CARTESIAN);
    tm.waitForIdle();

    tm.gripClose();
    return 0;
}

int TrackTargetServer::finish() {
    int ret = 0;
    vector<double> position;
    ret = arm.move(position, 10, Arm::MoveType::Absolute, Arm::CtrlType::PTP,
                   Arm::CoordType::JOINT);

    return ret;
}
int TrackTargetServer::aborted() { return 0; }
