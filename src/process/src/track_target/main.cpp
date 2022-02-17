#include "track_target_server.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "track_target_server");

    TrackTargetServer server("track_target");
    ros::spin();

    return 0;
}
