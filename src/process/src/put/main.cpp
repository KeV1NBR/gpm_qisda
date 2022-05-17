#include "track_target_server.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "put_server");

    TrackTargetServer server("put");
    ros::spin();

    return 0;
}
