#include "idle_server.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "idle_server");

    IdleServer server("idle");
    ros::spin();

    return 0;
}
