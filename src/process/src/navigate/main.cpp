#include "navigate.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigate_server");

    NavigateServer server("idle");
    ros::spin();

    return 0;
}
