#include "navigate.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigate_server");

    NavigateServer server("navigate");
    ros::spin();

    return 0;
}
