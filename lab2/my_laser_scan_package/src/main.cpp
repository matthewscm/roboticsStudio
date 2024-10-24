#include "subscriber.h"

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScanSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}