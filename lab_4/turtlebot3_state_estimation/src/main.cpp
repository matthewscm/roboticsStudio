#include "rclcpp/rclcpp.hpp"
#include "dead_reckoning.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeadReckoningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
