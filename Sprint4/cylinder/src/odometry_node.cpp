#include "odometry_node.h"

OdometryNode::OdometryNode() : Node("odometry_node") {
    // Create a subscriber for the Odometry message
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&OdometryNode::odom_callback, this, std::placeholders::_1));
}

// Accessor for getting the latest odometry
nav_msgs::msg::Odometry OdometryNode::get_latest_odometry() {
    return robot_pose_;
}

// Callback function for processing odometry messages
void OdometryNode::odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    robot_pose_ = *msg; // Update the robot's pose
    RCLCPP_INFO(this->get_logger(), "Robot's position: x: %f, y: %f", robot_pose_.pose.pose.position.x, robot_pose_.pose.pose.position.y);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto odometry_node = std::make_shared<OdometryNode>();
    rclcpp::spin(odometry_node);
    rclcpp::shutdown();
    return 0;
}