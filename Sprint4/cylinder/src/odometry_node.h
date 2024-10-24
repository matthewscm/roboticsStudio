#ifndef ODOMETRY_NODE_HPP
#define ODOMETRY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdometryNode : public rclcpp::Node {
public:
    // Constructor
    OdometryNode();

    // Accessor for getting the latest odometry
    nav_msgs::msg::Odometry get_latest_odometry();

private:
    // Subscriber for the Odometry message
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    nav_msgs::msg::Odometry robot_pose_; // To store the robot's pose

    // Callback function for processing odometry messages
    void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
};

#endif // ODOMETRY_NODE_HPP
