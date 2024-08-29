#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class DeadReckoningNode : public rclcpp::Node {
public:
    DeadReckoningNode();
    void move_robot(double linear_speed, double angular_speed, double distance, bool forward);

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    double add_gaussian_noise(double mean, double stddev);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    double current_x_;
    double current_y_;
    double current_theta_;

    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;
};

#endif // DEAD_RECKONING_H