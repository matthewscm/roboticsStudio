#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <random>
#include <cmath>

class DeadReckoningNode : public rclcpp::Node {
public:
    DeadReckoningNode();

private:
    void move_robot(double linear_speed, double angular_speed);
    void publish_cmd_vel(double linear_speed, double angular_speed);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    // State variables
    double x_, y_, theta_;
    double start_x_, start_y_;
    double distance_;
    bool forward_;
   // Parameters
    double linear_speed_;
    double angular_speed_;

    // Noise parameters
    std::default_random_engine generator_;
    std::normal_distribution<double> noise_distribution_;


};

#endif  // DEAD_RECKONING_H

