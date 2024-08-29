#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class DeadReckoningNode : public rclcpp::Node {
public:
    DeadReckoningNode();
    void move_robot(double linear_speed, double angular_speed);
};

#endif  // DEAD_RECKONING_H

