#ifndef LASER_SCAN_SUBSCRIBER_HPP
#define LASER_SCAN_SUBSCRIBER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


class LaserScanSubscriber : public rclcpp::Node
{
public:
    LaserScanSubscriber();

private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    //rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    double target_angle_;
};



#endif // LASER_SCAN_SUBSCRIBER_HPP