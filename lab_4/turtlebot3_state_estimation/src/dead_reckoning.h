#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  
#include <random>

class DeadReckoningNode : public rclcpp::Node {
public:
    DeadReckoningNode();
    // Destructor
    ~DeadReckoningNode();
    // Functions
    void move_robot(double linear_speed, double angular_speed);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timer_callback();

private:
 // State variables
    double x_;
    double y_;
    double theta_;
    double start_x_;
    double start_y_;
    double distance_;
    bool forward_;
    

    
    // Parameters
    double linear_speed_;
    double angular_speed_;

    // Gaussian noise parameters
    std::default_random_engine generator_;
    std::normal_distribution<double> noise_distribution_;

    // Publishers and Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimated_odom_pub;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;


    // Helper function to check if the target distance has been reached
    bool has_reached_distance(double tolerance);
};

#endif  // DEAD_RECKONING_H
