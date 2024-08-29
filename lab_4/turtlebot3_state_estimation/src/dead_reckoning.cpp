#include "dead_reckoning.h"

// Constructor
DeadReckoningNode::DeadReckoningNode() : Node("dead_reckoning") {
    // Initialize state variables
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    start_x_ = 0.0;
    start_y_ = 0.0;
    distance_ = 0.0;
    forward_ = true;

    // Initialize noise parameters
    generator_ = std::default_random_engine();
    noise_distribution_ = std::normal_distribution<double>(0.0, 0.01);

    // Initialize parameters
    this->declare_parameter("linear_speed", 0.1);
    this->declare_parameter("angular_speed", 0.1);
    this->declare_parameter("distance", 1.0);
    this->declare_parameter("forward", true);
    this->get_parameter("linear_speed", linear_speed_);
    this->get_parameter("angular_speed", angular_speed_);
    this->get_parameter("distance", distance_);
    this->get_parameter("forward", forward_);

    // Initialize publishers
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Initialize subscribers
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&DeadReckoningNode::odom_callback, this, std::placeholders::_1)
    );

    // Initialize timer
    auto timer_callback = std::bind(&DeadReckoningNode::timer_callback, this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
}

void DeadReckoningNode::move_robot(double linear_speed, double angular_speed) {
    // Create a twist message
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = linear_speed;
    twist.angular.z = angular_speed;

    // Publish the twist message
    cmd_vel_pub->publish(twist);
}

void DeadReckoningNode::publish_cmd_vel(double linear_speed, double angular_speed) {
    // Create a twist message
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = linear_speed;
    twist.angular.z = angular_speed;

    // Publish the twist message
    cmd_vel_pub->publish(twist);
}

void DeadReckoningNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Add Gaussian noise to the odometry readings
    double noisy_x = msg->pose.pose.position.x + noise_distribution_(generator_);
    double noisy_y = msg->pose.pose.position.y + noise_distribution_(generator_);
    double noisy_theta = tf2::getYaw(msg->pose.pose.orientation) + noise_distribution_(generator_);

    // Update state variables with noisy data
    x_ = noisy_x;
    y_ = noisy_y;
    theta_ = noisy_theta;

    // Publish noisy odometry
    auto noisy_odom = nav_msgs::msg::Odometry();
    noisy_odom.header.stamp = this->now();
    noisy_odom.header.frame_id = "odom";
    noisy_odom.pose.pose.position.x = noisy_x;
    noisy_odom.pose.pose.position.y = noisy_y;
    noisy_odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, noisy_theta));
    odom_pub->publish(noisy_odom);
}

void DeadReckoningNode::timer_callback() {
    // Calculate movement direction
    double linear_speed = forward_ ? linear_speed_ : -linear_speed_;
    
    // Move robot based on parameters
    move_robot(linear_speed, angular_speed_);
    
    // Update position using dead reckoning
    double dt = 0.1; // Example time step
    double delta_x = linear_speed * cos(theta_) * dt;
    double delta_y = linear_speed * sin(theta_) * dt;
    x_ += delta_x;
    y_ += delta_y;
    theta_ += angular_speed_ * dt;

    // Check if distance traveled is sufficient
    double distance_traveled = std::sqrt(std::pow(x_ - start_x_, 2) + std::pow(y_ - start_y_, 2));
    if (distance_traveled >= distance_) {
        // Stop the robot
        move_robot(0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Target distance reached.");
    }
}
