#include "dead_reckoning.h"

// Constructor
DeadReckoningNode::DeadReckoningNode() : Node("dead_reckoning") {
    // Initialize state variables
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    start_x_ = 0.0;
    start_y_ = 0.0;
    distance_ = 10.0;
    forward_ = true;
    ending_ = false;

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
    estimated_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("estimated_odom", 10);


    // Initialize subscribers
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&DeadReckoningNode::odom_callback, this, std::placeholders::_1)
    );


    // Initialize timer
    auto timer_callback = std::bind(&DeadReckoningNode::timer_callback, this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);

    
}

// Destructor
DeadReckoningNode::~DeadReckoningNode() {
    // Log before publishing stop command
    RCLCPP_INFO(this->get_logger(), "Node destructor called, issuing stop command.");
    
    // auto twist = geometry_msgs::msg::Twist();
    // twist.linear.x = 0.0;
    // twist.angular.z = 0.0;
    // cmd_vel_pub->publish(twist);

    // Check if publisher is valid before publishing
    ending_ = true;
    if (cmd_vel_pub) {
        // Publish stop command
        RCLCPP_INFO(this->get_logger(), "Stop command ");
        move_robot(0.0, 0.0);
    }

    // Stop the timer
    if (timer_) {
        timer_->cancel();
    }


    // Allow some time for the stop command to be processed
    std::this_thread::sleep_for(std::chrono::seconds(2)); 

    // Log after stop command
    RCLCPP_INFO(this->get_logger(), "Stop command issued, allowing time for processing.");
}


void DeadReckoningNode::move_robot(double linear_speed, double angular_speed) {
    // Create a twist message
    
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = forward_ ? linear_speed : -linear_speed;
    twist.angular.z = angular_speed;

    if (ending_) {
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
    }

    //RCLCPP_INFO(this->get_logger(), "Publishing Twist: linear.x=%f, angular.z=%f", twist.linear.x, twist.angular.z);

    // Publish the twist message
    cmd_vel_pub->publish(twist);
}


void DeadReckoningNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Store the initial odometry values
    if (start_x_ == 0.0 && start_y_ == 0.0) {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        theta_ = std::atan2(
            2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                    msg->pose.pose.orientation.x * msg->pose.pose.orientation.y),
            1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                        msg->pose.pose.orientation.z * msg->pose.pose.orientation.z)
        );
        start_x_ = x_;
        start_y_ = y_;
    }
}


void DeadReckoningNode::update_odometry() {
    // Update position based on velocity
    double delta_x = linear_speed_ * std::cos(theta_) * 0.1; // Assuming 100 ms timer
    double delta_y = linear_speed_ * std::sin(theta_) * 0.1;
    double delta_theta = angular_speed_ * 0.1;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // Create a quaternion for the orientation
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta_); // Set orientation using Roll-Pitch-Yaw angles

    // Convert to geometry_msgs::msg::Quaternion manually
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg.x = quat.x();
    quat_msg.y = quat.y();
    quat_msg.z = quat.z();
    quat_msg.w = quat.w();

    // Create an estimated odometry message
    nav_msgs::msg::Odometry estimated_odom_msg;
    estimated_odom_msg.header.stamp = this->now();
    estimated_odom_msg.header.frame_id = "odom";
    estimated_odom_msg.pose.pose.position.x = x_;
    estimated_odom_msg.pose.pose.position.y = y_;
    estimated_odom_msg.pose.pose.orientation = quat_msg;
    estimated_odom_msg.twist.twist.linear.x = linear_speed_;
    estimated_odom_msg.twist.twist.angular.z = angular_speed_;

    // Publish the estimated odometry
    estimated_odom_pub->publish(estimated_odom_msg);
}


void DeadReckoningNode::timer_callback() {
    // Move the robot
    move_robot(linear_speed_, angular_speed_);
    update_odometry();

    // Check if the robot has traveled the required distance
    double current_distance = std::sqrt(std::pow(x_ - start_x_, 2) + std::pow(y_ - start_y_, 2));
    // if (current_distance >= distance_) {
    //     // Stop the robot
    //     move_robot(0.0, 0.0);

    //     // Stop the timer
    //     timer_->cancel();
    // }
}

bool DeadReckoningNode::has_reached_distance(double tolerance) {
    // Check if the robot has traveled the desired distance
    double current_distance = std::sqrt(std::pow(x_ - start_x_, 2) + std::pow(y_ - start_y_, 2));
    return current_distance >= (distance_ - tolerance);
}
