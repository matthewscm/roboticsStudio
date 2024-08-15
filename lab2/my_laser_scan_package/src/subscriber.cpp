# include "subscriber.h"


LaserScanSubscriber:: LaserScanSubscriber()
: Node ("subscriber")
{
    // Create a subscriber to listen to /scan topic
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&LaserScanSubscriber::topic_callback, this, std::placeholders::_1));

    //Publish imformation from the scan
   // publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_info", 10);


    // Set the angle in degrees 
    target_angle_ = 40.0; 

}

void LaserScanSubscriber::topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

    // Convert target angle from degrees to radians
    double target_angle_rad = target_angle_ * (M_PI / 180.0);

    // Find the index of the target angle
    size_t angle_index = static_cast<size_t>((target_angle_rad - msg->angle_min) / msg->angle_increment);

    // Check if the index is within range
    if (angle_index < msg->ranges.size())
    {
        double range = msg->ranges[angle_index];
        RCLCPP_INFO(this->get_logger(), "Range at %.2f degrees: %.2f meters", target_angle_, range);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Angle index out of range.");
    }


}