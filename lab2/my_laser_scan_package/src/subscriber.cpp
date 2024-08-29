# include "subscriber.h"


LaserScanSubscriber:: LaserScanSubscriber()
: Node ("subscriber")
{
    // Create a subscriber to listen to /scan topic
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&LaserScanSubscriber::topic_callback, this, std::placeholders::_1));

    //Publish range to be shown in rviz 
   publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_info", 10);


    // Set the target angle range in degrees 
    // min_angle_ = 30.0;
    // max_angle_ = 60.0;
    target_index_ = 100;

}

void LaserScanSubscriber::topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

    // // Convert angles from degrees to radians
    // double min_angle_rad = min_angle_ * (M_PI / 180.0);
    // double max_angle_rad = max_angle_ * (M_PI / 180.0);

    // Create a new LaserScan message to publish    
    auto angle_scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    //Copy header information from the original message
    angle_scan_msg->header = msg->header;
    
    // Set the index filter
    int filter_number= 2; 

    // // Set the angle range for the filtered scan
    // angle_scan_msg->angle_min = min_angle_rad;
    // angle_scan_msg->angle_max = max_angle_rad;
     // Set the angle and range information from the original message
    angle_scan_msg->angle_min = msg->angle_min;
    angle_scan_msg->angle_max = msg->angle_max;

    //Set the angle increment for the filtered scan
    // Ensure that the angle increment is adjusted to account for the filter
    angle_scan_msg->angle_increment = msg->angle_increment * filter_number; 
    
    //Set ranges and insensities vectors 
    angle_scan_msg->ranges.clear();
    angle_scan_msg->intensities.clear();


    // // Set Start Index
    // size_t start_index = static_cast<size_t>((min_angle_rad - msg->angle_min) / msg->angle_increment);
    // //Set end index
    // size_t end_index = static_cast<size_t>((max_angle_rad - msg->angle_min) / msg->angle_increment);

    // // Ensure indices are within bounds
    //  start_index = std::max(start_index, size_t(0));
    // /end_index = std::min(end_index, msg->ranges.size() - 1);


    // // Check if the index is within range
    // if (angle_index < msg->ranges.size())
    // {
    //     double range = msg->ranges[angle_index];
    //     RCLCPP_INFO(this->get_logger(), "Range at %.2f degrees: %.2f meters", target_angle_, range);
    // }
    // else
    // {
    //     RCLCPP_WARN(this->get_logger(), "Angle index out of range.");
    // }

 
    //Read scan in range 
    // for (size_t i = start_index; i <= end_index && i < msg->ranges.size(); ++i)
    // {
    //     angle_scan_msg->ranges.push_back(msg->ranges[i]);
    //     if (!msg->intensities.empty())
    //     {
    //         angle_scan_msg->intensities.push_back(msg->intensities[i]);
    //     }
    // }

    //Read scan at every nth index and populate the new message
    for(size_t i = 0; i < msg->ranges.size(); i += filter_number) {
        angle_scan_msg->ranges.push_back(msg->ranges[i]); 
        if(!msg->intensities.empty()) { 
            angle_scan_msg->intensities.push_back(msg->intensities[i]); 
        }
    }


    //Copy the rest of the message information
    angle_scan_msg->scan_time = msg->scan_time;
    angle_scan_msg->time_increment = msg->time_increment;
    angle_scan_msg->range_min = msg->range_min;
    angle_scan_msg->range_max = msg->range_max;

    //Publish scan info 
    publisher_->publish(*angle_scan_msg);

    // Send given scan index to the console
    if (target_index_ < msg->ranges.size())
    {
        double target_range = msg->ranges[target_index_];
        RCLCPP_INFO(this->get_logger(), "Reading at index %zu: %.2f meters", target_index_, target_range);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Target index %zu out of range.", target_index_);
    }

}