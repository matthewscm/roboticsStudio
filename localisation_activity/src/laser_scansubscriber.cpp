#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/twist.hpp>  // For sending velocity commands

class LaserScanSubscriber : public rclcpp::Node
{
public:
    LaserScanSubscriber() : Node("laser_scan_subscriber")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing LaserScanSubscriber");

        // Subscribe to LaserScan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",  // Update with your actual scan topic if different
            10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { this->listener_callback(msg); }
        );

        // Subscribe to the AMCL pose topic
        amcl_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose",
            10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { this->amcl_callback(msg); }
        );

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        prev_ranges_.clear();
        is_first_scan_ = true;
        scan_saved_ = false;
        rotated_ = false;
        amcl_received_first_ = false;
        amcl_received_second_ = false;
        yaw_completed_ = false;

        RCLCPP_INFO(this->get_logger(), "LaserScanSubscriber initialized");
    }

    ~LaserScanSubscriber()
    {
        RCLCPP_INFO(this->get_logger(), "LaserScanSubscriber is being destroyed");
        // Clean up any resources if needed here
    }

private:
    void listener_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
            std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());

            if (is_first_scan_)
            {
                RCLCPP_INFO(this->get_logger(), "Saving initial scan");
                save_scan_image(ranges, msg->angle_min, msg->angle_max, "initial_scan.png");
                is_first_scan_ = false;
                RCLCPP_INFO(this->get_logger(), "Rotating robot");
                rotate_robot();
            }
            else if (!scan_saved_ && rotated_)
            {
                RCLCPP_INFO(this->get_logger(), "Saving rotated scan");
                save_scan_image(ranges, msg->angle_min, msg->angle_max, "rotated_scan.png");
                scan_saved_ = true;

            }
        if (amcl_received_first_ && amcl_received_second_ && !yaw_completed_)
        {
            calculate_yaw_difference();  // Calculate yaw difference after second AMCL pose received
        }
    }

    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!amcl_received_first_)
        {
            RCLCPP_INFO(this->get_logger(), "First AMCL pose received");
            first_amcl_pose_ = msg->pose.pose;
            amcl_received_first_ = true;
        }
        else if (!amcl_received_second_)
        {
            RCLCPP_INFO(this->get_logger(), "Second AMCL pose received");
            second_amcl_pose_ = msg->pose.pose;
            amcl_received_second_ = true;
        }
    }

    void calculate_yaw_difference()
    {
        RCLCPP_INFO(this->get_logger(), "Calculating yaw difference");

        double yaw_first = calculate_yaw_from_quaternion(first_amcl_pose_.orientation);
        double yaw_second = calculate_yaw_from_quaternion(second_amcl_pose_.orientation);
        double yaw_diff = yaw_second - yaw_first;
        yaw_completed_ = true;

        RCLCPP_INFO(this->get_logger(), "Yaw difference: %f", yaw_diff);
    }

    double calculate_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
    {
        // Calculate yaw from quaternion (manual calculation without "get yaw")
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);  // Returns yaw in radians
    }

    void save_scan_image(const std::vector<float>& ranges, float angle_min, float angle_max, const std::string& filename)
    {
        RCLCPP_INFO(this->get_logger(), "Creating image for scan");
        int num_readings = ranges.size();
        cv::Mat image(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

        float angle_increment = (angle_max - angle_min) / num_readings;
        cv::Scalar color(0, 0, 0);

        for (int i = 0; i < num_readings; ++i)
        {
            float angle = angle_min + i * angle_increment;
            float x = ranges[i] * std::cos(angle);
            float y = ranges[i] * std::sin(angle);

            int x_pixel = static_cast<int>(400 + x * 100);  // Scale and offset
            int y_pixel = static_cast<int>(400 - y * 100);  // Scale and offset

            if (x_pixel >= 0 && x_pixel < image.cols && y_pixel >= 0 && y_pixel < image.rows)
            {
                image.at<cv::Vec3b>(y_pixel, x_pixel) = cv::Vec3b(0, 0, 0);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Displaying image");

        cv::imshow("Laser Scan", image);
        cv::waitKey(30);  // Display image for 30 ms
        cv::imwrite(filename, image);  // Save image to file
    }

    void rotate_robot()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing rotation command");
        auto msg = geometry_msgs::msg::Twist();
        msg.angular.z = 0.5;  // Adjust angular speed as needed
        cmd_vel_publisher_->publish(msg);

        // Sleep or wait for rotation to complete
        rclcpp::sleep_for(std::chrono::seconds(5));  // Adjust time based on speed and desired rotation
        msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(msg);
        rotated_ = true;
        RCLCPP_INFO(this->get_logger(), "Rotation command completed");
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    std::vector<float> prev_ranges_;
    geometry_msgs::msg::Pose first_amcl_pose_, second_amcl_pose_;
    bool is_first_scan_;
    bool scan_saved_;
    bool rotated_;
    bool amcl_received_first_;
    bool amcl_received_second_;
    bool yaw_completed_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanSubscriber>());
    rclcpp::shutdown();
    return 0;
}



