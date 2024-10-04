#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>

class CylinderDetector : public rclcpp::Node {
public:
    CylinderDetector() : Node("cylinder_node") {
        // Create a subscriber to the LaserScan topic
        laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&CylinderDetector::laserScanCallback, this, std::placeholders::_1));

        // Create a subscriber to the robot's pose
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "robot_pose", 10, std::bind(&CylinderDetector::poseCallback, this, std::placeholders::_1));
    }

private:
    geometry_msgs::msg::Pose current_pose_;

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        // Update the current pose of the robot
        current_pose_ = *msg;
    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Log that the detector is looking for a cylinder
        RCLCPP_INFO(this->get_logger(), "Looking for cylinder...");

        const double object_diameter = 0.30; // Diameter of the cylinder in meters
        const double tolerance = 0.05; // Tolerance for detection
        const double LENGTH_THRESHOLD = 0.3; // Minimum length to consider for a cylinder segment

        bool inSegment = false;
        double segmentLength = 0.0;
        double segmentRange = 0.0;
        double prevRange = 0.0; 
        unsigned int readings = 0;
        unsigned int indexCount = 0;

        for (unsigned int i = 0; i < msg->ranges.size(); ++i) {
            // Extract range and angle for the current scan
            double range = msg->ranges[i];
            double angle = msg->angle_min + i * msg->angle_increment;

            // Skip invalid ranges (e.g., infinite or NaN)
            if (std::isinf(range) || std::isnan(range)) {
                continue;
            }

            // Calculate distance between the current and previous points
            double distance = std::abs(range - prevRange);

            // Check if scans are from the same "object" by evaluating the distance threshold
            if (distance < 0.4) { // You can adjust this threshold
                if (!inSegment) {
                    inSegment = true;
                    segmentLength = 0.0;
                    segmentRange = 0.0;
                    readings = 0;
                    indexCount = 0; // Reset indexCount at the start of a new segment
                }
                // Increment segment size and other properties
                segmentLength += distance; // Consider the direct distance, not projected
                segmentRange += range;
                readings++;
                indexCount += i; // Keep track of indices for average angle calculation
            } else {
                // Finalize the segment once distance exceeds threshold
                if (inSegment) {
                    double average_range = segmentRange / readings;
                    double cylinder_radius = object_diameter / 2;

                    // Check if the segment qualifies as the cylinder based on length and range
                    if (segmentLength < LENGTH_THRESHOLD ) {
                        double segment_angle = msg->angle_min + (indexCount / readings) * msg->angle_increment;
                        double x_local = average_range * std::cos(segment_angle);
                        double y_local = average_range * std::sin(segment_angle);

                        // Transform local coordinates to global coordinates
                        double x_global = current_pose_.position.x + x_local * std::cos(current_pose_.orientation.z) - y_local * std::sin(current_pose_.orientation.z);
                        double y_global = current_pose_.position.y + x_local * std::sin(current_pose_.orientation.z) + y_local * std::cos(current_pose_.orientation.z);
                        
                        RCLCPP_INFO(this->get_logger(), "Cylinder detected at global location: (%.2f, %.2f)", x_global, y_global);
                        return; // Exit once detected, since only one cylinder is considered
                    }
                    inSegment = false; // Reset for next potential segment
                }
            }
            prevRange = range; // Update previous range for the next iteration
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylinderDetector>());
    rclcpp::shutdown();
    return 0;
}


// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <cmath>

// class CylinderDetector : public rclcpp::Node {
// public:
//     CylinderDetector() : Node("cylinder_node") {
//         // Create a subscriber to the LaserScan topic
//         laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "scan", 10, std::bind(&CylinderDetector::laserScanCallback, this, std::placeholders::_1));
//     }

// private:
//     void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
//         // Log that the detector is looking for a cylinder
//         RCLCPP_INFO(this->get_logger(), "Looking for cylinder...");

//         const double object_diameter = 0.30; // Diameter of the cylinder in meters
//         const double tolerance = 0.05; // Tolerance for detection
//         const double LENGTH_THRESHOLD = 0.3; // Minimum length to consider for a cylinder segment

//         bool inSegment = false;
//         double segmentLength = 0.0;
//         double segmentRange = 0.0;
//         double prevRange = 0.0; 
//         unsigned int readings = 0;
//         unsigned int indexCount = 0;

//         for (unsigned int i = 0; i < msg->ranges.size(); ++i) {
//             // Extract range and angle for the current scan
//             double range = msg->ranges[i];
//             double angle = msg->angle_min + i * msg->angle_increment;

//             // Skip invalid ranges (e.g., infinite or NaN)
//             if (std::isinf(range) || std::isnan(range)) {
//                 continue;
//             }

//             // Calculate distance between the current and previous points
//             double distance = std::abs(range - prevRange);

//             // Check if scans are from the same "object" by evaluating the distance threshold
//             if (distance < 0.4) { // You can adjust this threshold
//                 if (!inSegment) {
//                     inSegment = true;
//                     segmentLength = 0.0;
//                     segmentRange = 0.0;
//                     readings = 0;
//                     indexCount = 0; // Reset indexCount at the start of a new segment
//                 }
//                 // Increment segment size and other properties
//                 segmentLength += distance; // Consider the direct distance, not projected
//                 segmentRange += range;
//                 readings++;
//                 indexCount += i; // Keep track of indices for average angle calculation
//             } else {
//                 // Finalize the segment once distance exceeds threshold
//                 if (inSegment) {
//                     double average_range = segmentRange / readings;
//                     double cylinder_radius = object_diameter / 2;

//                     // Check if the segment qualifies as the cylinder based on length and range
//                     if (segmentLength < LENGTH_THRESHOLD ) {
//                         double segment_angle = msg->angle_min + (indexCount / readings) * msg->angle_increment;
//                         double x = average_range * std::cos(segment_angle);
//                         double y = average_range * std::sin(segment_angle);
                        
//                         RCLCPP_INFO(this->get_logger(), "Cylinder detected at: (%.2f, %.2f)", x, y);
//                         return; // Exit once detected, since only one cylinder is considered
//                     }
//                     inSegment = false; // Reset for next potential segment
//                 }
//             }
//             prevRange = range; // Update previous range for the next iteration
//         }
//     }

//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<CylinderDetector>());
//     rclcpp::shutdown();
//     return 0;
// }

