#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>

/**
    * @brief A class to detect a cylinder from a set of points
 */
class CylinderNode : public rclcpp::Node {
public:

    /**
    * @brief Construct a new Cylinder Node object
    */
    CylinderNode() : Node("cylinder_node") {
        // Create a subscriber for the LaserScan message
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderNode::laser_callback, this, std::placeholders::_1));
        // Create a subscriber for the Odometry message
        pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&CylinderNode::odom_callback, this, std::placeholders::_1));
        // Create a publisher for the Marker message
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("cylinder_marker", 10);
    }

private:
    // Create the subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    // Create the variables
    nav_msgs::msg::Odometry robot_pose_; // To store the robot's pose
    std::vector<geometry_msgs::msg::Point> points_in_segment; // To store the points in the segment
    int read_counter = 0;

    /**
    * @brief Callback function for the Odometry message sent by dead reckoning
    * @param msg The Odometry message
    */
    void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
        robot_pose_ = *msg; // Update the robot's pose
    }

    /**
    * @brief Callback function for the LaserScan message
    * @param msg The LaserScan message
    */
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Constants
        const float diameter = 0.3;  // Cylinder diameter in meters
        const float radius = diameter / 2.0; // Half diameter
        const float circumference = M_PI * diameter; // Circumference of the cylinder
        const float half_circumference = circumference / 2.0; // Half circumference
        const float threshold = 0.15; // Tolerance for detection (5 cm)
        std::vector<geometry_msgs::msg::Point> detected_points;

        // Process the Lidar data to find points that could form a cylinder
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            // Check if the scan value is valid
            if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min) {
                // Convert polar coordinates to Cartesian
                float angle = msg->angle_min + i * msg->angle_increment;
                float x = msg->ranges[i] * std::cos(angle);
                float y = msg->ranges[i] * std::sin(angle);
                // Store the detected point
                geometry_msgs::msg::Point point;
                point.x = x;
                point.y = y;
                point.z = 0.0;  // Assuming 2D plane
                detected_points.push_back(point);
            }
        }
        // Check for cylinder-like structure in detected points
        if (detect_cylinder(detected_points, half_circumference, threshold)) {
            read_counter++;
            //RCLCPP_INFO(this->get_logger(), "Cylinder Count");
            // Filter out mistakes/misreads
            if (read_counter == 8){
                RCLCPP_INFO(this->get_logger(), "Cylinder detected!");
                publish_marker(points_in_segment);  // Publish marker at the detected points
                read_counter = 0;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "No cylinder detected.");
            delete_marker();  // Delete the marker
            read_counter = 0;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(30));
    }

    /**
    * @brief Detects a cylinder from a set of points
    * @param points The set of points
    * @param diameter The diameter of the cylinder
    * @param threshold The tolerance for detection
    * @return true if a cylinder is detected, false otherwise
    */
    bool detect_cylinder(const std::vector<geometry_msgs::msg::Point>& points, float diameter, float threshold) {
        if (points.empty()) {
            return false; // Handle empty input
        }

        geometry_msgs::msg::Point previous_point = points[0];
        double total_distance = 0.0;
        bool isInSegment = false;

        for (size_t i = 1; i < points.size(); ++i) {
            const auto& point = points[i];
            double dx = point.x - previous_point.x;
            double dy = point.y - previous_point.y;
            double distance_sq = dx * dx + dy * dy; // Use squared distance to avoid sqrt

            // If readings are close together, assume they are from the same object
            if (distance_sq < 0.1) { 
                if (!isInSegment) {
                    isInSegment = true;
                    // Clear the points in the segment
                    total_distance = 0.0;
                }
                total_distance += std::sqrt(distance_sq); // Sum distance of the segment
                points_in_segment.push_back(point); // Store the point
            } else {
                if (isInSegment) {
                    // Check if the segment is a cylinder
                    if (total_distance < diameter + (threshold*0.7) && total_distance > diameter - threshold) {
                        points_in_segment.push_back(point);
                        return true; // Found a cylinder
                    } else {
                         // Clear the points if not a cylinder
                        points_in_segment.clear();
                        total_distance = 0.0;
                    }
                }
                // Reset the segment
                isInSegment = false;
            }
            previous_point = point; // Update previous_point unconditionally
        }
        return false; // No cylinder detected
    }

    /**
    * @brief Publishes a marker at the centroid of the detected points
    * @param points The detected points
    */
    void publish_marker(const std::vector<geometry_msgs::msg::Point>& points) {
    if (points.empty()) {
        return; // No points, no marker to publish
    }

    // Calculate the centroid of the points
    float sum_x = 0.0, sum_y = 0.0;
    for (const auto& point : points) {
        sum_x += point.x;
        sum_y += point.y;
    }

    float centroid_x = sum_x / points.size();
    float centroid_y = sum_y / points.size();

    // Get the robot's position
    double x_position = robot_pose_.pose.pose.position.x;  
    double y_position = robot_pose_.pose.pose.position.y;  

    // Extract the quaternion from robot_pose_
    double qx = robot_pose_.pose.pose.orientation.x;
    double qy = robot_pose_.pose.pose.orientation.y;
    double qz = robot_pose_.pose.pose.orientation.z;
    double qw = robot_pose_.pose.pose.orientation.w;

    // Calculate the yaw angle from the quaternion
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double yaw = atan2(siny_cosp, cosy_cosp); // Yaw angle in radians

    // Transform the centroid to the robot frame
    // Rotation transformation
    float map_x = x_position + (centroid_x * cos(yaw) - centroid_y * sin(yaw));
    float map_y = y_position + (centroid_x * sin(yaw) + centroid_y * cos(yaw));

        // Create a marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // Set to the correct frame
        marker.header.stamp = this->now();
        marker.ns = "cylinder_detection";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position of the marker (use the transformed position)
        marker.pose.position.x = map_x;  
        marker.pose.position.y = map_y;  
        marker.pose.position.z = 0.0;  // Assume it's on the ground
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.3;  // Diameter of the cylinder
        marker.scale.y = 0.3;  // Diameter of the cylinder
        marker.scale.z = 0.05;   // Height of the cylinder

        marker.color.r = 1.0;  // Red color
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;        

        // Publish the marker
        RCLCPP_INFO(this->get_logger(), "Publishing marker");
        marker_publisher_->publish(marker);
    }

    /**
    * @brief Deletes the marker
    */
    void delete_marker() {
    // Create a marker with action DELETE
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";  // Set to the correct frame
    marker.header.stamp = this->now();
    marker.ns = "cylinder_detection";
    marker.id = 1;
    marker.action = visualization_msgs::msg::Marker::DELETE;  // Set action to DELETE

    // Publish the deletion marker
    marker_publisher_->publish(marker);
    }

};

/**
    * @brief Main function to run the node
    * @param argc Number of arguments
    * @param argv The arguments
    * @return int The exit status
 */

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylinderNode>());
    rclcpp::shutdown();
    return 0;
}
