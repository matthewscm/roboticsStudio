#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>

class CylinderNode : public rclcpp::Node {
public:
    CylinderNode() : Node("cylinder_node") {
        // Create a subscriber to the Laser scan topic
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderNode::laser_callback, this, std::placeholders::_1));

        pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&CylinderNode::odom_callback, this, std::placeholders::_1));

        
        // Create a publisher for markers
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("cylinder_marker", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    
    nav_msgs::msg::Odometry robot_pose_; // To store the robot's pose
    std::vector<geometry_msgs::msg::Point> points_in_segment;

    void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
        robot_pose_ = *msg; // Update the robot's pose
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        const float diameter = 0.30;  // Cylinder diameter in meters
        const float radius = diameter / 2.0; // Half diameter
        const float threshold = 0.05; // Tolerance for detection (5 cm)

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
// adding ocmment to test git
                detected_points.push_back(point);
            }
        }

        // Check for cylinder-like structure in detected points
        if (detect_cylinder(detected_points, diameter, threshold)) {
            RCLCPP_INFO(this->get_logger(), "Cylinder detected!");
            publish_marker(points_in_segment);  // Publish marker at the detected points
        } else {
            RCLCPP_INFO(this->get_logger(), "No cylinder detected.");
            delete_marker();  // Delete the marker
        }
        rclcpp::sleep_for(std::chrono::milliseconds(300));
    }

    bool detect_cylinder(const std::vector<geometry_msgs::msg::Point>& points, float diameter, float threshold) {
        geometry_msgs::msg::Point previous_point = points[0];
        double distance = 0;
        for (const auto& point : points) {
            distance = std::sqrt((point.x - previous_point.x) * (point.x - previous_point.x) + 
                                 (point.y - previous_point.y) * (point.y - previous_point.y));
            if (distance > 0.4) {
                previous_point = point;
                points_in_segment.clear();
                distance = 0;
                continue; // Skip to the next point
            } 
            if (distance < diameter + threshold && distance > diameter - threshold) {
                    points_in_segment.push_back(point);
                    return true;
                }
            else{
                points_in_segment.clear();
            }
            previous_point = point;
        }
        return false;
    }

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

        double x_position = robot_pose_.pose.pose.position.x;  // Correctly access x
        double y_position = robot_pose_.pose.pose.position.y;  // Correctly access y
        // Transform the centroid to the map frame
        float map_x = centroid_x + x_position;
        float map_y = centroid_y + y_position;

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
        marker_publisher_->publish(marker);
    }

        void delete_marker() {
        // // Create a marker with action DELETE
        // visualization_msgs::msg::Marker marker;
        // marker.header.frame_id = "map";  // Set to the correct frame
        // marker.header.stamp = this->now();
        // marker.ns = "cylinder_detection";
        // marker.id = 1;
        // marker.action = visualization_msgs::msg::Marker::DELETE;  // Set action to DELETE

        // // Publish the deletion marker
        // marker_publisher_->publish(marker);
    }

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylinderNode>());
    rclcpp::shutdown();
    return 0;
}
