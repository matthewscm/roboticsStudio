#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class ImagePublisherNode : public rclcpp::Node {
public:
    ImagePublisherNode(const std::string &image_path, const std::string &topic_name)
        : Node("image_publisher"), image_path_(image_path) {

        // Create an image publisher
        image_pub_ = image_transport::create_publisher(this, topic_name);

        // Load the PGM image from file as a grayscale image
        image_ = cv::imread(image_path_, cv::IMREAD_GRAYSCALE);
        if (image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", image_path_.c_str());
            return;
        }

        // Create a timer to periodically publish the image
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&ImagePublisherNode::publishImage, this));
    }

private:
    void publishImage() {
        if (!image_.empty()) {
            // Convert OpenCV image to ROS2 image message
            std::shared_ptr<sensor_msgs::msg::Image> msg =
                cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image_).toImageMsg();
            image_pub_.publish(*msg);
            RCLCPP_INFO(this->get_logger(), "PGM image published on topic.");
        }
    }

    image_transport::Publisher image_pub_;
    cv::Mat image_;
    std::string image_path_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: image_publisher_node <image_path> <topic_name>" << std::endl;
        return -1;
    }

    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create the publisher node
    rclcpp::spin(std::make_shared<ImagePublisherNode>(argv[1], argv[2]));

    rclcpp::shutdown();
    return 0;
}
