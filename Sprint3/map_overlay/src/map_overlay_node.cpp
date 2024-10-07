#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class MapOverlayNode : public rclcpp::Node {
public:
    MapOverlayNode() : Node("map_overlay") {
        // Subscribers for two map images
        image_sub_1_ = image_transport::create_subscription(
            this, "map1", std::bind(&MapOverlayNode::imageCallback1, this, std::placeholders::_1), "raw");
        image_sub_2_ = image_transport::create_subscription(
            this, "map2", std::bind(&MapOverlayNode::imageCallback2, this, std::placeholders::_1), "raw");
    }

private:
    // Callback functions for the two images
    void imageCallback1(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        try {
            img1_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            if (!img2_.empty()) {
                overlayImages();
            }
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void imageCallback2(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        try {
            img2_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            if (!img1_.empty()) {
                overlayImages();
            }
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    // Overlay two images
    void overlayImages() {
        // Resize the images to the same size if necessary
        cv::Size new_size(img1_.cols * 0.5, img1_.rows * 0.5);
        cv::resize(img1_, img1_, new_size);
        cv::resize(img2_, img2_, new_size);

        // Simple overlay: blending the two images
        cv::Mat overlay;
        double alpha = 0.6;  // Weight for the first image
        double beta = 1.0 - alpha;  // Weight for the second image
        cv::addWeighted(img1_, alpha, img2_, beta, 0.0, overlay);

        // Display the overlayed image in a window
        cv::imshow("Map Overlay", overlay);
        cv::waitKey(1);  // This will keep the OpenCV window updating
    }

    image_transport::Subscriber image_sub_1_;
    image_transport::Subscriber image_sub_2_;
    cv::Mat img1_, img2_;  // Store the images to be overlaid
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOverlayNode>());
    rclcpp::shutdown();
    return 0;
}
