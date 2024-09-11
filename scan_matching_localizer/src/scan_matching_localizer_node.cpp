#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>

class ScanMatchingLocalizer : public rclcpp::Node {
public:
    ScanMatchingLocalizer()
        : Node("scan_matching_localizer"), image_captured_(false), map_received_(false), angle_difference_(0.0) {
        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&ScanMatchingLocalizer::mapCallback, this, std::placeholders::_1));
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanMatchingLocalizer::scanCallback, this, std::placeholders::_1));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        cv::namedWindow("Map Image", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Laser Scan Image", cv::WINDOW_AUTOSIZE);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg) {
        occupancyGridToImage(mapMsg);
        cv::imshow("Map Image", m_MapColImage);
        cv::waitKey(1);
        map_received_ = true;  // Indicate that the map has been received
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!map_received_) {
            RCLCPP_WARN(this->get_logger(), "Map not received yet. Scan matching will not proceed.");
            return;  // Exit if the map has not been received
        }

        cv::Mat scan_image = laserScanToMat(msg);

        if (!image_captured_) {
            first_image_ = scan_image.clone();
            image_captured_ = true;
            cv::imshow("Laser Scan Image", first_image_);
            cv::waitKey(1);
        } else {
            second_image_ = scan_image.clone();
            cv::imshow("Laser Scan Image", second_image_);
            cv::waitKey(1);
            
            calculateYawChange();
            if (std::abs(angle_difference_) < 1.0) {  // Assuming 1 degree tolerance
                RCLCPP_INFO(this->get_logger(), "Localization successful. Angle is close to 0.");
                return;  // End the localization process
            }
            
            rotateRobot();
        }
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        int img_size = 500;
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max) {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
                int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    image.at<uchar>(y, x) = 255;
                }
            }
        }
        return image;
    }

    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid) {
        int grid_data;
        unsigned int row, col, val;

        m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

        for (row = 0; row < grid->info.height; row++) {
            for (col = 0; col < grid->info.width; col++) {
                grid_data = grid->data[row * grid->info.width + col];
                if (grid_data != -1) {
                    val = 255 - (255 * grid_data) / 100;
                    val = (val == 0) ? 255 : 0;
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
                } else {
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
                }
            }
        }

        cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                                                0, 1, 0,
                                                0, 0, 0);
        cv::erode(m_temp_img, m_MapBinImage, kernel);

        m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);
        cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR);
    }

    void rotateRobot() {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = 0.5;  // Rotate at 0.5 radians per second
        cmd_publisher_->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::seconds(2));
        twist_msg.angular.z = 0.0;
        cmd_publisher_->publish(twist_msg);
    }

    void calculateYawChange() {
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(m_MapColImage, first_image_, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Not enough points for affine transformation. Points detected: %zu", srcPoints.size());
            return;
        }

        try {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
                return;
            }

            double angle = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
            angle_difference_ = angle * 180.0 / CV_PI;
            
            RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }
    }

void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                            std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
    if (img1.empty() || img2.empty()) {
        RCLCPP_ERROR(this->get_logger(), "One or both images are empty.");
        return;
    }

    cv::Mat resized_img2;
    cv::resize(img2, resized_img2, img1.size()); // Resize img2 to match img1 size

    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;

    try {
        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(resized_img2, cv::noArray(), keypoints2, descriptors2);
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV error: %s", e.what());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Image 1 size: %d x %d", img1.cols, img1.rows);
    RCLCPP_INFO(this->get_logger(), "Resized Image 2 size: %d x %d", resized_img2.cols, resized_img2.rows);
    RCLCPP_INFO(this->get_logger(), "Number of keypoints in image 1: %zu", keypoints1.size());
    RCLCPP_INFO(this->get_logger(), "Number of keypoints in resized image 2: %zu", keypoints2.size());

    if (keypoints2.empty()) {
        RCLCPP_WARN(this->get_logger(), "No keypoints found in resized image 2.");
        return;
    }

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
        return a.distance < b.distance;
    });

    std::vector<cv::DMatch> goodMatches;
    double min_dist = matches.front().distance;
    for (const auto& match : matches) {
        if (match.distance <= std::max(2 * min_dist, 30.0)) {
            goodMatches.push_back(match);
        }
    }

    srcPoints.clear();
    dstPoints.clear();
    for (const auto& match : goodMatches) {
        srcPoints.push_back(keypoints1[match.queryIdx].pt);
        dstPoints.push_back(keypoints2[match.trainIdx].pt);
    }

    RCLCPP_INFO(this->get_logger(), "Number of good matches: %zu", goodMatches.size());

    if (srcPoints.empty()) {
        RCLCPP_WARN(this->get_logger(), "No good matches found.");
    }

    cv::Mat img_matches;
    cv::drawMatches(img1, keypoints1, resized_img2, keypoints2, goodMatches, img_matches);
    cv::imshow("Matches", img_matches);
    cv::waitKey(1);
}



    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    cv::Mat m_temp_img;
    cv::Mat m_MapBinImage;
    cv::Mat m_MapColImage;
    cv::Mat first_image_, second_image_;
    bool image_captured_;
    bool map_received_;
    double angle_difference_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMatchingLocalizer>());
    rclcpp::shutdown();
    return 0;
}

