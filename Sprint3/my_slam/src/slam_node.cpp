#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>

class SLAM: public rclcpp::Node {
public:
    SLAM()
        : Node("slam_node"), image_captured_(false), map_received_(false), angle_difference_(0.0) {
        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&SLAM::mapCallback, this, std::placeholders::_1));
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SLAM::scanCallback, this, std::placeholders::_1));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        cv::namedWindow("Map Image", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Laser Scan Image", cv::WINDOW_AUTOSIZE);

        robot_rotated_ = true;
    }

private:
 
        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        std::cout << "mapCallback" << std::endl;

        occupancyGridToImage(mapMsg);

        cv::Mat tmp_col_img = m_MapColImage.clone();

        cv::rotate(tmp_col_img, tmp_col_img, cv::ROTATE_90_COUNTERCLOCKWISE);
        std::cout << "VISUALISE MAP" << std::endl;
        cv::imshow("Map Image", tmp_col_img);
        cv::waitKey(1);
        map_received_ = true;
        rclcpp::sleep_for(std::chrono::seconds(15)); 
    }


      void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!map_received_) {
            RCLCPP_WARN(this->get_logger(), "Map not received yet. Scan matching will not proceed.");
            return;  // Exit if the map has not been received
        }
        rclcpp::sleep_for(std::chrono::seconds(2));  // Sleep for 200 milliseconds
        // Check if the robot has rotated before saving the scan image
        if (robot_rotated_) {
            cv::Mat scan_image = laserScanToMat(msg);

            // Always replace the previous scan image with the new one
            first_image_ = scan_image.clone();
            cv::imshow("Laser Scan Image", first_image_);
            cv::waitKey(1);  // Display the updated image

            // Reset the flag after saving the image
            robot_rotated_ = false;
        }

        // Proceed to calculate the yaw change and check the difference
        calculateYawChange();
        if (std::abs(angle_difference_) < 1.0) {  // 1 degree tolerance
            RCLCPP_INFO(this->get_logger(), "Localization successful. Angle is close to 0.");
            //return;  // End the localization process
        }

        rotateRobot();  // Rotate the robot based on the yaw calculation
    }


    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        int img_size = 2001;
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


    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
        {
            int grid_data; //Store grid cell data
            unsigned int row, col, val;
            // Create a temporary image to store the occupancy grid data
            m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);
            // Create a binary image to store the binary version of the occupancy grid
            std::cout << "DataParse started for map: " << grid->header.stamp.sec << " Dim: " << grid->info.height << "x" << grid->info.width << std::endl;
            // Parse the occupancy grid data and store it in the temporary image
            for (row = 0; row < grid->info.height; row++) {
                for (col = 0; col < grid->info.width; col++) {
                    // Get the grid data
                    grid_data = grid->data[row * grid->info.width + col];
                    // Convert the grid data to a grayscale value
                    if (grid_data != -1) {
                        // Invert the grayscale value
                        val = 255 - (255 * grid_data) / 100;
                        // Convert the grayscale value to a binary value
                        val = (val == 0) ? 255 : 0;
                        // Store the binary value in the temporary image
                        m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
                    } else {
                        // Store the unknown value in the temporary image
                        m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
                    }
                }
            }
            // Store the map scale, origin, and size
            map_scale_ = grid->info.resolution;
            origin_x = grid->info.origin.position.x;
            origin_y = grid->info.origin.position.y;
            size_x = grid->info.width;
            size_y = grid->info.height;
            // Convert the temporary image to a binary image
            cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                                        0, 1, 0,
                                        0, 0, 0);
            cv::erode(m_temp_img, m_MapBinImage, kernel);
            // Convert the binary image to a color image
            m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);
            cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR);
            std::cout << "Occupancy grid map converted to a binary image\n";

            // Display the image to verify
            cv::imshow("Occupancy Grid", m_MapColImage);
            cv::waitKey(1);
        }


    void rotateRobot() {
        const double rotation_speed = 0.2;  // radians per second
         const double threshold = 1;      // Small angle difference threshold (degrees)
        // Convert angle_difference from degrees to radians
        double angle_difference_adjusted = angle_difference_ ;
        double angle_difference_radians = angle_difference_adjusted* M_PI / 180.0;

        // Check if the angle difference is below the threshold
        if (std::abs(angle_difference_) < threshold) {
            RCLCPP_INFO(this->get_logger(), "Angle difference is too small to rotate. No movement.");
            //return;
        }

        double duration_seconds = std::abs(angle_difference_radians) / rotation_speed;
        int duration_milliseconds = duration_seconds * 1000;
         // Print the duration in seconds to the terminal
        RCLCPP_INFO(this->get_logger(), "Rotation duration: %.2f seconds", duration_seconds);

        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = rotation_speed * (angle_difference_adjusted > 0 ? 1.0 : -1.0);  // Rotate in the correct direction
        cmd_publisher_->publish(twist_msg);

        // Sleep for the calculated duration
        rclcpp::sleep_for(std::chrono::milliseconds(duration_milliseconds));

        // Stop the rotation
        twist_msg.angular.z = 0.0;
        cmd_publisher_->publish(twist_msg);

        robot_rotated_ = true;
    }



    void calculateYawChange() {
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(m_MapColImage, first_image_, srcPoints, dstPoints);
    // Check if enough points were detected for the transformation
        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Not enough points for affine transformation. Points detected: %zu", srcPoints.size());
            return;
        }
        // Estimate the affine transformation matrix
        try {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            // Check if the transformation matrix is empty
            if (transform_matrix.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
                return;
            }
            // Calculate the angle difference from the transformation matrix
            double angle = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
            angle_difference_ = angle * 180.0 / CV_PI;
            // Print the estimated angle difference to the terminal
            RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }
    }


    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
        // Create ORB feature detector and descriptor
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;
        

        // Detect and compute features in both images   
        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);
        // Create a BFMatcher object
        // std::cout << "srcPoints size: " << srcPoints.size() << ", dstPoints size: " << dstPoints.size() << std::endl;
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        // std::cout << "srcPoints size: " << srcPoints.size() << ", dstPoints size: " << dstPoints.size() << std::endl;
        std::vector<cv::DMatch> matches;
        // std::cout << "srcPoints size: " << srcPoints.size() << ", dstPoints size: " << dstPoints.size() << std::endl;
        // Match the descriptors
        matcher.match(descriptors1, descriptors2, matches);
         
        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Determine the number of top matches to keep (RANSAC)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        // Keep only the best matches (top 30%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);
        // Store the matched points
        for (const auto& match : matches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
        
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
    bool robot_rotated_;
    double map_scale_;
    double origin_x;
    double origin_y;
    unsigned int size_x;
    unsigned int size_y;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SLAM>());
    rclcpp::shutdown();
    return 0;
}
