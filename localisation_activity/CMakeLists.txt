cmake_minimum_required(VERSION 3.8)
project(localisation_activity)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)  
find_package(tf2 REQUIRED)  
find_package(tf2_geometry_msgs REQUIRED)  


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Declare the executable
add_executable(laser_scan_subscriber src/laser_scansubscriber.cpp)

# Add dependencies
ament_target_dependencies(laser_scan_subscriber
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  geometry_msgs  
  tf2
  tf2_geometry_msgs
)

# Install
install(TARGETS
  laser_scan_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
