cmake_minimum_required(VERSION 3.8)
project(cylinder_detector)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# Create executable
add_executable(cylinder_node src/cylinder_node.cpp)

# Link libraries
ament_target_dependencies(cylinder_node rclcpp sensor_msgs)

# Install executable
install(TARGETS
  cylinder_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

