cmake_minimum_required(VERSION 3.8)
project(scan_matching_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable
add_executable(scan_matching_localizer_node src/scan_matching_localizer_node.cpp)

# Link libraries
ament_target_dependencies(scan_matching_localizer_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  OpenCV
)

# Install targets
install(TARGETS
  scan_matching_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any (commented out if not used)
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/launch
# )

ament_package()

