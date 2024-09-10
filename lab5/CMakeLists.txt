cmake_minimum_required(VERSION 3.8)
project(lab5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(scan_to_image_node src/node.cpp)

ament_target_dependencies(scan_to_image_node
  rclcpp
  sensor_msgs
  geometry_msgs
  OpenCV
)

install(TARGETS
  scan_to_image_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


