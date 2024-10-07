cmake_minimum_required(VERSION 3.8)
project(map_overlay)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

add_executable(map_overlay_node src/map_overlay_node.cpp)
ament_target_dependencies(map_overlay_node rclcpp sensor_msgs cv_bridge OpenCV image_transport)

install(TARGETS
  map_overlay_node
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(image_publisher_node src/image_publisher_node.cpp)
ament_target_dependencies(image_publisher_node rclcpp sensor_msgs cv_bridge OpenCV image_transport)

install(TARGETS
  image_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

