/**
 * @mainpage My Cylinder Detection Project
 * 
 * @section intro_sec Introduction
 * This project demonstrates a ROS2 node for detecting cylinders using LaserScan data. It processes the LIDAR data to find and mark cylindrical objects in a 2D plane and publishes a marker in RViz. The node subscribes to the LaserScan topic and the Dead Reckoning Odometry topic to get the robot's pose. 
 * It then processes the LIDAR data and detected cylinders then applies the transformation to the marker. The node also provides a service to delete the marker. 
 *
 * @section run_sec Running the node
 * To run the node, use the following command:
    * @code
    * ros2 launch turtlebot3_gazebo team_world_launch.launch.py
    * ros2 run turtlebot3_state_estimation  dead_reckoning_node
    * ros2 run scan_matching_localizer scan_matching_localizer_node
    * ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/ros2_ws/src/roboticsStudio/Sprint3/map3.yaml
    * ros2 run cylinder_detector cylinder_node
    * @endcode
 * 
  * @section functions_sec Functions
    * The node has the following functions:
    * - odom_callback: Callback function for the Odometry message sent by dead reckoning
    * - laser_callback: Callback function for the LaserScan message
    * - detect_cylinder: Detects a cylinder-like structure in the detected points
    * - publish_marker: Publishes a marker at the centroid of the detected points
    * - delete_marker: Deletes the marker
    * - main: Main function to run the node
 * @subsection subscribe_sec Subscribed Topics
    * - /scan (sensor_msgs::msg::LaserScan): The LaserScan data
    * - /estimated_odom (nav_msgs::msg::Odometry): The Dead Reckoning Odometry data
    * @subsection publish_sec Published Topics
    * - /visualization_marker (visualization_msgs::msg::Marker): The marker for the detected cylinder
    * @subsection param_sec Parameters
    * - diameter: The diameter of the cylinder
    * - radius: The radius of the cylinder
    * - threshold: The tolerance for detection
    * - detected_points: The detected points
    * - read_counter: The counter for the number of reads
    * - robot_pose_: The robot's pose
    * - points_in_segment: The points in the segment
    * @section output_sec Output
    * @subsection terminal_sec Terminal Output
    * The node outputs the following messages to the terminal:
    * - Cylinder detected! when a cylinder is detected
    * - No cylinder detected. when no cylinder is detected
    * @subsection rviz_sec RViz Output
    * The node publishes a marker in RViz when a cylinder is detected.
    * @image html rviz.png "Rviz with the red marker" @image html gazebo.png "Gazebo with the cylinder"
    * 
 
 *
 * @section author_sec Author
 * Claire Matthews
 */
