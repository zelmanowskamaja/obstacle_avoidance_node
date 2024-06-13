cmake_minimum_required(VERSION 3.14)
project(obstacle_avoidance_node)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(autoware_cmake REQUIRED)
find_package(OpenCV REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(obstacle_avoidance_node SHARED 
  src/obstacle_avoidance.cpp
  src/obstacle_detection.cpp
)
ament_target_dependencies(obstacle_avoidance_node
  rclcpp
  autoware_auto_planning_msgs
  sensor_msgs
  std_msgs
  rclcpp_components
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  nav_msgs
  OpenCV
)

# If ObstacleDetection is a node
add_executable(obstacle_detection_node src/obstacle_detection.cpp)
ament_target_dependencies(obstacle_detection_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  OpenCV
  tf2
  tf2_ros
)
target_link_libraries(obstacle_detection_node
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_nodes(obstacle_avoidance_node "obstacle_avoidance::ObstacleAvoidance")
# rclcpp_components_register_nodes(obstacle_detection_node "ObstacleDetection")


# Installation of node and other files
install(TARGETS
  obstacle_avoidance_node
  obstacle_detection_node
  DESTINATION lib
)

install(PROGRAMS
  scripts/obstacle_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies and package
ament_export_libraries(obstacle_avoidance_node)
ament_package()
