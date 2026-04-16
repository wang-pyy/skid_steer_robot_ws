cmake_minimum_required(VERSION 3.8)
project(robot_odom)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_library(GPIOD_LIBRARY NAMES gpiod)
find_path(GPIOD_INCLUDE_DIR NAMES gpiod.h)

add_executable(odom_node
  src/encoder.cpp
  src/odometry.cpp
  src/odom_node.cpp
)

target_include_directories(odom_node PRIVATE
  include
  ${GPIOD_INCLUDE_DIR}
)

target_link_libraries(odom_node ${GPIOD_LIBRARY})

ament_target_dependencies(odom_node
  rclcpp nav_msgs sensor_msgs std_msgs geometry_msgs tf2 tf2_ros
)

install(TARGETS odom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
