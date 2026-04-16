cmake_minimum_required(VERSION 3.8)
project(robot_bno055)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(bno055_node
  src/bno055_driver.cpp
  src/bno055_node.cpp
)

target_include_directories(bno055_node PRIVATE include)

ament_target_dependencies(bno055_node
  rclcpp sensor_msgs std_msgs
)

install(TARGETS bno055_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
