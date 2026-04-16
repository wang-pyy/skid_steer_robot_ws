cmake_minimum_required(VERSION 3.8)
project(robot_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find libgpiod
find_library(GPIOD_LIBRARY NAMES gpiod)
find_path(GPIOD_INCLUDE_DIR NAMES gpiod.h)

add_executable(driver_node
  src/motor.cpp
  src/pid_controller.cpp
  src/base_controller.cpp
  src/driver_node.cpp
)

target_include_directories(driver_node PRIVATE
  include
  ${GPIOD_INCLUDE_DIR}
)

target_link_libraries(driver_node ${GPIOD_LIBRARY})

ament_target_dependencies(driver_node
  rclcpp
  geometry_msgs
  std_msgs
)

install(TARGETS driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
