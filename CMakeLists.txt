cmake_minimum_required(VERSION 3.8)
project(quadcopter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)

# Create cpp executables 
add_executable(control_position_node src/control_position.cpp)
ament_target_dependencies(control_position_node rclcpp px4_msgs)

# Install cpp executables 

install(TARGETS
  control_position_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()