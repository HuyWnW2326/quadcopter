cmake_minimum_required(VERSION 3.5)
project(read_sensor_pkg)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(control_position src/control_position.cpp)
ament_target_dependencies(control_position rclcpp px4_msgs)

add_executable(read_sensor src/read_sensor.cpp)
ament_target_dependencies(read_sensor rclcpp px4_msgs)

install(TARGETS
  control_position
  read_sensor
  DESTINATION lib/${PROJECT_NAME})

ament_package()
