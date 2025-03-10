cmake_minimum_required(VERSION 3.5)
project(quadcopter)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(control_position_node src/control_position.cpp)
ament_target_dependencies(control_position_node rclcpp px4_msgs)

add_executable(read_sensor_node src/read_sensor.cpp)
ament_target_dependencies(read_sensor_node rclcpp px4_msgs)

add_executable(processes src/processes.cpp)

install(TARGETS
  control_position_node
  read_sensor_node
  processes
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
