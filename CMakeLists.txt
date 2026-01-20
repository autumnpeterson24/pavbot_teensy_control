cmake_minimum_required(VERSION 3.8)
project(pavbot_teensy_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pavbot_msgs REQUIRED)

include_directories(include)

add_executable(cmd_vel_to_wheels
  src/cmd_vel_to_wheels.cpp
)
ament_target_dependencies(cmd_vel_to_wheels
  rclcpp geometry_msgs std_msgs pavbot_msgs
)

add_executable(teensy_transport_stub
  src/teensy_transport_stub.cpp
)
ament_target_dependencies(teensy_transport_stub
  rclcpp std_msgs pavbot_msgs
)

install(TARGETS
  cmd_vel_to_wheels
  teensy_transport_stub
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
