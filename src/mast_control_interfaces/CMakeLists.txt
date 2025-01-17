cmake_minimum_required(VERSION 3.5)
project(mast_control_interfaces)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # Ensure std_msgs is found here

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PanTiltCommand.msg"
  "msg/PanTiltState.msg"
  DEPENDENCIES std_msgs
)

ament_package()