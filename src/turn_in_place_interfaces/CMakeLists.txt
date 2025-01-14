cmake_minimum_required(VERSION 3.8)
project(turn_in_place_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TurnCommand.msg"
)

# Export package dependencies
ament_package()