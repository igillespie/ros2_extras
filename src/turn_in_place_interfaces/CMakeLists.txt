cmake_minimum_required(VERSION 3.8)
project(turn_in_place_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_py REQUIRED)  # ✅ Ensure Python bindings are generated
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Turn.action"
  DEPENDENCIES geometry_msgs action_msgs
)

# Ensure Python bindings are properly installed
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}
  DESTINATION lib/python3.11/site-packages/
)

# Export package dependencies
ament_package()