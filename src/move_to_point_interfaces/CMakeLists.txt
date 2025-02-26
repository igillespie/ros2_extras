cmake_minimum_required(VERSION 3.8)
project(move_to_point_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_py REQUIRED)  # ✅ Ensure Python bindings are generated
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPath.srv"
  "action/MoveToPoint.action"
  DEPENDENCIES geometry_msgs std_msgs
)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}
  DESTINATION lib/python3.11/site-packages/
)

ament_package()