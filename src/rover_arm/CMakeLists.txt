cmake_minimum_required(VERSION 3.8)
project(rover_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PiPCA9685 REQUIRED)

# Add executable
add_library(rover_arm_hardware SHARED src/rover_arm_hardware.cpp)
ament_target_dependencies(rover_arm_hardware rclcpp hardware_interface pluginlib PiPCA9685)

find_library(I2C_LIBRARY i2c REQUIRED)

target_link_libraries(rover_arm_hardware
  PiPCA9685
  ${I2C_LIBRARY}  # Link the I2C library
)

# Declare a plugin
pluginlib_export_plugin_description_file(hardware_interface rover_arm_hardware_plugin.xml)

install(
  TARGETS rover_arm_hardware
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES rover_arm_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration files
install(
  FILES config/rover_arm_controllers.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
