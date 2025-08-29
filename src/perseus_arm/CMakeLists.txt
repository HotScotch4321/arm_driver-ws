cmake_minimum_required(VERSION 3.8)
project(perseus_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Core ROS2 dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Hardware interface dependencies
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Find pigpio
find_package(PkgConfig REQUIRED)
pkg_check_modules(PIGPIO REQUIRED libpigpiod_if2)

# Create the hardware interface library
add_library(perseus_arm_hardware SHARED
  src/arm_driver.cpp
)

target_include_directories(perseus_arm_hardware PRIVATE
  include
  ${PIGPIO_INCLUDE_DIRS}
)

target_link_libraries(perseus_arm_hardware
  ${PIGPIO_LIBRARIES}
)

ament_target_dependencies(perseus_arm_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# Register the plugin
pluginlib_export_plugin_description_file(hardware_interface perseus_arm_hardware.xml)

# Install the hardware library
install(TARGETS perseus_arm_hardware
  DESTINATION lib
)

# Install plugin description file
install(FILES perseus_arm_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(FILES
  robot.urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install assets directory
install(DIRECTORY
  assets/
  DESTINATION share/${PROJECT_NAME}/assets
  FILES_MATCHING PATTERN "*.stl"
)

# Install launch files (if you have any)
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()