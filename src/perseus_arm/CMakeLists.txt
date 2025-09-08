cmake_minimum_required(VERSION 3.8)
project(perseus_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Build hardware interface library
add_library(${PROJECT_NAME} SHARED src/arm_driver.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Register plugin
pluginlib_export_plugin_description_file(hardware_interface perseus_arm_plugin.xml)

# Install library and plugin description
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES perseus_arm_plugin.xml DESTINATION share/${PROJECT_NAME})
install(FILES robot.urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY assets/ DESTINATION share/${PROJECT_NAME}/assets)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()