cmake_minimum_required(VERSION 3.8)
project(evarobot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Optional dependencies for future hardware interface implementation
# Uncomment when implementing the hardware interface
# find_package(rclcpp REQUIRED)
# find_package(hardware_interface REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)

# Include directories (if directory exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  include_directories(include)
endif()

# Hardware interface library (will be implemented later)
# add_library(${PROJECT_NAME} SHARED
#   src/evarobot_hardware_interface.cpp
# )
# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   hardware_interface
#   pluginlib
#   rclcpp_lifecycle
# )

# Install hardware interface plugin
# pluginlib_export_plugin_description_file(hardware_interface evarobot_hardware_interface.xml)

# Install include files (only if directory exists and has content)
# Commented out since include directory is currently empty
# install(DIRECTORY include/
#   DESTINATION include
# )

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install targets
# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
