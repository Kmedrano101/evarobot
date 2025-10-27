cmake_minimum_required(VERSION 3.8)
project(evarobot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  evarobot_firmware/arduino_serial_com.py
  evarobot_firmware/keyboard_control.py
  evarobot_firmware/joy_teleop_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install test scripts
install(PROGRAMS
  tests/test_motor_serial.py
  tests/test_pid_feedback.py
  DESTINATION lib/${PROJECT_NAME}
)

# Note: Arduino firmware (firmware/evarobot_control/) is maintained in the repo
# but not installed with ROS2 package. Upload directly to Arduino using Arduino IDE.

# Install README
install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.launch.py"
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
  PATTERN "*.yaml"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
