cmake_minimum_required(VERSION 3.8)
project(evarobot_rc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# RC Receiver Node
add_executable(rc_receiver_node
  src/rc_receiver_node.cpp
  src/crsf_parser.cpp
  src/serial_port.cpp
  src/channel_mapper.cpp
)

ament_target_dependencies(rc_receiver_node
  rclcpp
  geometry_msgs
  std_msgs
)

install(TARGETS
  rc_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
