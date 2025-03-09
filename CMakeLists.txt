cmake_minimum_required(VERSION 3.8)
project(myrobot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_msgs REQUIRED)

#find_package(ament_auto REQUIRED)

#################
# Setup targets #
#################

#include_directories()
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# Add the include directory
#include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(lidar_transform src/lidar_transform.cpp)
ament_target_dependencies(lidar_transform
  rclcpp
  sensor_msgs
  pcl_ros
  tf2_ros
  geometry_msgs
)
add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node
  rclcpp
  std_msgs
)

############
# Install ##
############

ament_export_include_directories(include/${PROJECT_NAME})

install(TARGETS 
  test_node
  lidar_transform 
  DESTINATION lib/${PROJECT_NAME})
#install(TARGETS  DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

###########
# Python ##
###########

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/test.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/joy_control.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/frame_manager.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

############
# Testing ##
############

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
