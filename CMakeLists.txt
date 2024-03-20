cmake_minimum_required(VERSION 3.8)
project(can_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(can_reader_node src/can_reader.cpp)
ament_target_dependencies(can_reader_node rclcpp )

add_executable(can_reader_cont_node src/can_reader_cont.cpp)
ament_target_dependencies(can_reader_cont_node rclcpp)

add_executable(can_reader_cont_ros_node src/can_reader_cont_ros.cpp)
ament_target_dependencies(can_reader_cont_ros_node rclcpp sensor_msgs)

add_executable(can2ros_imu_nav_node src/can2ros_imu_nav.cpp)
ament_target_dependencies(can2ros_imu_nav_node rclcpp sensor_msgs)

add_executable(can2ros_imu_nav_gnss_pose_node src/can2ros_imu_nav_gnss_pose.cpp)
ament_target_dependencies(can2ros_imu_nav_gnss_pose_node rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  can_reader_node
  can_reader_cont_node
  can_reader_cont_ros_node
  can2ros_imu_nav_node
  can2ros_imu_nav_gnss_pose_node
  
  DESTINATION lib/${PROJECT_NAME})
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
