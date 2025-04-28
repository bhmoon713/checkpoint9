cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
#   "action/GoToPose.action"
  DEPENDENCIES sensor_msgs geometry_msgs
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

add_executable(patrol_with_service src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service rclcpp sensor_msgs geometry_msgs custom_interfaces)

add_executable(direction_service src/direction_service.cpp)
ament_target_dependencies(direction_service rclcpp sensor_msgs custom_interfaces)

add_executable(test_service src/test_service.cpp)
ament_target_dependencies(test_service rclcpp sensor_msgs custom_interfaces)

add_executable(pre_approach_node src/pre_approach.cpp)
ament_target_dependencies(pre_approach_node
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  custom_interfaces
)

add_executable(laser_to_cart_tf_publisher src/laser_to_cart_tf_publisher.cpp)
ament_target_dependencies(laser_to_cart_tf_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

add_executable(robot_chase src/robot_chase.cpp)
ament_target_dependencies(robot_chase rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)


install(TARGETS
  pre_approach_node
  patrol_with_service
  direction_service
  test_service
  laser_to_cart_tf_publisher
  robot_chase
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  rviz_config
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()
