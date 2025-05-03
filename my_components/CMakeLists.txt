cmake_minimum_required(VERSION 3.8)
project(my_components)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(custom_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

include_directories(include)
# moverobot_component 
add_library(moverobot_component SHARED src/moverobot_component.cpp)
target_compile_definitions(moverobot_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(moverobot_component
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs")
rclcpp_components_register_nodes(moverobot_component "my_components::MoveRobot")
set(node_plugins "${node_plugins}my_components::MoveRobot;$<TARGET_FILE:moverobot_component>\n")

# pre_approach_component 
add_library(pre_approach_component SHARED src/pre_approach_component.cpp)
target_compile_definitions(pre_approach_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(pre_approach_component
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "nav_msgs"
  "geometry_msgs")
rclcpp_components_register_nodes(pre_approach_component "my_components::PreApproach")
set(node_plugins "${node_plugins}my_components::PreApproach;$<TARGET_FILE:pre_approach_component>\n")

# attach_server_component 
add_library(attach_server_component SHARED src/attach_server_component.cpp)
target_compile_definitions(attach_server_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(attach_server_component
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "custom_interfaces"
  "geometry_msgs")
rclcpp_components_register_nodes(attach_server_component "my_components::AttachServer")
set(node_plugins "${node_plugins}my_components::AttachServer;$<TARGET_FILE:attach_server_component>\n")

# attach_client_component 
add_library(attach_client_component SHARED src/attach_client_component.cpp)
target_compile_definitions(attach_client_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(attach_client_component
  "rclcpp"
  "rclcpp_components"
#   "sensor_msgs"
#   "nav_msgs"
#   "geometry_msgs"
#   "tf2"
#   "tf2_ros"
#   "tf2_geometry_msgs"
  "custom_interfaces")

rclcpp_components_register_nodes(attach_client_component "my_components::AttachClient")
set(node_plugins "${node_plugins}my_components::AttachClient;$<TARGET_FILE:attach_client_component>\n")


add_library(readodom_component SHARED src/readodom_component.cpp)
target_compile_definitions(readodom_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(readodom_component
  "rclcpp"
  "rclcpp_components"
  "nav_msgs")
rclcpp_components_register_nodes(readodom_component "my_components::OdomSubscriber")
set(node_plugins "${node_plugins}my_components::OdomSubscriber;$<TARGET_FILE:readodom_component>\n")

# pre_approach_component 
add_executable(manual_composition src/manual_composition.cpp)
target_link_libraries(manual_composition moverobot_component readodom_component)
ament_target_dependencies(manual_composition "rclcpp")

add_library(server_component SHARED src/server_component.cpp)
target_compile_definitions(server_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(server_component
  "std_srvs"
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs")
rclcpp_components_register_nodes(server_component "my_components::Server")
set(node_plugins "${node_plugins}my_components::Server;$<TARGET_FILE:server_component>\n")

install(TARGETS
  manual_composition
  server_component
  pre_approach_component 
  attach_server_component
  readodom_component
  attach_client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# Install launch files.

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()