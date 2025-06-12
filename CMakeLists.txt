cmake_minimum_required(VERSION 3.10)
project(trajectory_plan)

# Set C++ standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interfaces FIRST (creates trajectory_plan target)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickAndPlace.action"
  DEPENDENCIES 
  geometry_msgs
  action_msgs
)

# Include directories
include_directories(
  include
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

# Create trajectory planner library with DIFFERENT name to avoid conflict
add_library(${PROJECT_NAME}_lib 
  src/trajectory_planner.cpp
  src/robot_operation.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to library
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_eigen
  tf2
  geometry_msgs
  visualization_msgs
  action_msgs
)

# Create executable
add_executable(pick_and_place_node src/pick_and_place_node.cpp)

# Link library to executable
target_link_libraries(pick_and_place_node ${PROJECT_NAME}_lib)

ament_target_dependencies(pick_and_place_node
  rclcpp
  rclcpp_action
  geometry_msgs
  control_msgs
  tf2
)

# Link action interfaces to library and executable
rosidl_target_interfaces(${PROJECT_NAME}_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(pick_and_place_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install library
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executable
install(TARGETS pick_and_place_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install action files
install(
  DIRECTORY action/
  DESTINATION share/${PROJECT_NAME}/action
)

# Install config files (if any)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

# Export package information
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_eigen
  tf2
  geometry_msgs
  visualization_msgs
  action_msgs
)

# Export action interfaces
ament_export_dependencies(rosidl_default_runtime)

ament_package()