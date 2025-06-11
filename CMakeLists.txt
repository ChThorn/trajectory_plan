cmake_minimum_required(VERSION 3.10)
project(trajectory_plan)

# Set C++ standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

# Create trajectory planner library (simplified structure)
add_library(${PROJECT_NAME} 
  src/trajectory_planner.cpp
  src/robot_operation.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_eigen
  tf2
  geometry_msgs
  visualization_msgs
)

# Create executable
add_executable(pick_and_place_node src/pick_and_place_node.cpp)

target_link_libraries(pick_and_place_node ${PROJECT_NAME})

ament_target_dependencies(pick_and_place_node
  rclcpp
  geometry_msgs
  control_msgs
  rclcpp_action
  tf2
)

# Install library
install(TARGETS ${PROJECT_NAME}
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

# Install config files (if any)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

# Export package information
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_eigen
  tf2
  geometry_msgs
  visualization_msgs
)

ament_package()