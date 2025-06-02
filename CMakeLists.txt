cmake_minimum_required(VERSION 3.10)
project(trajectory_plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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

# Add include directories
include_directories(
  include
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

# Create the trajectory planner library
add_library(${PROJECT_NAME} 
  src/workspace_config.cpp
  src/table_config.cpp
  src/robot_operation.cpp
  src/trajectory_planner.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_eigen
  tf2
  geometry_msgs
)

# Create the pick and place node executable
add_executable(pick_and_place_node src/pick_and_place_node.cpp)
target_link_libraries(pick_and_place_node ${PROJECT_NAME})
ament_target_dependencies(pick_and_place_node
  rclcpp
  geometry_msgs
  control_msgs
  rclcpp_action
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS pick_and_place_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Export package dependencies
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
)

ament_package()
