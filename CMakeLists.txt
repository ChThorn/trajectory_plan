# cmake_minimum_required(VERSION 3.12)
# project(trajectory_plan)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(moveit_core REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)
# find_package(moveit_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_eigen REQUIRED)
# find_package(control_msgs REQUIRED)
# find_package(rclcpp_action REQUIRED)

# set(dependencies
#   rclcpp
#   rclcpp_action
#   control_msgs
#   moveit_core
#   moveit_ros_planning_interface
#   moveit_msgs
#   geometry_msgs
#   tf2
#   tf2_ros
#   tf2_eigen
# )

# # Include directories
# include_directories(include)

# # Trajectory planner library
# add_library(trajectory_planner_lib
#   src/trajectory_planner.cpp
# )
# ament_target_dependencies(trajectory_planner_lib ${dependencies})

# # Pick and place node
# add_executable(pick_and_place_node
#   src/pick_and_place_node.cpp
# )
# ament_target_dependencies(pick_and_place_node ${dependencies})
# target_link_libraries(pick_and_place_node trajectory_planner_lib)

# # Install
# install(TARGETS
#   trajectory_planner_lib
#   pick_and_place_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY
#   launch
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY
#   include/
#   DESTINATION include
# )

# ament_package()


cmake_minimum_required(VERSION 3.12)
project(trajectory_plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
# Additional packages for robot_state_monitor
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  control_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  std_msgs
)

# Include directories
include_directories(include)

# Trajectory planner library
add_library(trajectory_planner_lib
  src/trajectory_planner.cpp
)
ament_target_dependencies(trajectory_planner_lib ${dependencies})

# Pick and place node
add_executable(pick_and_place_node
  src/pick_and_place_node.cpp
)
ament_target_dependencies(pick_and_place_node ${dependencies})
target_link_libraries(pick_and_place_node trajectory_planner_lib)

# Robot state monitor node
add_executable(robot_state_monitor
  src/robot_state_monitor.cpp
)
ament_target_dependencies(robot_state_monitor ${dependencies})

# Install
install(TARGETS
  trajectory_planner_lib
  pick_and_place_node
  robot_state_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

# Install Python scripts (optional - if using Python version)
# install(PROGRAMS
#   scripts/robot_state_monitor.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()