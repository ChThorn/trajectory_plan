#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "workspace_config.hpp"
#include "table_config.hpp"
#include "robot_operation.hpp"

namespace trajectory_plan
{
class TrajectoryPlanner
{
public:
  TrajectoryPlanner(const rclcpp::Node::SharedPtr& node);
  
  // Core pick and place operations
  bool planPickAndPlace(
    const geometry_msgs::msg::Pose& pick_pose,
    const geometry_msgs::msg::Pose& place_pose,
    double approach_distance = 0.1,
    double retreat_distance = 0.1
  );
  
  bool planMountedTablePickAndPlace(
    const geometry_msgs::msg::Pose& pick_pose,
    const geometry_msgs::msg::Pose& place_pose,
    double approach_distance = 0.08,
    double retreat_distance = 0.08
  );
  
  // Robot operation methods (delegated to RobotOperation)
  bool executePlan();
  bool moveToHome();
  bool moveToSafePosition();
  void setVelocityScaling(double scaling);
  void setAccelerationScaling(double scaling);
  void setSmoothMotion(bool enable = true);
  void printCurrentPose();
  void waitForMotionComplete(double seconds = 1.0);
  
  // Table configuration methods (delegated to TableConfig)
  bool setupTableCollisionObject();
  void setTableDimensions(double length, double width, double height, 
                         double x_offset = 0.0, double y_offset = 0.0);
  
  // Workspace configuration methods (delegated to WorkspaceConfig)
  bool addWorkspaceConstraints();
  
  // Additional validation methods
  bool validateTrajectoryAgainstTable(const moveit_msgs::msg::RobotTrajectory& trajectory);

  void enableWorkspaceVisualization(bool enable = true);
  void setWorkspaceVisualizationColor(float r, float g, float b, float a = 0.7);
  bool validatePoseInWorkspace(const geometry_msgs::msg::Pose& pose);
  void printWorkspaceLimits();

private:

rclcpp::TimerBase::SharedPtr setup_timer_;
  rclcpp::Node::SharedPtr node_;
  
  // Core components
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  
  // Modular components
  std::unique_ptr<WorkspaceConfig> workspace_config_;
  std::unique_ptr<TableConfig> table_config_;
  std::unique_ptr<RobotOperation> robot_operation_;
  
  // Table collision tracking
  bool table_collision_setup_ = false;
  
  // Table dimensions for this class (kept for compatibility)
  struct TableDimensions {
    double length = 1.2;
    double width = 0.8;
    double height = 0.05;
    double x_offset = 0.0;
    double y_offset = 0.0;
    double z_position = -0.027;
  } table_dims_;
};
}

#endif // TRAJECTORY_PLANNER_HPP