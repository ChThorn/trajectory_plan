#ifndef WORKSPACE_CONFIG_HPP
#define WORKSPACE_CONFIG_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace trajectory_plan
{
class WorkspaceConfig
{
public:
  WorkspaceConfig(const rclcpp::Node::SharedPtr& node, 
                 const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group);
  
  // Set workspace dimensions
  void setWorkspaceDimensions(double width, double depth, double height);
  
  // Set workspace position (center point)
  void setWorkspacePosition(double x, double y, double z);
  
  // Apply workspace constraints to the move group
  bool applyWorkspaceConstraints();
  
  // Clear workspace constraints
  void clearWorkspaceConstraints();

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  
  // Workspace dimensions
  struct WorkspaceDimensions {
    double width = 2.0;   // Width in meters (X direction)
    double depth = 2.0;   // Depth in meters (Y direction)
    double height = 1.0;  // Height in meters (Z direction)
    double x_position = 0.0; // X center position
    double y_position = 0.0; // Y center position
    double z_position = 0.5; // Z center position (above robot base)
  } workspace_dims_;
  
  // Flag to track if constraints are applied
  bool constraints_applied_ = false;
};
}

#endif // WORKSPACE_CONFIG_HPP
