#include "trajectory_plan/workspace_config.hpp"
#include <algorithm>

namespace trajectory_plan
{
WorkspaceConfig::WorkspaceConfig(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group)
  : node_(node)
  , move_group_(move_group)
{
  RCLCPP_INFO(node_->get_logger(), "WorkspaceConfig initialized");
}

void WorkspaceConfig::setWorkspaceDimensions(double width, double depth, double height)
{
  workspace_dims_.width = width;
  workspace_dims_.depth = depth;
  workspace_dims_.height = height;
  
  RCLCPP_INFO(node_->get_logger(), "📐 Workspace dimensions updated:");
  RCLCPP_INFO(node_->get_logger(), "   Size: %.2f x %.2f x %.2f m", 
              width, depth, height);
}

void WorkspaceConfig::setWorkspacePosition(double x, double y, double z)
{
  workspace_dims_.x_position = x;
  workspace_dims_.y_position = y;
  workspace_dims_.z_position = z;
  
  RCLCPP_INFO(node_->get_logger(), "📍 Workspace position updated:");
  RCLCPP_INFO(node_->get_logger(), "   Position: [%.3f, %.3f, %.3f] m", 
              x, y, z);
}

bool WorkspaceConfig::applyWorkspaceConstraints()
{
  RCLCPP_INFO(node_->get_logger(), "📐 Adding workspace constraints...");
  
  // Add position constraints for critical links to stay above table
  moveit_msgs::msg::Constraints workspace_constraints;
  
  // Get robot model to identify critical links
  auto robot_model = move_group_->getRobotModel();
  auto joint_group = robot_model->getJointModelGroup("mainpulation"); // Keep working name
  auto link_names = joint_group->getLinkModelNames();
  
  // Add position constraints for links that could collide with table
  // Focus on elbow and wrist links
  std::vector<std::string> critical_links = {"link3", "link4", "link5"}; // Adjust based on your URDF
  
  for (const auto& link_name : critical_links) {
    // Check if link exists in robot model
    if (std::find(link_names.begin(), link_names.end(), link_name) != link_names.end()) {
      
      moveit_msgs::msg::PositionConstraint pos_constraint;
      pos_constraint.header.frame_id = move_group_->getPlanningFrame();
      pos_constraint.link_name = link_name;
      
      // Create constraint region (above table)
      shape_msgs::msg::SolidPrimitive constraint_region;
      constraint_region.type = constraint_region.BOX;
      constraint_region.dimensions = {2.0, 2.0, 1.0}; // Large working volume above table
      
      geometry_msgs::msg::Pose constraint_pose;
      constraint_pose.position.x = 0.0;
      constraint_pose.position.y = 0.0; 
      constraint_pose.position.z = 0.0;  // At robot base level (from working code)
      constraint_pose.orientation.w = 1.0;
      
      pos_constraint.constraint_region.primitives.push_back(constraint_region);
      pos_constraint.constraint_region.primitive_poses.push_back(constraint_pose);
      pos_constraint.weight = 1.0;
      
      workspace_constraints.position_constraints.push_back(pos_constraint);
      
      RCLCPP_INFO(node_->get_logger(), "   Added constraint for link: %s", link_name.c_str());
    }
  }
  
  // Apply constraints to move group
  move_group_->setPathConstraints(workspace_constraints);
  constraints_applied_ = true;
  
  RCLCPP_INFO(node_->get_logger(), "✅ Workspace constraints applied");
  return true;
}

void WorkspaceConfig::clearWorkspaceConstraints()
{
  if (constraints_applied_) {
    move_group_->clearPathConstraints();
    constraints_applied_ = false;
    RCLCPP_INFO(node_->get_logger(), "🧹 Workspace constraints cleared");
  }
}

} // namespace trajectory_plan