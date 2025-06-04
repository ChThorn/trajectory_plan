// =============================================================================
// ENHANCED workspace_config.cpp - With Workspace Boundary Visualization  
// =============================================================================

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
  RCLCPP_INFO(node_->get_logger(), "WorkspaceConfig initialized with visualization support");
  
  // Create marker publisher for workspace boundary visualization
  marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "workspace_boundary", 10);
  
  // Set default workspace boundary color (blue with transparency)
  setWorkspaceBoundaryColor(1.0, 0.0, 0.0, 1.0);
  
  RCLCPP_INFO(node_->get_logger(), "📐 Workspace boundary visualization ready");
}

void WorkspaceConfig::setWorkspaceDimensions(double width, double depth, double height)
{
  workspace_dims_.width = width;
  workspace_dims_.depth = depth;
  workspace_dims_.height = height;
  
  RCLCPP_INFO(node_->get_logger(), "📐 Workspace dimensions updated:");
  RCLCPP_INFO(node_->get_logger(), "   Size: %.2f x %.2f x %.2f m", 
              width, depth, height);
  
  // Update visualization if enabled
  if (viz_settings_.enabled) {
    publishWorkspaceBoundary();
  }
}

void WorkspaceConfig::setWorkspacePosition(double x, double y, double z)
{
  workspace_dims_.x_position = x;
  workspace_dims_.y_position = y;
  workspace_dims_.z_position = z;
  
  RCLCPP_INFO(node_->get_logger(), "📍 Workspace position updated:");
  RCLCPP_INFO(node_->get_logger(), "   Center: [%.3f, %.3f, %.3f] m", 
              x, y, z);
  
  // Update visualization if enabled
  if (viz_settings_.enabled) {
    publishWorkspaceBoundary();
  }
}

void WorkspaceConfig::enableWorkspaceVisualization(bool enable)
{
  viz_settings_.enabled = enable;
  
  if (enable) {
    RCLCPP_INFO(node_->get_logger(), "🔳 Workspace boundary visualization ENABLED");
    
    // Create timer to publish workspace boundary regularly
    visualization_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { publishWorkspaceBoundary(); });
    
    // Publish immediately
    publishWorkspaceBoundary();
  } else {
    RCLCPP_INFO(node_->get_logger(), "🔲 Workspace boundary visualization DISABLED");
    
    // Cancel timer
    if (visualization_timer_) {
      visualization_timer_->cancel();
    }
    
    // Clear markers
    visualization_msgs::msg::MarkerArray clear_markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = move_group_->getPlanningFrame();
    clear_marker.header.stamp = node_->now();
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_markers.markers.push_back(clear_marker);
    marker_publisher_->publish(clear_markers);
  }
}

void WorkspaceConfig::setWorkspaceBoundaryColor(float r, float g, float b, float a)
{
  viz_settings_.color_r = r;
  viz_settings_.color_g = g;
  viz_settings_.color_b = b;
  viz_settings_.color_a = a;
  
  RCLCPP_INFO(node_->get_logger(), "🎨 Workspace boundary color set to RGBA(%.2f, %.2f, %.2f, %.2f)", 
              r, g, b, a);
}

void WorkspaceConfig::publishWorkspaceBoundary()
{
  if (!viz_settings_.enabled) return;
  
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Create the main bounding box wireframe
  createWorkspaceBoundaryMarkers(marker_array);
  
  // Publish the markers
  marker_publisher_->publish(marker_array);
}

void WorkspaceConfig::createWorkspaceBoundaryMarkers(visualization_msgs::msg::MarkerArray& marker_array)
{
  // Clear existing markers
  marker_array.markers.clear();
  
  // Create wireframe bounding box using LINE_LIST
  visualization_msgs::msg::Marker box_marker;
  box_marker.header.frame_id = move_group_->getPlanningFrame();
  box_marker.header.stamp = node_->now();
  box_marker.ns = "workspace_boundary";
  box_marker.id = 0;
  box_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  box_marker.action = visualization_msgs::msg::Marker::ADD;
  
  // Set marker properties
  box_marker.scale.x = viz_settings_.line_width; // Line width
  box_marker.color.r = viz_settings_.color_r;
  box_marker.color.g = viz_settings_.color_g;
  box_marker.color.b = viz_settings_.color_b;
  box_marker.color.a = viz_settings_.color_a;
  
  // Calculate workspace bounds
  double x_min = workspace_dims_.x_position - workspace_dims_.width / 2.0;
  double x_max = workspace_dims_.x_position + workspace_dims_.width / 2.0;
  double y_min = workspace_dims_.y_position - workspace_dims_.depth / 2.0;
  double y_max = workspace_dims_.y_position + workspace_dims_.depth / 2.0;
  double z_min = workspace_dims_.z_position - workspace_dims_.height / 2.0;
  double z_max = workspace_dims_.z_position + workspace_dims_.height / 2.0;
  
  // Define 8 corners of the bounding box
  std::vector<geometry_msgs::msg::Point> corners(8);
  
  corners[0].x = x_min; corners[0].y = y_min; corners[0].z = z_min; // Bottom face
  corners[1].x = x_max; corners[1].y = y_min; corners[1].z = z_min;
  corners[2].x = x_max; corners[2].y = y_max; corners[2].z = z_min;
  corners[3].x = x_min; corners[3].y = y_max; corners[3].z = z_min;
  
  corners[4].x = x_min; corners[4].y = y_min; corners[4].z = z_max; // Top face
  corners[5].x = x_max; corners[5].y = y_min; corners[5].z = z_max;
  corners[6].x = x_max; corners[6].y = y_max; corners[6].z = z_max;
  corners[7].x = x_min; corners[7].y = y_max; corners[7].z = z_max;
  
  // Create wireframe edges (12 edges total)
  // Bottom face edges (4 edges)
  box_marker.points.push_back(corners[0]); box_marker.points.push_back(corners[1]);
  box_marker.points.push_back(corners[1]); box_marker.points.push_back(corners[2]);
  box_marker.points.push_back(corners[2]); box_marker.points.push_back(corners[3]);
  box_marker.points.push_back(corners[3]); box_marker.points.push_back(corners[0]);
  
  // Top face edges (4 edges) 
  box_marker.points.push_back(corners[4]); box_marker.points.push_back(corners[5]);
  box_marker.points.push_back(corners[5]); box_marker.points.push_back(corners[6]);
  box_marker.points.push_back(corners[6]); box_marker.points.push_back(corners[7]);
  box_marker.points.push_back(corners[7]); box_marker.points.push_back(corners[4]);
  
  // Vertical edges (4 edges)
  box_marker.points.push_back(corners[0]); box_marker.points.push_back(corners[4]);
  box_marker.points.push_back(corners[1]); box_marker.points.push_back(corners[5]);
  box_marker.points.push_back(corners[2]); box_marker.points.push_back(corners[6]);
  box_marker.points.push_back(corners[3]); box_marker.points.push_back(corners[7]);
  
  marker_array.markers.push_back(box_marker);
  
  // Create corner markers for better visibility
  for (int i = 0; i < 8; ++i) {
    visualization_msgs::msg::Marker corner_marker;
    corner_marker.header.frame_id = move_group_->getPlanningFrame();
    corner_marker.header.stamp = node_->now();
    corner_marker.ns = "workspace_corners";
    corner_marker.id = i;
    corner_marker.type = visualization_msgs::msg::Marker::SPHERE;
    corner_marker.action = visualization_msgs::msg::Marker::ADD;
    
    corner_marker.pose.position = corners[i];
    corner_marker.pose.orientation.w = 1.0;
    
    corner_marker.scale.x = 0.02; // 2cm diameter spheres
    corner_marker.scale.y = 0.02;
    corner_marker.scale.z = 0.02;
    
    corner_marker.color.r = viz_settings_.color_r;
    corner_marker.color.g = viz_settings_.color_g;
    corner_marker.color.b = viz_settings_.color_b;
    corner_marker.color.a = std::min(1.0f, viz_settings_.color_a + 0.4f); // More opaque corners
    
    marker_array.markers.push_back(corner_marker);
  }
}

bool WorkspaceConfig::isPositionInWorkspace(double x, double y, double z)
{
  double x_min = workspace_dims_.x_position - workspace_dims_.width / 2.0;
  double x_max = workspace_dims_.x_position + workspace_dims_.width / 2.0;
  double y_min = workspace_dims_.y_position - workspace_dims_.depth / 2.0;
  double y_max = workspace_dims_.y_position + workspace_dims_.depth / 2.0;
  double z_min = workspace_dims_.z_position - workspace_dims_.height / 2.0;
  double z_max = workspace_dims_.z_position + workspace_dims_.height / 2.0;
  
  return (x >= x_min && x <= x_max &&
          y >= y_min && y <= y_max &&
          z >= z_min && z <= z_max);
}

bool WorkspaceConfig::validatePoseInWorkspace(const geometry_msgs::msg::Pose& pose)
{
  bool in_workspace = isPositionInWorkspace(pose.position.x, pose.position.y, pose.position.z);
  
  if (!in_workspace) {
    RCLCPP_WARN(node_->get_logger(), 
                "❌ Pose [%.3f, %.3f, %.3f] is OUTSIDE workspace boundary!", 
                pose.position.x, pose.position.y, pose.position.z);
    printWorkspaceLimits();
  } else {
    RCLCPP_INFO(node_->get_logger(), 
                "✅ Pose [%.3f, %.3f, %.3f] is within workspace boundary", 
                pose.position.x, pose.position.y, pose.position.z);
  }
  
  return in_workspace;
}

void WorkspaceConfig::printWorkspaceLimits()
{
  double x_min = workspace_dims_.x_position - workspace_dims_.width / 2.0;
  double x_max = workspace_dims_.x_position + workspace_dims_.width / 2.0;
  double y_min = workspace_dims_.y_position - workspace_dims_.depth / 2.0;
  double y_max = workspace_dims_.y_position + workspace_dims_.depth / 2.0;
  double z_min = workspace_dims_.z_position - workspace_dims_.height / 2.0;
  double z_max = workspace_dims_.z_position + workspace_dims_.height / 2.0;
  
  RCLCPP_INFO(node_->get_logger(), "📐 Current Workspace Limits:");
  RCLCPP_INFO(node_->get_logger(), "   X: [%.3f, %.3f] m", x_min, x_max);
  RCLCPP_INFO(node_->get_logger(), "   Y: [%.3f, %.3f] m", y_min, y_max);
  RCLCPP_INFO(node_->get_logger(), "   Z: [%.3f, %.3f] m", z_min, z_max);
}

bool WorkspaceConfig::addEndEffectorWorkspaceConstraints()
{
  RCLCPP_INFO(node_->get_logger(), "📐 Adding END-EFFECTOR workspace constraints...");
  
  try {
    moveit_msgs::msg::Constraints workspace_constraints;
    
    // Add position constraint for end-effector
    moveit_msgs::msg::PositionConstraint pos_constraint;
    pos_constraint.header.frame_id = move_group_->getPlanningFrame();
    pos_constraint.link_name = move_group_->getEndEffectorLink();
    
    // Create constraint region (workspace bounding box)
    shape_msgs::msg::SolidPrimitive constraint_region;
    constraint_region.type = constraint_region.BOX;
    constraint_region.dimensions = {
      workspace_dims_.width, 
      workspace_dims_.depth, 
      workspace_dims_.height
    };
    
    geometry_msgs::msg::Pose constraint_pose;
    constraint_pose.position.x = workspace_dims_.x_position;
    constraint_pose.position.y = workspace_dims_.y_position;
    constraint_pose.position.z = workspace_dims_.z_position;
    constraint_pose.orientation.w = 1.0;
    
    pos_constraint.constraint_region.primitives.push_back(constraint_region);
    pos_constraint.constraint_region.primitive_poses.push_back(constraint_pose);
    pos_constraint.weight = 1.0;
    
    workspace_constraints.position_constraints.push_back(pos_constraint);
    
    // Apply constraints to move group
    move_group_->setPathConstraints(workspace_constraints);
    constraints_applied_ = true;
    
    RCLCPP_INFO(node_->get_logger(), "✅ End-effector workspace constraints applied");
    RCLCPP_INFO(node_->get_logger(), "   End-effector: %s", move_group_->getEndEffectorLink().c_str());
    printWorkspaceLimits();
    
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Exception in addEndEffectorWorkspaceConstraints: %s", e.what());
    return false;
  }
}

bool WorkspaceConfig::applyWorkspaceConstraints()
{
  // Use the new end-effector constraint method
//   return addEndEffectorWorkspaceConstraints();
  return addFullArmWorkspaceConstraints();
}

void WorkspaceConfig::clearWorkspaceConstraints()
{
  if (constraints_applied_) {
    move_group_->clearPathConstraints();
    constraints_applied_ = false;
    RCLCPP_INFO(node_->get_logger(), "🧹 Workspace constraints cleared");
  }
}

bool WorkspaceConfig::printRobotLinkNames()
{
  RCLCPP_INFO(node_->get_logger(), "🔍 ROBOT LINK ANALYSIS:");
  
  try {
    auto robot_model = move_group_->getRobotModel();
    auto joint_group = robot_model->getJointModelGroup("mainpulation");
    auto link_names = joint_group->getLinkModelNames();
    
    RCLCPP_INFO(node_->get_logger(), "📋 All links in 'mainpulation' group (%zu total):", link_names.size());
    for (size_t i = 0; i < link_names.size(); ++i) {
      RCLCPP_INFO(node_->get_logger(), "   [%zu] %s", i, link_names[i].c_str());
    }
    
    // Suggest which links to constrain
    RCLCPP_INFO(node_->get_logger(), "💡 Recommended links to constrain (exclude base/fixed links):");
    for (const auto& link : link_names) {
      if (link != "link0" && link != "base_link" && link != "base") {
        RCLCPP_INFO(node_->get_logger(), "   ✅ %s", link.c_str());
      } else {
        RCLCPP_INFO(node_->get_logger(), "   ❌ %s (skip - base/fixed link)", link.c_str());
      }
    }
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Error analyzing robot links: %s", e.what());
    return false;
  }
}


bool WorkspaceConfig::addFullArmWorkspaceConstraints()
{
  RCLCPP_INFO(node_->get_logger(), "📐 Adding FULL ARM workspace constraints...");
  
  try {
    // First, print robot links for reference
    printRobotLinkNames();
    
    moveit_msgs::msg::Constraints workspace_constraints;
    
    // Get robot model
    auto robot_model = move_group_->getRobotModel();
    auto joint_group = robot_model->getJointModelGroup("mainpulation");
    auto all_links = joint_group->getLinkModelNames();
    
    // Define links to constrain (adjust based on your robot)
    // Start with common robot link names, then customize based on printRobotLinkNames() output
    std::vector<std::string> target_links = {
      "link1", "link2", "link3", "link4", "link5", "link6",  // Common 6-DOF robot links
      "tcp", "tool0", "ee_link", "end_effector"              // Common end-effector names
    };
    
    // Filter to only existing links
    std::vector<std::string> valid_links;
    for (const auto& target : target_links) {
      if (std::find(all_links.begin(), all_links.end(), target) != all_links.end()) {
        valid_links.push_back(target);
      }
    }
    
    // If no predefined links found, use auto-detection (exclude base links)
    if (valid_links.empty()) {
      RCLCPP_WARN(node_->get_logger(), "⚠️ No predefined links found, using auto-detection...");
      for (const auto& link : all_links) {
        if (link != "link0" && link != "base_link" && link != "base") {
          valid_links.push_back(link);
        }
      }
    }
    
    RCLCPP_INFO(node_->get_logger(), "🔗 Constraining %zu robot links:", valid_links.size());
    
    // Add position constraints for each valid link
    for (const auto& link_name : valid_links) {
      moveit_msgs::msg::PositionConstraint pos_constraint;
      pos_constraint.header.frame_id = move_group_->getPlanningFrame();
      pos_constraint.link_name = link_name;
      
      // Create constraint region (expanded workspace bounding box)
      shape_msgs::msg::SolidPrimitive constraint_region;
      constraint_region.type = constraint_region.BOX;
      constraint_region.dimensions = {
        workspace_dims_.width, 
        workspace_dims_.depth, 
        workspace_dims_.height
      };
      
      geometry_msgs::msg::Pose constraint_pose;
      constraint_pose.position.x = workspace_dims_.x_position;
      constraint_pose.position.y = workspace_dims_.y_position;
      constraint_pose.position.z = workspace_dims_.z_position;
      constraint_pose.orientation.w = 1.0;
      
      pos_constraint.constraint_region.primitives.push_back(constraint_region);
      pos_constraint.constraint_region.primitive_poses.push_back(constraint_pose);
      pos_constraint.weight = 1.0;  // Equal weight for all links
      
      workspace_constraints.position_constraints.push_back(pos_constraint);
      
      RCLCPP_INFO(node_->get_logger(), "   ✅ %s", link_name.c_str());
    }
    
    if (valid_links.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "❌ No valid links found to constrain!");
      return false;
    }
    
    // Apply constraints to move group
    move_group_->setPathConstraints(workspace_constraints);
    constraints_applied_ = true;
    
    RCLCPP_INFO(node_->get_logger(), "✅ FULL ARM workspace constraints applied successfully!");
    RCLCPP_INFO(node_->get_logger(), "   🔗 Constrained links: %zu", valid_links.size());
    RCLCPP_INFO(node_->get_logger(), "   🛡️ ALL robot links must stay within workspace boundary");
    RCLCPP_INFO(node_->get_logger(), "   🎯 Robot should no longer extend outside red/blue box");
    
    printWorkspaceLimits();
    
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Exception in addFullArmWorkspaceConstraints: %s", e.what());
    return false;
  }
}

} // namespace trajectory_plan