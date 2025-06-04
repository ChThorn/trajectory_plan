// =============================================================================
// ENHANCED workspace_config.hpp - With Workspace Boundary Visualization
// =============================================================================

#ifndef WORKSPACE_CONFIG_HPP
#define WORKSPACE_CONFIG_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
  
  // NEW: Workspace boundary visualization
  void enableWorkspaceVisualization(bool enable = true);
  void publishWorkspaceBoundary();
  void setWorkspaceBoundaryColor(float r, float g, float b, float a = 0.3);
  
  // NEW: Enhanced constraint methods
  bool addEndEffectorWorkspaceConstraints();
  bool validatePoseInWorkspace(const geometry_msgs::msg::Pose& pose);
  
  // NEW: Workspace boundary checking
  bool isPositionInWorkspace(double x, double y, double z);
  void printWorkspaceLimits();

  // NEW: Full arm workspace constraints
  bool addFullArmWorkspaceConstraints();
  bool printRobotLinkNames();

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  
  // NEW: Visualization publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  
  // Workspace dimensions
  struct WorkspaceDimensions {
    double width = 1.0;      // Width in meters (X direction)
    double depth = 1.0;      // Depth in meters (Y direction)  
    double height = 0.8;     // Height in meters (Z direction)
    double x_position = 0.4; // X center position (forward from base)
    double y_position = 0.0; // Y center position (left/right)
    double z_position = 0.2; // Z center position (above base)
  } workspace_dims_;
  
  // NEW: Visualization settings
  struct VisualizationSettings {
    bool enabled = false;
    float color_r = 0.0;
    float color_g = 0.0; 
    float color_b = 1.0;
    float color_a = 0.3;
    double line_width = 0.01;
  } viz_settings_;
  
  // Flag to track if constraints are applied
  bool constraints_applied_ = false;
  
  // NEW: Private helper methods
  void createWorkspaceBoundaryMarkers(visualization_msgs::msg::MarkerArray& marker_array);
  void createBoundingBoxMarker(visualization_msgs::msg::Marker& marker);
  void createCornerMarkersMarker(visualization_msgs::msg::MarkerArray& marker_array);
};
}

#endif // WORKSPACE_CONFIG_HPP