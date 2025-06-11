#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "robot_operation.hpp"

namespace trajectory_plan
{

// Configuration structure - single source of truth
struct PlannerConfiguration
{
  // Table configuration
  struct {
    double length = 0.98;
    double width = 0.49;
    double height = 0.04;
    double x_offset = 0.0;
    double y_offset = 0.0;
    double z_position = -0.027;
  } table;
  
  // Workspace configuration
  struct {
    double width = 1.4;
    double depth = 1.2;
    double height = 1.0;
    double x_position = 0.4;
    double y_position = 0.0;
    double z_position = 0.5;
    
    // Visualization settings
    bool visualization_enabled = false;
    float color_r = 1.0, color_g = 0.0, color_b = 0.0, color_a = 1.0;
  } workspace;
  
  // Robot configuration
  struct {
    double velocity_scaling = 0.3;
    double acceleration_scaling = 0.3;
    std::vector<double> home_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> safe_joints = {0.0, -0.5, -1.0, 0.0, 1.5, 0.0};
  } robot;
};

class TrajectoryPlanner
{
public:
  explicit TrajectoryPlanner(const rclcpp::Node::SharedPtr& node);
  
  // Configuration methods
  bool initialize();
  void updateConfiguration(const PlannerConfiguration& config);
  const PlannerConfiguration& getConfiguration() const { return config_; }
  
  // Core planning operations
  bool planPickAndPlace(
    const geometry_msgs::msg::Pose& pick_pose,
    const geometry_msgs::msg::Pose& place_pose,
    double approach_distance = 0.08,
    double retreat_distance = 0.08
  );
  
  // Professional pick-and-place with mm/degrees input (NEW)
  bool executeProfessionalPickAndPlace(
    double pick_x_mm, double pick_y_mm, double pick_z_mm,
    double pick_roll_deg, double pick_pitch_deg, double pick_yaw_deg,
    double place_x_mm, double place_y_mm, double place_z_mm,
    double place_roll_deg, double place_pitch_deg, double place_yaw_deg,
    double clearance_height_mm = 50.0
  );
  
  // Robot control methods - delegated to RobotOperation
  bool executePlan();
  bool moveToHome();
  bool moveToSafePosition();
  void setVelocityScaling(double scaling);
  void setAccelerationScaling(double scaling);
  void setSmoothMotion(bool enable = true);
  void printCurrentPose();
  void waitForMotionComplete(double seconds = 1.0);
  
  // Scene management methods
  bool setupCollisionScene();
  bool addWorkspaceConstraints();
  void clearWorkspaceConstraints();
  
  // Validation methods
  bool validatePoseInWorkspace(const geometry_msgs::msg::Pose& pose);
  bool validateTrajectoryCollisionFree(const moveit_msgs::msg::RobotTrajectory& trajectory);
  
  // Visualization methods
  void enableWorkspaceVisualization(bool enable = true);
  void setWorkspaceVisualizationColor(float r, float g, float b, float a = 1.0);
  void printWorkspaceLimits();

private:
  // Core ROS and MoveIt components
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  
  // Robot operation handler
  std::unique_ptr<RobotOperation> robot_operation_;
  
  // Configuration
  PlannerConfiguration config_;
  
  // State tracking
  bool initialized_ = false;
  bool table_collision_setup_ = false;
  bool workspace_constraints_applied_ = false;
  
  // Visualization
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  rclcpp::TimerBase::SharedPtr setup_timer_;
  
  // Private setup methods
  bool initializeMoveIt();
  bool initializeRobotOperation();
  bool initializeVisualization();
  void performDelayedSetup();
  
  // Scene management private methods
  bool setupTableCollisionObject();
  bool addFullArmWorkspaceConstraints();
  
  // Validation private methods
  bool isPositionInWorkspace(double x, double y, double z) const;
  bool printRobotLinkNames();
  
  // Visualization private methods
  void publishWorkspaceBoundary();
  void createWorkspaceBoundaryMarkers(visualization_msgs::msg::MarkerArray& marker_array);
  
  // Utility methods
  geometry_msgs::msg::Pose createApproachPose(const geometry_msgs::msg::Pose& target_pose, double distance);
  geometry_msgs::msg::Pose createRetreatPose(const geometry_msgs::msg::Pose& target_pose, double distance);
};

} // namespace trajectory_plan

#endif // TRAJECTORY_PLANNER_HPP