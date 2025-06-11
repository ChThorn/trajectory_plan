#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include "robot_operation.hpp"

namespace trajectory_plan
{

/**
 * @struct PlannerConfiguration
 * @brief Consolidates all configurable parameters for the planning environment.
 */
struct PlannerConfiguration
{
  struct {
    double length = 0.98, width = 0.49, height = 0.04;
    double x_offset = 0.0, y_offset = 0.0, z_position = -0.027; // Centered on the base_link
  } table;
  
  struct {
    double width = 1.4, depth = 1.2, height = 1.0;
    double x_position = 0.4, y_position = 0.0, z_position = 0.5;
    bool visualization_enabled = false;
  } workspace;
  
  struct {
    double velocity_scaling = 0.3, acceleration_scaling = 0.3;
  } robot;
};

/**
 * @class TrajectoryPlanner
 * @brief Orchestrates high-level robotic tasks, managing the planning scene and robot operations.
 */
class TrajectoryPlanner
{
public:
  explicit TrajectoryPlanner(const rclcpp::Node::SharedPtr& node);
  
  // --- Setup and Configuration ---
  bool initialize();
  void updateConfiguration(const PlannerConfiguration& config);
  
  // --- High-Level Operations ---
  bool executeProfessionalPickAndPlace(
    double pick_x_mm, double pick_y_mm, double pick_z_mm,
    double pick_roll_deg, double pick_pitch_deg, double pick_yaw_deg,
    double place_x_mm, double place_y_mm, double place_z_mm,
    double place_roll_deg, double place_pitch_deg, double place_yaw_deg,
    double clearance_height_mm = 50.0
  );
  
  // --- Delegated Robot Actions ---
  bool moveToHome();
  bool moveToSafePosition();
  void setSmoothMotion(bool enable = true);
  void printCurrentPose();
  
  // --- Scene and Visualization ---
  void enableWorkspaceVisualization(bool enable = true);
  void printWorkspaceLimits();

private:
  // --- Core Components ---
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::unique_ptr<RobotOperation> robot_operation_;
  
  // --- Configuration & State ---
  PlannerConfiguration config_;
  bool initialized_ = false;
  
  // --- Visualization ---
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  
  // --- Private Setup Methods ---
  bool setupCollisionScene();
  bool addWorkspaceConstraints();
  void publishWorkspaceBoundary();
  
  // --- Validation ---
  bool validatePoseInWorkspace(const geometry_msgs::msg::Pose& pose);
};

} // namespace trajectory_plan

#endif // TRAJECTORY_PLANNER_HPP
