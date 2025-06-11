#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include "robot_operation.hpp"

namespace trajectory_plan
{

    struct PlannerConfiguration
    {
        struct {
            double length = 0.98, width = 0.49, height = 0.04;
            double x_offset = 0.0, y_offset = 0.0, z_position = -0.027;
        } table;
        
        struct {
            double width = 1.4, depth = 1.2, height = 1.0;
            double x_position = 0.4, y_position = 0.0, z_position = 0.5;
            bool visualization_enabled = false;
        } workspace;
        
        struct {
            double velocity_scaling = 0.3, acceleration_scaling = 0.3;
            double planning_time = 5.0;           // NEW: configurable planning time
            int planning_attempts = 10;           // NEW: configurable planning attempts
            double cartesian_step_size = 0.01;   // NEW: configurable Cartesian step size
            double cartesian_jump_threshold = 0.0; // NEW: configurable jump threshold
        } robot;
        
        struct {
            double gripper_action_time = 1.0;     // NEW: configurable gripper timing
            std::string manipulation_group = "manipulation"; // NEW: configurable group name
        } gripper;
    };

class TrajectoryPlanner
{
public:
  
 ~TrajectoryPlanner() {
    // Ensure proper cleanup of timers and publishers
    if (visualization_timer_) {
        visualization_timer_->cancel();
    }
    if (marker_publisher_) {
        marker_publisher_.reset();
    }
    RCLCPP_DEBUG(node_->get_logger(), "TrajectoryPlanner destroyed");
  }
  explicit TrajectoryPlanner(const rclcpp::Node::SharedPtr& node);
  
  bool initialize();
  void updateConfiguration(const PlannerConfiguration& config);
  
  bool executeProfessionalPickAndPlace(
    double pick_x_mm, double pick_y_mm, double pick_z_mm,
    double pick_roll_deg, double pick_pitch_deg, double pick_yaw_deg,
    double place_x_mm, double place_y_mm, double place_z_mm,
    double place_roll_deg, double place_pitch_deg, double place_yaw_deg,
    double clearance_height_mm = 50.0
  );
  
  bool moveToHome();
  bool moveToSafePosition();
  void setSmoothMotion(bool enable = true);
  void printCurrentPose();
  
  // --- [NEW] SHUTDOWN HANDLING ---
  /**
   * @brief Halts any ongoing robot motion. Intended for use on shutdown.
   */
  void stop();

  void enableWorkspaceVisualization(bool enable = true);
  void printWorkspaceLimits();

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::unique_ptr<RobotOperation> robot_operation_;
  
  PlannerConfiguration config_;
  bool initialized_ = false;
  
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  
  bool setupCollisionScene();
  bool addWorkspaceConstraints();
  void publishWorkspaceBoundary();
  
  bool validatePoseInWorkspace(const geometry_msgs::msg::Pose& pose);
};

} // namespace trajectory_plan

#endif // TRAJECTORY_PLANNER_HPP
