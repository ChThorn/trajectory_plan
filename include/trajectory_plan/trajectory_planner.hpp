#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include "robot_operation.hpp"
#include <atomic>
#include <mutex>

namespace trajectory_plan
{

struct PlannerConfiguration
{
    struct {
        double length = 0.98, width = 0.49, height = 0.04;
        double x_offset = 0.43, y_offset = 0.0, z_position = -0.023;
    } table;
    
    struct {
        double width = 1.0, depth = 1.0, height = 1.0;
        double x_position = 0.25, y_position = 0.0, z_position = 0.5;
        bool visualization_enabled = false;
    } workspace;
    
    struct {
        double velocity_scaling = 0.3, acceleration_scaling = 0.3;
        double planning_time = 5.0;           // Configurable planning time
        int planning_attempts = 10;           // Configurable planning attempts
        double cartesian_step_size = 0.01;   // Configurable Cartesian step size
        double cartesian_jump_threshold = 0.0; // Configurable jump threshold
    } robot;
    
    struct {
        double gripper_action_time = 1.0;     // Configurable gripper timing
        std::string manipulation_group = "mainpulation"; // Configurable group name
    } gripper;
};

/**
 * @class TrajectoryPlanner
 * @brief Production-ready trajectory planner with enhanced safety and error handling.
 *
 * This class provides a high-level interface for robot trajectory planning and execution,
 * integrating workspace constraints, collision avoidance, and comprehensive error recovery.
 */
class TrajectoryPlanner
{
public:
    explicit TrajectoryPlanner(const rclcpp::Node::SharedPtr& node);
    
    ~TrajectoryPlanner() {
        safeShutdown();
    }
    
    // --- Core Interface ---
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
    
    // --- Robot Control ---
    bool moveToHome();
    bool moveToSafePosition();
    void setSmoothMotion(bool enable = true);
    void printCurrentPose();
    
    // --- Safety & Shutdown ---
    void stop();
    bool isOperational() const { return operational_.load(); }
    
    // --- Workspace Management ---
    void enableWorkspaceVisualization(bool enable = true);
    void printWorkspaceLimits();
    
    // --- Access to Robot Operation for Configuration ---
    RobotOperation* getRobotOperation() { 
        std::lock_guard<std::mutex> lock(operation_mutex_);
        return robot_operation_.get(); 
    }

private:
    // --- Core Components ---
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::unique_ptr<RobotOperation> robot_operation_;
    
    // --- Thread Safety ---
    mutable std::mutex operation_mutex_;
    mutable std::mutex visualization_mutex_;
    std::atomic<bool> operational_{false};
    std::atomic<bool> shutdown_requested_{false};
    
    // --- Configuration ---
    PlannerConfiguration config_;
    bool initialized_ = false;
    
    // --- Visualization ---
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr visualization_timer_;
    
    // --- Private Methods ---
    bool setupCollisionScene();
    bool addWorkspaceConstraints();
    void publishWorkspaceBoundary();
    bool validatePoseInWorkspace(const geometry_msgs::msg::Pose& pose);
    void safeShutdown();
    
    // --- Enhanced Validation ---
    bool validateConfiguration() const;
    bool validateWorkspaceConfiguration() const;
    bool validateRobotConfiguration() const;
};

} // namespace trajectory_plan

#endif // TRAJECTORY_PLANNER_HPP