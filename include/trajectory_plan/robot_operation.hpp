#ifndef ROBOT_OPERATION_HPP
#define ROBOT_OPERATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <vector>
#include <atomic>
#include <mutex>
#include <memory>

namespace trajectory_plan
{

// --- [NEW] Enhanced error handling with specific error types ---
class RobotOperationError : public std::runtime_error {
public:
    enum class Type { 
        PLANNING_FAILED, 
        EXECUTION_FAILED, 
        WORKSPACE_VIOLATION,
        JOINT_LIMIT_VIOLATION,
        SAFETY_VIOLATION,
        SHUTDOWN_REQUESTED
    };
    
    RobotOperationError(Type type, const std::string& msg) 
        : std::runtime_error(msg), error_type_(type) {}
    Type getType() const { return error_type_; }
private:
    Type error_type_;
};

// --- [NEW] Safety and performance configuration ---
struct SafetyConfiguration {
    std::vector<double> home_joints;
    std::vector<double> safe_joints;
    std::vector<double> emergency_joints;
    
    double max_joint_velocity = 1.0;          // rad/s
    double max_joint_acceleration = 2.0;      // rad/s²
    double safety_stop_timeout = 2.0;         // seconds
    double planning_timeout = 5.0;            // seconds
    int max_planning_attempts = 10;
    double cartesian_step_size = 0.01;        // meters
    double cartesian_jump_threshold = 0.0;    // disabled
    
    bool require_manual_recovery = false;
    bool validate_joint_limits = true;
    bool validate_reachability = true;

    // === NEW: Adaptive timeout configuration ===
    struct {
        double home_position_timeout = 15.0;      // Reduced for simple moves
        double approach_planning_timeout = 30.0;   // Reduced from 60s
        double cartesian_planning_timeout = 30.0;  // Reduced from 60s
        double complex_planning_timeout = 60.0;    // For genuinely complex moves
        bool adaptive_timeouts = true;             // Enable adaptive timeout scaling
    } timeouts;
    
    // === NEW: Performance optimization ===
    struct {
        bool fast_first_attempt = true;      // Optimize first attempts
        double retry_delay_ms = 50.0;        // Reduced retry delay
        double progress_log_interval = 5.0;  // Log progress every 5s
        bool immediate_exit_on_success = true; // Exit immediately on success
    } performance;
};

struct OperationMetrics {
    std::chrono::steady_clock::time_point start_time;
    std::chrono::duration<double> planning_time{0};
    std::chrono::duration<double> execution_time{0};
    std::chrono::duration<double> total_time{0};
    size_t recovery_attempts = 0;
    bool success = false;
    std::string failure_reason;
    
    // NEW: Enhanced metrics
    std::vector<std::string> completed_phases;
    std::vector<std::string> failed_operations;
    std::map<std::string, int> retry_counts;
    double gripper_time = 0.0;
    double cartesian_path_efficiency = 0.0; // % of path successfully planned
};

/**
 * @class RobotOperation
 * @brief Production-ready robot motion primitives with enhanced safety and error handling.
 */
class RobotOperation
{
public:
    RobotOperation(const rclcpp::Node::SharedPtr& node,
                  const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group);
    
    ~RobotOperation();
    
    // --- Configuration ---
    void updateSafetyConfiguration(const SafetyConfiguration& config);
    void setVelocityScaling(double scaling);
    void setAccelerationScaling(double scaling);
    void setSmoothMotion(bool enable = true);
    
    // --- Enhanced safety positions with validation ---
    bool setHomePosition(const std::vector<double>& joints);
    bool setSafePosition(const std::vector<double>& joints);
    bool setEmergencySafePosition(const std::vector<double>& joints);
    
    // --- Core Motion Primitives with enhanced validation ---
    bool planToPose(const geometry_msgs::msg::Pose& target_pose);
    bool planToJointPosition(const std::vector<double>& joint_values);
    bool executePlan();
    
    // --- High-Level Sequences ---
    bool moveToHome();
    bool moveToSafePosition();
    
    bool executeProfessionalPickAndPlace(
        double pick_x_mm, double pick_y_mm, double pick_z_mm,
        double pick_roll_deg, double pick_pitch_deg, double pick_yaw_deg,
        double place_x_mm, double place_y_mm, double place_z_mm,
        double place_roll_deg, double place_pitch_deg, double place_yaw_deg,
        double clearance_height_mm = 50.0
    );
    
    // --- Gripper Control ---
    void openGripper();
    void closeGripper();
    
    // --- Enhanced Safety & Recovery ---
    bool executeRecoverySequence(const std::string& failure_context = "");
    void emergencyStop();
    bool isOperational() const { return operational_.load(); }
    
    // --- Utilities ---
    void checkAndPrintCurrentPose();
    void waitForMotionComplete(double seconds = 1.0);
    geometry_msgs::msg::Pose createPoseFromMmAndDegrees(
        double x_mm, double y_mm, double z_mm,
        double roll_deg, double pitch_deg, double yaw_deg
    );
    
    // --- Metrics ---
    OperationMetrics getLastOperationMetrics() const { return last_metrics_; }
    void resetMetrics();

    // --- Enhanced Planning with Retry Logic ---
    bool planWithRetry(const geometry_msgs::msg::Pose& target_pose, 
        const std::string& phase_name,
        std::chrono::seconds timeout = std::chrono::seconds(60));

    bool planToJointWithRetry(const std::vector<double>& joint_values,
                const std::string& phase_name, 
                std::chrono::seconds timeout = std::chrono::seconds(60));

    // --- Enhanced Cartesian Planning with Retry Logic ---
    bool planAndExecuteCartesianPathWithRetry(
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        const std::string& phase_name,
        std::chrono::seconds timeout = std::chrono::seconds(60),
        double min_success_fraction = 0.9
    );

private:
    // --- Core Components ---
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    
    // --- Thread Safety ---
    mutable std::mutex operation_mutex_;
    std::atomic<bool> operational_{true};
    std::atomic<bool> emergency_stop_requested_{false};
    
    // --- Configuration ---
    SafetyConfiguration safety_config_;
    double velocity_scaling_factor_;
    double acceleration_scaling_factor_;
    
    // --- State ---
    bool initialized_;
    moveit::planning_interface::MoveGroupInterface::Plan current_plan_;
    OperationMetrics last_metrics_;
    
    // --- Enhanced Validation Methods ---
    bool validateJointPosition(const std::vector<double>& joint_values, const std::string& position_name);
    bool validatePoseReachability(const geometry_msgs::msg::Pose& pose);
    bool validateJointLimits(const std::vector<double>& joint_values);
    bool checkEmergencyStop();
    
    // --- Private Motion Methods ---
    bool planAndExecuteCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints);
    bool attemptSafeMove(const std::vector<double>& joint_values, const std::string& position_name);
    
    // --- Metrics Helpers ---
    void startMetrics();
    void recordPlanningTime();
    void recordExecutionTime();
    void recordFailure(const std::string& reason);
    void recordPhaseCompletion(const std::string& phase);
    void recordOperationFailure(const std::string& operation);
    void recordRetry(const std::string& operation);

    // --- Retry Planning Helpers ---
    struct RetryStatistics {
        int total_attempts = 0;
        int successful_attempts = 0;
        std::chrono::duration<double> total_planning_time{0};
        std::chrono::steady_clock::time_point start_time;
        bool timeout_reached = false;
    };
    
    bool attemptPlanning(const geometry_msgs::msg::Pose& target_pose, RetryStatistics& stats);
    bool attemptJointPlanning(const std::vector<double>& joint_values, RetryStatistics& stats);
    void logRetryProgress(const std::string& phase_name, const RetryStatistics& stats, 
                         std::chrono::seconds timeout);

    // --- Cartesian Retry Helpers ---
    struct CartesianRetryStatistics {
        int total_attempts = 0;
        int successful_attempts = 0;
        double best_fraction = 0.0;
        std::chrono::duration<double> total_planning_time{0};
        std::chrono::steady_clock::time_point start_time;
        bool timeout_reached = false;
    };
    
    bool attemptCartesianPlanning(
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        CartesianRetryStatistics& stats,
        double min_success_fraction,
        moveit_msgs::msg::RobotTrajectory& best_trajectory
    );
    
    void logCartesianRetryProgress(const std::string& phase_name, 
                                  const CartesianRetryStatistics& stats, 
                                  std::chrono::seconds timeout);

    bool executeCartesianTrajectory(
                                    const moveit_msgs::msg::RobotTrajectory& trajectory, 
                                    const std::string& phase_name);

    // === ADD THESE NEW COLLISION VALIDATION METHODS ===
    bool validateTrajectoryCollisionFree();
    bool validateCurrentStateCollisionFree();
};

} // namespace trajectory_plan

#endif // ROBOT_OPERATION_HPP