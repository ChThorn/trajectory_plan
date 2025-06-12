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
};

struct OperationMetrics {
    std::chrono::steady_clock::time_point start_time;
    std::chrono::duration<double> planning_time{0};
    std::chrono::duration<double> execution_time{0};
    size_t recovery_attempts = 0;
    bool success = false;
    std::string failure_reason;
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
};

} // namespace trajectory_plan

#endif // ROBOT_OPERATION_HPP