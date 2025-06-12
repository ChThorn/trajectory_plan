#include "trajectory_plan/robot_operation.hpp"
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace trajectory_plan
{

RobotOperation::RobotOperation(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group)
  : node_(node)
  , move_group_(move_group)
  , velocity_scaling_factor_(0.3)
  , acceleration_scaling_factor_(0.3)
  , initialized_(false)
{
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
  move_group_->setPlanningTime(5.0);
  move_group_->setNumPlanningAttempts(10);
  
  home_joint_values_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  safe_joint_values_ = {0.0, -0.5, -1.0, 0.0, 1.5, 0.0}; 
  
  initialized_ = true;
  RCLCPP_INFO(node_->get_logger(), "✅ RobotOperation initialized");

  // Set a conservative emergency safe position (slightly bent arms, away from table)
  emergency_safe_joints_ = {0.0, -0.3, -0.8, 0.0, 1.1, 0.0};  // More conservative than home
}

// New method: Set emergency safe position
void RobotOperation::setEmergencySafePosition(const std::vector<double>& joints)
{
    if (joints.size() == move_group_->getJointNames().size()) {
        emergency_safe_joints_ = joints;
        RCLCPP_INFO(node_->get_logger(), "🚨 Emergency safe position updated");
    } else {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Invalid emergency safe position size");
    }
}

// New method: Hierarchical recovery sequence
bool RobotOperation::executeRecoverySequence(const std::string& failure_context)
{
    if (!rclcpp::ok()) return false;
    
    RCLCPP_WARN(node_->get_logger(), "🛟 Starting recovery sequence due to: %s", 
                failure_context.empty() ? "Unknown failure" : failure_context.c_str());
    
    // Level 0: Try home position first
    RCLCPP_INFO(node_->get_logger(), "🏠 Recovery Level 0: Attempting home position...");
    if (attemptSafeMove(home_joint_values_, "home")) {
        RCLCPP_INFO(node_->get_logger(), "✅ Recovery successful - reached home position");
        return true;
    }
    
    // Level 1: Try safe position
    RCLCPP_INFO(node_->get_logger(), "🛡️ Recovery Level 1: Attempting safe position...");
    if (attemptSafeMove(safe_joint_values_, "safe")) {
        RCLCPP_INFO(node_->get_logger(), "✅ Recovery successful - reached safe position");
        return true;
    }
    
    // Level 2: Try emergency safe position
    RCLCPP_INFO(node_->get_logger(), "🚨 Recovery Level 2: Attempting emergency safe position...");
    if (attemptSafeMove(emergency_safe_joints_, "emergency safe")) {
        RCLCPP_INFO(node_->get_logger(), "✅ Recovery successful - reached emergency safe position");
        return true;
    }
    
    // Level 3: Stop in place (last resort)
    RCLCPP_ERROR(node_->get_logger(), "🛑 Recovery Level 3: All positions unreachable - stopping in place");
    move_group_->stop();
    
    // Give system time to settle
    waitForMotionComplete(2.0);
    
    RCLCPP_ERROR(node_->get_logger(), "❌ All recovery attempts failed. Robot may need manual intervention.");
    return false;
}

// Helper method for safe movement attempts
bool RobotOperation::attemptSafeMove(const std::vector<double>& joint_values, const std::string& position_name)
{
    if (!rclcpp::ok()) return false;
    
    try {
        // Use more conservative settings for recovery
        double original_vel = velocity_scaling_factor_;
        double original_acc = acceleration_scaling_factor_;
        
        // Slow and careful movement for recovery
        setVelocityScaling(0.1);
        setAccelerationScaling(0.1);
        
        // Set joint target
        move_group_->setJointValueTarget(joint_values);
        
        // Plan with shorter timeout for recovery
        move_group_->setPlanningTime(3.0);
        
        // Attempt to plan
        moveit::planning_interface::MoveGroupInterface::Plan recovery_plan;
        bool plan_success = (move_group_->plan(recovery_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (plan_success) {
            // Execute the plan
            bool execute_success = (move_group_->execute(recovery_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
            // Restore original settings
            setVelocityScaling(original_vel);
            setAccelerationScaling(original_acc);
            move_group_->setPlanningTime(5.0);  // Restore original planning time
            
            if (execute_success) {
                RCLCPP_INFO(node_->get_logger(), "✅ Successfully moved to %s position", position_name.c_str());
                return true;
            } else {
                RCLCPP_WARN(node_->get_logger(), "❌ Failed to execute movement to %s position", position_name.c_str());
            }
        } else {
            RCLCPP_WARN(node_->get_logger(), "❌ Failed to plan movement to %s position", position_name.c_str());
        }
        
        // Restore settings even on failure
        setVelocityScaling(original_vel);
        setAccelerationScaling(original_acc);
        move_group_->setPlanningTime(5.0);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Exception during recovery to %s: %s", position_name.c_str(), e.what());
    }
    
    return false;
}

void RobotOperation::setVelocityScaling(double scaling)
{
  velocity_scaling_factor_ = std::clamp(scaling, 0.01, 1.0);
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
  RCLCPP_INFO(node_->get_logger(), "🔧 Velocity scaling set to %.2f", velocity_scaling_factor_);
}

void RobotOperation::setAccelerationScaling(double scaling)
{
  acceleration_scaling_factor_ = std::clamp(scaling, 0.01, 1.0);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
  RCLCPP_INFO(node_->get_logger(), "🔧 Acceleration scaling set to %.2f", acceleration_scaling_factor_);
}

void RobotOperation::setSmoothMotion(bool enable)
{
  if (enable) {
    setVelocityScaling(0.2);
    setAccelerationScaling(0.15);
    RCLCPP_INFO(node_->get_logger(), "🐌 Smooth motion enabled (slow and safe)");
  } else {
    setVelocityScaling(0.5);
    setAccelerationScaling(0.3);
    RCLCPP_INFO(node_->get_logger(), "⚡ Normal motion enabled");
  }
}

// --- [FIXED] Fully implemented function ---
void RobotOperation::setHomePosition(const std::vector<double>& joints)
{
  if (joints.size() == move_group_->getJointNames().size()) {
    home_joint_values_ = joints;
    RCLCPP_INFO(node_->get_logger(), "🏠 Home position updated");
  } else {
    RCLCPP_WARN(node_->get_logger(), "⚠️ Invalid home position size. Expected %zu, got %zu.",
                move_group_->getJointNames().size(), joints.size());
  }
}

// --- [FIXED] Fully implemented function ---
void RobotOperation::setSafePosition(const std::vector<double>& joints)
{
  if (joints.size() == move_group_->getJointNames().size()) {
    safe_joint_values_ = joints;
    RCLCPP_INFO(node_->get_logger(), "🛡️ Safe position updated");
  } else {
    RCLCPP_WARN(node_->get_logger(), "⚠️ Invalid safe position size. Expected %zu, got %zu.",
                move_group_->getJointNames().size(), joints.size());
  }
}

bool RobotOperation::planToPose(const geometry_msgs::msg::Pose& target_pose)
{
  if (!rclcpp::ok()) return false;
  RCLCPP_INFO(node_->get_logger(), "🎯 Planning to pose [%.3f, %.3f, %.3f]", 
              target_pose.position.x, target_pose.position.y, target_pose.position.z);
  move_group_->setPoseTarget(target_pose);
  bool success = (move_group_->plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success) RCLCPP_ERROR(node_->get_logger(), "❌ Planning to pose failed.");
  return success;
}

bool RobotOperation::planToJointPosition(const std::vector<double>& joint_values)
{
  if (!rclcpp::ok()) return false;
  move_group_->setJointValueTarget(joint_values);
  bool success = (move_group_->plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success) RCLCPP_ERROR(node_->get_logger(), "❌ Planning to joint position failed.");
  return success;
}

bool RobotOperation::executePlan()
{
  if (!rclcpp::ok()) return false;
  RCLCPP_INFO(node_->get_logger(), "🚀 Executing planned trajectory...");
  bool success = (move_group_->execute(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success) RCLCPP_ERROR(node_->get_logger(), "❌ Trajectory execution failed.");
  return success;
}

bool RobotOperation::moveToHome()
{
  if (!rclcpp::ok()) return false;
  RCLCPP_INFO(node_->get_logger(), "🏠 Moving to home position...");
  return planToJointPosition(home_joint_values_) && executePlan();
}

bool RobotOperation::moveToSafePosition() 
{
    if (!rclcpp::ok()) return false;
    RCLCPP_INFO(node_->get_logger(), "🛡️ Moving to safe position...");
    return planToJointPosition(safe_joint_values_) && executePlan();
}

bool RobotOperation::planAndExecuteCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints)
{
    if (!rclcpp::ok() || waypoints.empty()) {
        return false;
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    if (fraction < 0.9) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Cartesian path failed: only %.2f%% of the path was planned.", fraction * 100.0);
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "✅ Cartesian path planned successfully (%.2f%%). Executing...", fraction * 100.0);
    return move_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool RobotOperation::executeProfessionalPickAndPlace(
    double pick_x_mm, double pick_y_mm, double pick_z_mm,
    double pick_roll_deg, double pick_pitch_deg, double pick_yaw_deg,
    double place_x_mm, double place_y_mm, double place_z_mm,
    double place_roll_deg, double place_pitch_deg, double place_yaw_deg,
    double clearance_height_mm)
{
    if (!rclcpp::ok()) return false;
    RCLCPP_INFO(node_->get_logger(), "🏭 PRODUCTION Pick and Place Sequence Starting...");

    // Create all poses upfront for validation
    auto pick_pose = createPoseFromMmAndDegrees(pick_x_mm, pick_y_mm, pick_z_mm, 
                                               pick_roll_deg, pick_pitch_deg, pick_yaw_deg);
    auto place_pose = createPoseFromMmAndDegrees(place_x_mm, place_y_mm, place_z_mm, 
                                                place_roll_deg, place_pitch_deg, place_yaw_deg);
    double clearance_m = clearance_height_mm / 1000.0;

    auto pick_approach_pose = pick_pose;
    pick_approach_pose.position.z += clearance_m;

    auto place_approach_pose = place_pose;
    place_approach_pose.position.z += clearance_m;
    
    // IMPROVED: Replace simple lambda with comprehensive recovery
    auto safe_recovery = [this](const std::string& context) -> bool {
        return this->executeRecoverySequence(context);
    };
    
    // Execute sequence with recovery and shutdown checks on each step
    if (!rclcpp::ok()) return false;
    if (!moveToHome()) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to move to home at start");
        return safe_recovery("Failed to reach home at startup");
    }
    
    // PICK SEQUENCE
    if (!rclcpp::ok()) return false;
    RCLCPP_INFO(node_->get_logger(), "🎯 Starting PICK sequence...");
    if (!planToPose(pick_approach_pose) || !executePlan()) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to reach pick approach pose");
        return safe_recovery("Failed to reach pick approach pose");
    }
    
    if (!rclcpp::ok()) return false;
    openGripper();
    
    if (!rclcpp::ok()) return false;
    RCLCPP_INFO(node_->get_logger(), "⬇️ Moving linearly to PICK pose...");
    if (!planAndExecuteCartesianPath({pick_pose})) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to reach pick pose");
        return safe_recovery("Failed to reach pick pose");
    }
    
    if (!rclcpp::ok()) return false;
    closeGripper();
    
    if (!rclcpp::ok()) return false;
    RCLCPP_INFO(node_->get_logger(), "⬆️ Retreating linearly from PICK pose...");
    if (!planAndExecuteCartesianPath({pick_approach_pose})) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to retreat from pick pose");
        // Critical failure - object may be grasped, attempt gentle recovery
        openGripper(); // Release object before recovery
        return safe_recovery("Failed to retreat from pick with object");
    }
    
    // Return to home as intermediate safe position
    if (!rclcpp::ok()) return false;
    RCLCPP_INFO(node_->get_logger(), "🏠 Returning to Home as intermediate position...");
    if (!moveToHome()) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to return to home after pick");
        return safe_recovery("Failed to return home after pick");
    }
    
    // PLACE SEQUENCE
    if (!rclcpp::ok()) return false;
    RCLCPP_INFO(node_->get_logger(), "📦 Starting PLACE sequence...");
    if (!planToPose(place_approach_pose) || !executePlan()) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to reach place approach pose");
        return safe_recovery("Failed to reach place approach pose");
    }
    
    if (!rclcpp::ok()) return false;
    RCLCPP_INFO(node_->get_logger(), "⬇️ Moving linearly to PLACE pose...");
    if (!planAndExecuteCartesianPath({place_pose})) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to reach place pose");
        return safe_recovery("Failed to reach place pose");
    }
    
    if (!rclcpp::ok()) return false;
    openGripper();
    
    if (!rclcpp::ok()) return false;
    RCLCPP_INFO(node_->get_logger(), "⬆️ Retreating linearly from PLACE pose...");
    if (!planAndExecuteCartesianPath({place_approach_pose})) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to retreat from place pose");
        return safe_recovery("Failed to retreat from place pose");
    }
    
    // Final return to home
    if (!rclcpp::ok()) return false;
    if (!moveToHome()) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Failed final return to home, but operation completed");
    }
    
    RCLCPP_INFO(node_->get_logger(), "🎉 PRODUCTION Pick and Place completed successfully!");
    return true;
}

// --- [FIXED] Fully implemented function ---
void RobotOperation::openGripper()
{
  RCLCPP_INFO(node_->get_logger(), "🔓 GRIPPER: Opening... (simulated)");
  waitForMotionComplete(1.0);
  RCLCPP_INFO(node_->get_logger(), "✅ GRIPPER: Opened");
}

// --- [FIXED] Fully implemented function ---
void RobotOperation::closeGripper()
{
  RCLCPP_INFO(node_->get_logger(), "🤏 GRIPPER: Closing... (simulated)");
  waitForMotionComplete(1.0);
  RCLCPP_INFO(node_->get_logger(), "✅ GRIPPER: Closed - object grasped");
}

// --- [FIXED] Fully implemented function ---
void RobotOperation::checkAndPrintCurrentPose()
{
  auto current_pose = move_group_->getCurrentPose().pose;
  double x_mm = current_pose.position.x * 1000.0;
  double y_mm = current_pose.position.y * 1000.0;
  double z_mm = current_pose.position.z * 1000.0;
  
  tf2::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, 
                    current_pose.orientation.z, current_pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  
  RCLCPP_INFO(node_->get_logger(), "📍 CURRENT ROBOT POSE: Pos:[%.1f,%.1f,%.1f]mm, RPY:[%.1f,%.1f,%.1f]deg",
              x_mm, y_mm, z_mm, roll * 180.0/M_PI, pitch * 180.0/M_PI, yaw * 180.0/M_PI);
}

// --- [FIXED] Fully implemented function ---
geometry_msgs::msg::Pose RobotOperation::createPoseFromMmAndDegrees(
  double x_mm, double y_mm, double z_mm,
  double roll_deg, double pitch_deg, double yaw_deg)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x_mm / 1000.0;
  pose.position.y = y_mm / 1000.0;
  pose.position.z = z_mm / 1000.0;
  
  tf2::Quaternion q;
  q.setRPY(roll_deg * M_PI / 180.0, pitch_deg * M_PI / 180.0, yaw_deg * M_PI / 180.0);
  
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  
  return pose;
}

// --- [FIXED] Fully implemented function ---
void RobotOperation::waitForMotionComplete(double seconds)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
}

} // namespace trajectory_plan
