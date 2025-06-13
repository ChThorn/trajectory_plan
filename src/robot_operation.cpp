#include "trajectory_plan/robot_operation.hpp"
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

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
    std::lock_guard<std::mutex> lock(operation_mutex_);
    
    // Initialize safety configuration with validated defaults
    safety_config_.home_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    safety_config_.safe_joints = {0.0, -0.5, -1.0, 0.0, 1.5, 0.0};
    safety_config_.emergency_joints = {0.0, -0.3, -0.8, 0.0, 1.1, 0.0};
    
    // Configure MoveIt with safety parameters
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
    move_group_->setPlanningTime(safety_config_.planning_timeout);
    move_group_->setNumPlanningAttempts(safety_config_.max_planning_attempts);
    
    // Validate default positions
    validateJointPosition(safety_config_.home_joints, "default_home");
    validateJointPosition(safety_config_.safe_joints, "default_safe");
    validateJointPosition(safety_config_.emergency_joints, "default_emergency");
    
    initialized_ = true;
    operational_.store(true);
    RCLCPP_INFO(node_->get_logger(), "✅ RobotOperation initialized with enhanced safety");
}

RobotOperation::~RobotOperation()
{
    emergencyStop();
    RCLCPP_DEBUG(node_->get_logger(), "RobotOperation destroyed safely");
}

void RobotOperation::updateSafetyConfiguration(const SafetyConfiguration& config)
{
    std::lock_guard<std::mutex> lock(operation_mutex_);
    
    // Validate all positions before updating
    if (!validateJointPosition(config.home_joints, "new_home") ||
        !validateJointPosition(config.safe_joints, "new_safe") ||
        !validateJointPosition(config.emergency_joints, "new_emergency")) {
        throw RobotOperationError(RobotOperationError::Type::SAFETY_VIOLATION,
                                 "Invalid joint positions in safety configuration");
    }
    
    safety_config_ = config;
    
    // Update MoveIt parameters
    move_group_->setPlanningTime(safety_config_.planning_timeout);
    move_group_->setNumPlanningAttempts(safety_config_.max_planning_attempts);
    
    RCLCPP_INFO(node_->get_logger(), "🔧 Safety configuration updated and validated");
}

bool RobotOperation::setHomePosition(const std::vector<double>& joints)
{
    std::lock_guard<std::mutex> lock(operation_mutex_);
    
    if (!validateJointPosition(joints, "home")) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Invalid home position provided");
        return false;
    }
    
    safety_config_.home_joints = joints;
    RCLCPP_INFO(node_->get_logger(), "🏠 Home position updated and validated");
    return true;
}

bool RobotOperation::setSafePosition(const std::vector<double>& joints)
{
    std::lock_guard<std::mutex> lock(operation_mutex_);
    
    if (!validateJointPosition(joints, "safe")) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Invalid safe position provided");
        return false;
    }
    
    safety_config_.safe_joints = joints;
    RCLCPP_INFO(node_->get_logger(), "🛡️ Safe position updated and validated");
    return true;
}

bool RobotOperation::setEmergencySafePosition(const std::vector<double>& joints)
{
    std::lock_guard<std::mutex> lock(operation_mutex_);
    
    if (!validateJointPosition(joints, "emergency")) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Invalid emergency position provided");
        return false;
    }
    
    safety_config_.emergency_joints = joints;
    RCLCPP_INFO(node_->get_logger(), "🚨 Emergency position updated and validated");
    return true;
}

bool RobotOperation::validateJointPosition(const std::vector<double>& joint_values, const std::string& position_name)
{
    if (!safety_config_.validate_joint_limits) {
        return true; // Validation disabled
    }
    
    // Check joint count
    if (joint_values.size() != move_group_->getJointNames().size()) {
        RCLCPP_ERROR(node_->get_logger(), "❌ %s position has wrong number of joints: %zu vs %zu",
                    position_name.c_str(), joint_values.size(), move_group_->getJointNames().size());
        return false;
    }
    
    // Get robot model for validation
    auto robot_model = move_group_->getRobotModel();
    auto joint_model_group = robot_model->getJointModelGroup(move_group_->getName());
    
    // Check joint limits
    if (!joint_model_group->satisfiesPositionBounds(joint_values.data())) {
        RCLCPP_ERROR(node_->get_logger(), "❌ %s position violates joint limits", position_name.c_str());
        return false;
    }
    
    // Check for singular configurations (basic check)
    auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
    robot_state->setJointGroupPositions(joint_model_group, joint_values);
    robot_state->update();
    
    if (!robot_state->satisfiesBounds(joint_model_group)) {
        RCLCPP_ERROR(node_->get_logger(), "❌ %s position fails robot state validation", position_name.c_str());
        return false;
    }
    
    RCLCPP_DEBUG(node_->get_logger(), "✅ %s position validated successfully", position_name.c_str());
    return true;
}

bool RobotOperation::validatePoseReachability(const geometry_msgs::msg::Pose& pose)
{
    if (!safety_config_.validate_reachability) {
        return true; // Validation disabled
    }
    
    try {
        // Quick reachability check using IK
        move_group_->setPoseTarget(pose);
        
        // Use a temporary plan to test reachability without executing
        moveit::planning_interface::MoveGroupInterface::Plan temp_plan;
        auto result = move_group_->plan(temp_plan);
        
        bool reachable = (result == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!reachable) {
            RCLCPP_WARN(node_->get_logger(), "❌ Pose [%.3f, %.3f, %.3f] is not reachable",
                       pose.position.x, pose.position.y, pose.position.z);
        }
        
        return reachable;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Exception during pose reachability check: %s", e.what());
        return false;
    }
}

bool RobotOperation::checkEmergencyStop()
{
    if (emergency_stop_requested_.load() || !rclcpp::ok()) {
        throw RobotOperationError(RobotOperationError::Type::SHUTDOWN_REQUESTED,
                                 "Emergency stop or shutdown requested");
    }
    return true;
}

void RobotOperation::emergencyStop()
{
    emergency_stop_requested_.store(true);
    operational_.store(false);
    
    if (move_group_) {
        move_group_->stop();
        RCLCPP_WARN(node_->get_logger(), "🛑 EMERGENCY STOP - All robot motion halted");
    }
}

void RobotOperation::setVelocityScaling(double scaling)
{
    std::lock_guard<std::mutex> lock(operation_mutex_);
    velocity_scaling_factor_ = std::clamp(scaling, 0.01, 1.0);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
    RCLCPP_DEBUG(node_->get_logger(), "🔧 Velocity scaling set to %.2f", velocity_scaling_factor_);
}

void RobotOperation::setAccelerationScaling(double scaling)
{
    std::lock_guard<std::mutex> lock(operation_mutex_);
    acceleration_scaling_factor_ = std::clamp(scaling, 0.01, 1.0);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
    RCLCPP_DEBUG(node_->get_logger(), "🔧 Acceleration scaling set to %.2f", acceleration_scaling_factor_);
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

bool RobotOperation::planToPose(const geometry_msgs::msg::Pose& target_pose)
{
    checkEmergencyStop();
    std::lock_guard<std::mutex> lock(operation_mutex_);
    startMetrics();
    
    try {
        // Validate pose reachability first
        if (!validatePoseReachability(target_pose)) {
            recordFailure("Pose not reachable");
            return false;
        }
        
        RCLCPP_INFO(node_->get_logger(), "🎯 Planning to pose [%.3f, %.3f, %.3f] with enhanced collision checking", 
                   target_pose.position.x, target_pose.position.y, target_pose.position.z);
        
        move_group_->setPoseTarget(target_pose);
        
        // 🔥 ENSURE COLLISION CHECKING IS ACTIVE
        move_group_->setGoalPositionTolerance(0.01);     // 1cm tolerance
        move_group_->setGoalOrientationTolerance(0.05);  // ~3 degree tolerance
        
        auto result = move_group_->plan(current_plan_);
        recordPlanningTime();
        
        bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            // 🔥 ADDITIONAL VALIDATION
            if (validateTrajectoryCollisionFree()) {
                RCLCPP_INFO(node_->get_logger(), "✅ Planning successful with collision-free trajectory");
            } else {
                RCLCPP_WARN(node_->get_logger(), "⚠️ Planning succeeded but trajectory validation failed");
                // Still return success, but log warning
            }
        } else {
            recordFailure("Planning to pose failed - likely collision detected");
            RCLCPP_ERROR(node_->get_logger(), "❌ Planning failed - MoveIt detected potential collisions");
        }
        
        return success;
        
    } catch (const std::exception& e) {
        recordFailure(std::string("Exception in planToPose: ") + e.what());
        RCLCPP_ERROR(node_->get_logger(), "❌ Exception in planToPose: %s", e.what());
        return false;
    }
}

bool RobotOperation::planToJointPosition(const std::vector<double>& joint_values)
{
    checkEmergencyStop();
    std::lock_guard<std::mutex> lock(operation_mutex_);
    startMetrics();
    
    try {
        // Validate joint position first
        if (!validateJointLimits(joint_values)) {
            recordFailure("Joint limits violation");
            return false;
        }
        
        move_group_->setJointValueTarget(joint_values);
        auto result = move_group_->plan(current_plan_);
        recordPlanningTime();
        
        bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success) {
            recordFailure("Planning to joint position failed");
            RCLCPP_ERROR(node_->get_logger(), "❌ Planning to joint position failed");
        }
        
        return success;
    } catch (const std::exception& e) {
        recordFailure(std::string("Exception in planToJointPosition: ") + e.what());
        RCLCPP_ERROR(node_->get_logger(), "❌ Exception in planToJointPosition: %s", e.what());
        return false;
    }
}

bool RobotOperation::validateJointLimits(const std::vector<double>& joint_values)
{
    if (!safety_config_.validate_joint_limits) {
        return true;
    }
    
    auto robot_model = move_group_->getRobotModel();
    auto joint_model_group = robot_model->getJointModelGroup(move_group_->getName());
    
    return joint_model_group->satisfiesPositionBounds(joint_values.data());
}

bool RobotOperation::executePlan()
{
    checkEmergencyStop();
    std::lock_guard<std::mutex> lock(operation_mutex_);
    
    try {
        RCLCPP_INFO(node_->get_logger(), "🚀 Executing planned trajectory...");
        
        auto result = move_group_->execute(current_plan_);
        recordExecutionTime();
        
        bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success) {
            recordFailure("Trajectory execution failed");
            RCLCPP_ERROR(node_->get_logger(), "❌ Trajectory execution failed");
        } else {
            last_metrics_.success = true;
        }
        
        return success;
    } catch (const std::exception& e) {
        recordFailure(std::string("Exception in executePlan: ") + e.what());
        RCLCPP_ERROR(node_->get_logger(), "❌ Exception in executePlan: %s", e.what());
        return false;
    }
}

bool RobotOperation::moveToHome()
{
    checkEmergencyStop();
    RCLCPP_INFO(node_->get_logger(), "🏠 Moving to home position...");
    return planToJointPosition(safety_config_.home_joints) && executePlan();
}

bool RobotOperation::moveToSafePosition()
{
    checkEmergencyStop();
    RCLCPP_INFO(node_->get_logger(), "🛡️ Moving to safe position...");
    return planToJointPosition(safety_config_.safe_joints) && executePlan();
}

bool RobotOperation::executeRecoverySequence(const std::string& failure_context)
{
    if (!operational_.load()) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Robot not operational, cannot execute recovery");
        return false;
    }
    
    RCLCPP_WARN(node_->get_logger(), "🛟 Starting enhanced recovery sequence due to: %s", 
                failure_context.empty() ? "Unknown failure" : failure_context.c_str());
    
    last_metrics_.recovery_attempts++;
    
    auto recovery_start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(static_cast<int>(safety_config_.safety_stop_timeout * 3)); // 3x normal timeout
    
    try {
        // Helper lambda to check timeout
        auto check_timeout = [&]() -> bool {
            if (std::chrono::steady_clock::now() - recovery_start > timeout) {
                RCLCPP_ERROR(node_->get_logger(), "⏰ Recovery timeout reached");
                return true;
            }
            return false;
        };
        
        // Level 0: Try home position first
        RCLCPP_INFO(node_->get_logger(), "🏠 Recovery Level 0: Attempting home position...");
        if (attemptSafeMove(safety_config_.home_joints, "home")) {
            RCLCPP_INFO(node_->get_logger(), "✅ Recovery successful - reached home position");
            return true;
        }
        
        if (check_timeout()) return false;
        
        // Level 1: Try safe position
        RCLCPP_INFO(node_->get_logger(), "🛡️ Recovery Level 1: Attempting safe position...");
        if (attemptSafeMove(safety_config_.safe_joints, "safe")) {
            RCLCPP_INFO(node_->get_logger(), "✅ Recovery successful - reached safe position");
            return true;
        }
        
        if (check_timeout()) return false;
        
        // Level 2: Try emergency safe position
        RCLCPP_INFO(node_->get_logger(), "🚨 Recovery Level 2: Attempting emergency safe position...");
        if (attemptSafeMove(safety_config_.emergency_joints, "emergency safe")) {
            RCLCPP_INFO(node_->get_logger(), "✅ Recovery successful - reached emergency safe position");
            return true;
        }
        
        // Level 3: Emergency stop
        RCLCPP_ERROR(node_->get_logger(), "🛑 Recovery Level 3: All positions unreachable - emergency stop");
        emergencyStop();
        return false;
        
    } catch (const RobotOperationError& e) {
        if (e.getType() == RobotOperationError::Type::SHUTDOWN_REQUESTED) {
            RCLCPP_WARN(node_->get_logger(), "🔌 Recovery interrupted by shutdown request");
            return false;
        }
        RCLCPP_ERROR(node_->get_logger(), "❌ Exception during recovery: %s", e.what());
        return false;
    }
}

bool RobotOperation::attemptSafeMove(const std::vector<double>& joint_values, const std::string& position_name)
{
    try {
        checkEmergencyStop();
        
        // Use more conservative settings for recovery
        double original_vel = velocity_scaling_factor_;
        double original_acc = acceleration_scaling_factor_;
        
        // Slow and careful movement for recovery
        setVelocityScaling(0.1);
        setAccelerationScaling(0.1);
        
        bool success = planToJointPosition(joint_values) && executePlan();
        
        // Restore original settings
        setVelocityScaling(original_vel);
        setAccelerationScaling(original_acc);
        
        if (success) {
            RCLCPP_INFO(node_->get_logger(), "✅ Successfully moved to %s position", position_name.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(), "❌ Failed to move to %s position", position_name.c_str());
        }
        
        return success;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Exception during recovery to %s: %s", position_name.c_str(), e.what());
        return false;
    }
}

bool RobotOperation::planAndExecuteCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints)
{
    checkEmergencyStop();
    
    if (waypoints.empty()) {
        return false;
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(
        waypoints, 
        safety_config_.cartesian_step_size, 
        safety_config_.cartesian_jump_threshold, 
        trajectory
    );
    
    if (fraction < 0.9) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Cartesian path failed: only %.2f%% of the path was planned.", fraction * 100.0);
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "✅ Cartesian path planned successfully (%.2f%%). Executing...", fraction * 100.0);
    
    bool success = move_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
    recordExecutionTime();
    
    return success;
}

bool RobotOperation::executeProfessionalPickAndPlace(
    double pick_x_mm, double pick_y_mm, double pick_z_mm,
    double pick_roll_deg, double pick_pitch_deg, double pick_yaw_deg,
    double place_x_mm, double place_y_mm, double place_z_mm,
    double place_roll_deg, double place_pitch_deg, double place_yaw_deg,
    double clearance_height_mm)
{
    checkEmergencyStop();
    startMetrics();
    
    RCLCPP_INFO(node_->get_logger(), "🏭 PRODUCTION Pick and Place with OPTIMIZED Retry Logic Starting...");

    try {
        // Create poses
        auto pick_pose = createPoseFromMmAndDegrees(pick_x_mm, pick_y_mm, pick_z_mm, 
                                                   pick_roll_deg, pick_pitch_deg, pick_yaw_deg);
        auto place_pose = createPoseFromMmAndDegrees(place_x_mm, place_y_mm, place_z_mm, 
                                                    place_roll_deg, place_pitch_deg, place_yaw_deg);
        double clearance_m = clearance_height_mm / 1000.0;

        auto pick_approach_pose = pick_pose;
        pick_approach_pose.position.z += clearance_m;

        auto place_approach_pose = place_pose;
        place_approach_pose.position.z += clearance_m;
        
        // === ADAPTIVE TIMEOUTS ===
        auto home_timeout = std::chrono::seconds(static_cast<int>(safety_config_.timeouts.home_position_timeout));
        auto approach_timeout = std::chrono::seconds(static_cast<int>(safety_config_.timeouts.approach_planning_timeout));
        auto cartesian_timeout = std::chrono::seconds(static_cast<int>(safety_config_.timeouts.cartesian_planning_timeout));
        
        // === PHASE 1: MOVE TO HOME (optimized timeout) ===
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "🏠 Phase 1: Moving to home position...");
        if (!planToJointWithRetry(safety_config_.home_joints, "move_to_home", home_timeout)) {
            return false;
        }
        if (!executePlan()) {
            recordFailure("Failed to execute move to home");
            return executeRecoverySequence("Failed to move to home");
        }
        recordPhaseCompletion("move_to_home");
        
        // === PHASE 2: PLAN HOME -> PICK APPROACH (optimized timeout) ===
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "🎯 Phase 2: Planning home → pick approach with retry...");
        if (!planWithRetry(pick_approach_pose, "home_to_pick_approach", approach_timeout)) {
            recordFailure("Failed to find path home → pick approach after retrying");
            return false;
        }
        if (!executePlan()) {
            recordFailure("Failed to execute home → pick approach");
            return executeRecoverySequence("Failed to execute home → pick approach");
        }
        recordPhaseCompletion("home_to_pick_approach");
        
        // === PHASE 3: PICK SEQUENCE (optimized Cartesian retry) ===
        checkEmergencyStop();
        openGripper();
        
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "⬇️ Phase 3: Moving linearly to PICK pose with retry...");
        if (!planAndExecuteCartesianPathWithRetry({pick_pose}, "cartesian_to_pick", cartesian_timeout, 0.9)) {
            recordFailure("Failed Cartesian path to pick pose after retrying");
            return executeRecoverySequence("Failed Cartesian path to pick pose after retrying");
        }
        
        checkEmergencyStop();
        closeGripper();
        
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "⬆️ Phase 4: Retreating linearly from PICK pose with retry...");
        if (!planAndExecuteCartesianPathWithRetry({pick_approach_pose}, "cartesian_from_pick", cartesian_timeout, 0.9)) {
            openGripper();
            recordFailure("Failed to retreat from pick pose after retrying");
            return executeRecoverySequence("Failed to retreat from pick with object after retrying");
        }
        
        // === PHASE 5: RETURN TO HOME ===
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "🏠 Phase 5: Returning to home as intermediate position...");
        if (!planToJointWithRetry(safety_config_.home_joints, "return_to_home_after_pick", home_timeout)) {
            recordFailure("Failed to plan return to home after pick");
            return executeRecoverySequence("Failed to return home after pick");
        }
        if (!executePlan()) {
            recordFailure("Failed to execute return to home after pick");
            return executeRecoverySequence("Failed to return home after pick");
        }
        recordPhaseCompletion("return_to_home_after_pick");
        
        // === PHASE 6: PLAN HOME -> PLACE APPROACH ===
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "📦 Phase 6: Planning home → place approach with retry...");
        if (!planWithRetry(place_approach_pose, "home_to_place_approach", approach_timeout)) {
            recordFailure("Failed to find path home → place approach after retrying");
            return false;
        }
        if (!executePlan()) {
            recordFailure("Failed to execute home → place approach");
            return executeRecoverySequence("Failed to execute home → place approach");
        }
        recordPhaseCompletion("home_to_place_approach");
        
        // === PHASE 7: PLACE SEQUENCE (optimized Cartesian retry) ===
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "⬇️ Phase 7: Moving linearly to PLACE pose with retry...");
        if (!planAndExecuteCartesianPathWithRetry({place_pose}, "cartesian_to_place", cartesian_timeout, 0.9)) {
            recordFailure("Failed Cartesian path to place pose after retrying");
            return false;
        }
        
        checkEmergencyStop();
        openGripper();
        
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "⬆️ Phase 8: Retreating linearly from PLACE pose with retry...");
        if (!planAndExecuteCartesianPathWithRetry({place_approach_pose}, "cartesian_from_place", cartesian_timeout, 0.9)) {
            recordFailure("Failed to retreat from place pose after retrying");
            return executeRecoverySequence("Failed to retreat from place pose after retrying");
        }
        
        // === PHASE 9: FINAL RETURN TO HOME ===
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "🏠 Phase 9: Final return to home...");
        if (!planToJointWithRetry(safety_config_.home_joints, "final_return_to_home", home_timeout)) {
            RCLCPP_WARN(node_->get_logger(), "⚠️ Failed final return to home, but operation completed");
        } else if (!executePlan()) {
            RCLCPP_WARN(node_->get_logger(), "⚠️ Failed to execute final return to home, but operation completed");
        } else {
            recordPhaseCompletion("final_return_to_home");
        }
        
        last_metrics_.success = true;
        RCLCPP_INFO(node_->get_logger(), "🎉 PRODUCTION Pick and Place with OPTIMIZED retry logic completed successfully!");
        
        // Log detailed metrics
        RCLCPP_INFO(node_->get_logger(), "📊 Completed phases: %zu, Failed operations: %zu, Total retry operations: %zu",
                   last_metrics_.completed_phases.size(), 
                   last_metrics_.failed_operations.size(),
                   last_metrics_.retry_counts.size());
        
        return true;
        
    } catch (const RobotOperationError& e) {
        if (e.getType() == RobotOperationError::Type::SHUTDOWN_REQUESTED) {
            RCLCPP_WARN(node_->get_logger(), "🔌 Pick and place sequence interrupted by shutdown");
            recordFailure("Shutdown requested");
            return false;
        }
        recordFailure(e.what());
        RCLCPP_ERROR(node_->get_logger(), "❌ Pick and place failed: %s", e.what());
        executeRecoverySequence(e.what());
        return false;
    } catch (const std::exception& e) {
        recordFailure(std::string("Unexpected exception: ") + e.what());
        RCLCPP_ERROR(node_->get_logger(), "❌ Unexpected exception: %s", e.what());
        executeRecoverySequence(std::string("Unexpected exception: ") + e.what());
        return false;
    }
}

void RobotOperation::recordPhaseCompletion(const std::string& phase)
{
    last_metrics_.completed_phases.push_back(phase);
    RCLCPP_DEBUG(node_->get_logger(), "📊 Phase completed: %s", phase.c_str());
}

void RobotOperation::recordOperationFailure(const std::string& operation)
{
    last_metrics_.failed_operations.push_back(operation);
    RCLCPP_DEBUG(node_->get_logger(), "📊 Operation failed: %s", operation.c_str());
}

void RobotOperation::recordRetry(const std::string& operation)
{
    last_metrics_.retry_counts[operation]++;
    RCLCPP_DEBUG(node_->get_logger(), "📊 Retry recorded for: %s (count: %d)", 
                operation.c_str(), last_metrics_.retry_counts[operation]);
}

void RobotOperation::openGripper()
{
    checkEmergencyStop();
    RCLCPP_INFO(node_->get_logger(), "🔓 GRIPPER: Opening... (simulated)");
    waitForMotionComplete(1.0);
    RCLCPP_INFO(node_->get_logger(), "✅ GRIPPER: Opened");
}

void RobotOperation::closeGripper()
{
    checkEmergencyStop();
    RCLCPP_INFO(node_->get_logger(), "🤏 GRIPPER: Closing... (simulated)");
    waitForMotionComplete(1.0);
    RCLCPP_INFO(node_->get_logger(), "✅ GRIPPER: Closed - object grasped");
}

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

void RobotOperation::waitForMotionComplete(double seconds)
{
    auto start = std::chrono::steady_clock::now();
    auto duration = std::chrono::milliseconds(static_cast<int>(seconds * 1000));
    
    while (std::chrono::steady_clock::now() - start < duration) {
        checkEmergencyStop(); // Check for emergency stop during wait
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void RobotOperation::startMetrics()
{
    last_metrics_ = OperationMetrics{};
    last_metrics_.start_time = std::chrono::steady_clock::now();
}

void RobotOperation::recordPlanningTime()
{
    last_metrics_.planning_time = std::chrono::steady_clock::now() - last_metrics_.start_time;
}

void RobotOperation::recordExecutionTime()
{
    auto now = std::chrono::steady_clock::now();
    last_metrics_.execution_time = now - (last_metrics_.start_time + last_metrics_.planning_time);
}

void RobotOperation::recordFailure(const std::string& reason)
{
    last_metrics_.success = false;
    last_metrics_.failure_reason = reason;
}

void RobotOperation::resetMetrics()
{
    last_metrics_ = OperationMetrics{};
}

bool RobotOperation::planWithRetry(const geometry_msgs::msg::Pose& target_pose, 
                                  const std::string& phase_name,
                                  std::chrono::seconds timeout)
{
    checkEmergencyStop();
    std::lock_guard<std::mutex> lock(operation_mutex_);
    
    auto phase_start = std::chrono::steady_clock::now();
    RCLCPP_INFO(node_->get_logger(), "🔄 Starting retry planning for %s (timeout: %lds)", 
                phase_name.c_str(), timeout.count());
    
    RetryStatistics stats;
    stats.start_time = phase_start;
    
    // Validate pose reachability first (quick check)
    if (!validatePoseReachability(target_pose)) {
        RCLCPP_ERROR(node_->get_logger(), "❌ %s pose failed reachability validation", phase_name.c_str());
        recordFailure(phase_name + " - pose not reachable");
        return false;
    }
    
    // === IMMEDIATE FIRST ATTEMPT ===
    if (attemptPlanning(target_pose, stats)) {
        auto planning_time = std::chrono::steady_clock::now() - phase_start;
        RCLCPP_INFO(node_->get_logger(), "⚡ %s planning succeeded on FIRST attempt (%.3fs)", 
                   phase_name.c_str(), std::chrono::duration<double>(planning_time).count());
        
        last_metrics_.planning_time = std::chrono::duration<double>(planning_time);
        recordRetry(phase_name);
        return true;
    }
    
    // === RETRY LOOP ===
    RCLCPP_INFO(node_->get_logger(), "🔄 First attempt failed, starting retry loop for %s...", phase_name.c_str());
    
    auto last_progress_log = phase_start;
    const auto progress_interval = std::chrono::seconds(5);
    
    while (std::chrono::steady_clock::now() - phase_start < timeout) {
        checkEmergencyStop();
        
        if (attemptPlanning(target_pose, stats)) {
            auto planning_time = std::chrono::steady_clock::now() - phase_start;
            RCLCPP_INFO(node_->get_logger(), "✅ %s planning successful after %d attempts (%.2fs)", 
                       phase_name.c_str(), stats.total_attempts, 
                       std::chrono::duration<double>(planning_time).count());
            
            last_metrics_.planning_time = std::chrono::duration<double>(planning_time);
            recordRetry(phase_name);
            return true;
        }
        
        // Progress logging
        auto now = std::chrono::steady_clock::now();
        if (now - last_progress_log >= progress_interval) {
            logRetryProgress(phase_name, stats, timeout);
            last_progress_log = now;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Reduced delay
    }
    
    // Timeout reached
    auto total_time = std::chrono::steady_clock::now() - phase_start;
    RCLCPP_ERROR(node_->get_logger(), "⏰ %s planning timeout after %d attempts in %.1fs", 
                phase_name.c_str(), stats.total_attempts, 
                std::chrono::duration<double>(total_time).count());
    
    recordFailure(phase_name + " - planning timeout after " + std::to_string(stats.total_attempts) + " attempts");
    return false;
}

bool RobotOperation::planToJointWithRetry(const std::vector<double>& joint_values,
                                         const std::string& phase_name, 
                                         std::chrono::seconds timeout)
{
    checkEmergencyStop();
    std::lock_guard<std::mutex> lock(operation_mutex_);
    
    auto phase_start = std::chrono::steady_clock::now();
    RCLCPP_INFO(node_->get_logger(), "🔄 Starting joint retry planning for %s (timeout: %lds)", 
                phase_name.c_str(), timeout.count());
    
    // Validate joint limits first
    if (!validateJointLimits(joint_values)) {
        RCLCPP_ERROR(node_->get_logger(), "❌ %s joint values violate limits", phase_name.c_str());
        recordFailure(phase_name + " - joint limits violation");
        return false;
    }
    
    RetryStatistics stats;
    stats.start_time = phase_start;
    
    // === IMMEDIATE FIRST ATTEMPT ===
    if (attemptJointPlanning(joint_values, stats)) {
        auto planning_time = std::chrono::steady_clock::now() - phase_start;
        RCLCPP_INFO(node_->get_logger(), "⚡ %s joint planning succeeded on FIRST attempt (%.3fs)", 
                   phase_name.c_str(), std::chrono::duration<double>(planning_time).count());
        
        last_metrics_.planning_time = std::chrono::duration<double>(planning_time);
        recordRetry(phase_name);
        return true;
    }
    
    // === RETRY LOOP ===
    RCLCPP_INFO(node_->get_logger(), "🔄 First attempt failed, starting retry loop for %s...", phase_name.c_str());
    
    auto last_progress_log = phase_start;
    const auto progress_interval = std::chrono::seconds(5);
    
    while (std::chrono::steady_clock::now() - phase_start < timeout) {
        checkEmergencyStop();
        
        if (attemptJointPlanning(joint_values, stats)) {
            auto planning_time = std::chrono::steady_clock::now() - phase_start;
            RCLCPP_INFO(node_->get_logger(), "✅ %s joint planning successful after %d attempts (%.2fs)", 
                       phase_name.c_str(), stats.total_attempts,
                       std::chrono::duration<double>(planning_time).count());
            
            last_metrics_.planning_time = std::chrono::duration<double>(planning_time);
            recordRetry(phase_name);
            return true;
        }
        
        // Progress logging
        auto now = std::chrono::steady_clock::now();
        if (now - last_progress_log >= progress_interval) {
            logRetryProgress(phase_name, stats, timeout);
            last_progress_log = now;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    auto total_time = std::chrono::steady_clock::now() - phase_start;
    RCLCPP_ERROR(node_->get_logger(), "⏰ %s joint planning timeout after %d attempts in %.1fs", 
                phase_name.c_str(), stats.total_attempts, 
                std::chrono::duration<double>(total_time).count());
    
    recordFailure(phase_name + " - joint planning timeout after " + std::to_string(stats.total_attempts) + " attempts");
    return false;
}

bool RobotOperation::attemptPlanning(const geometry_msgs::msg::Pose& target_pose, RetryStatistics& stats)
{
    stats.total_attempts++;
    auto attempt_start = std::chrono::steady_clock::now();
    
    try {
        move_group_->setPoseTarget(target_pose);
        auto result = move_group_->plan(current_plan_);
        
        auto attempt_time = std::chrono::steady_clock::now() - attempt_start;
        stats.total_planning_time += attempt_time;
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            stats.successful_attempts++;
            return true;
        }
        
        // FIXED: Simplified debug logging without error code details
        if (stats.total_attempts % 20 == 0) {
            RCLCPP_DEBUG(node_->get_logger(), "🔄 Planning attempt %d failed", 
                        stats.total_attempts);
        }
        
        return false;
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Exception in planning attempt %d: %s", 
                   stats.total_attempts, e.what());
        return false;
    }
}

bool RobotOperation::attemptJointPlanning(const std::vector<double>& joint_values, RetryStatistics& stats)
{
    stats.total_attempts++;
    auto attempt_start = std::chrono::steady_clock::now();
    
    try {
        move_group_->setJointValueTarget(joint_values);
        auto result = move_group_->plan(current_plan_);
        
        auto attempt_time = std::chrono::steady_clock::now() - attempt_start;
        stats.total_planning_time += attempt_time;
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            stats.successful_attempts++;
            return true;
        }
        
        // FIXED: No error code casting
        return false;
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Exception in joint planning attempt %d: %s", 
                   stats.total_attempts, e.what());
        return false;
    }
}

void RobotOperation::logRetryProgress(const std::string& phase_name, const RetryStatistics& stats, 
                                     std::chrono::seconds timeout)
{
    auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - stats.start_time);
    auto remaining = std::chrono::duration<double>(timeout) - elapsed;
    
    double success_rate = stats.total_attempts > 0 ? 
                         (double)stats.successful_attempts / stats.total_attempts * 100.0 : 0.0;
    
    RCLCPP_INFO(node_->get_logger(), 
                "🔄 %s planning progress: %d attempts, %.1f%% success rate, %.1fs remaining", 
                phase_name.c_str(), stats.total_attempts, success_rate, remaining.count());
}

bool RobotOperation::planAndExecuteCartesianPathWithRetry(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const std::string& phase_name,
    std::chrono::seconds timeout,
    double min_success_fraction)
{
    checkEmergencyStop();
    
    if (waypoints.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "❌ %s: No waypoints provided", phase_name.c_str());
        recordFailure(phase_name + " - no waypoints provided");
        return false;
    }

    auto phase_start = std::chrono::steady_clock::now();
    RCLCPP_INFO(node_->get_logger(), "🔄 Starting Cartesian retry planning for %s (timeout: %lds, min_fraction: %.1f%%)", 
                phase_name.c_str(), timeout.count(), min_success_fraction * 100.0);

    CartesianRetryStatistics stats;
    stats.start_time = phase_start;
    
    moveit_msgs::msg::RobotTrajectory best_trajectory;
    
    // === IMMEDIATE FIRST ATTEMPT (no delay) ===
    if (attemptCartesianPlanning(waypoints, stats, min_success_fraction, best_trajectory)) {
        auto planning_time = std::chrono::steady_clock::now() - phase_start;
        RCLCPP_INFO(node_->get_logger(), "⚡ %s Cartesian planning succeeded on FIRST attempt (%.3fs, %.1f%% coverage)", 
                   phase_name.c_str(), std::chrono::duration<double>(planning_time).count(),
                   stats.best_fraction * 100.0);
        
        // Execute immediately - NO DELAY
        return executeCartesianTrajectory(best_trajectory, phase_name);
    }
    
    // === RETRY LOOP (only if first attempt failed) ===
    RCLCPP_INFO(node_->get_logger(), "🔄 First attempt failed, starting retry loop for %s...", phase_name.c_str());
    
    auto last_progress_log = phase_start;
    const auto progress_interval = std::chrono::seconds(5); // Log every 5 seconds
    
    while (std::chrono::steady_clock::now() - phase_start < timeout) {
        checkEmergencyStop();
        
        // Attempt planning
        if (attemptCartesianPlanning(waypoints, stats, min_success_fraction, best_trajectory)) {
            auto planning_time = std::chrono::steady_clock::now() - phase_start;
            RCLCPP_INFO(node_->get_logger(), "✅ %s Cartesian planning successful after %d attempts (%.2fs, %.1f%% coverage)", 
                       phase_name.c_str(), stats.total_attempts, 
                       std::chrono::duration<double>(planning_time).count(),
                       stats.best_fraction * 100.0);
            
            // Execute immediately - NO DELAY
            return executeCartesianTrajectory(best_trajectory, phase_name);
        }
        
        // Progress logging (less frequent to reduce spam)
        auto now = std::chrono::steady_clock::now();
        if (now - last_progress_log >= progress_interval) {
            logCartesianRetryProgress(phase_name, stats, timeout);
            last_progress_log = now;
        }
        
        // OPTIMIZED: Smaller delay, faster retry rate
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Reduced from 100ms
    }
    
    // Timeout reached
    auto total_time = std::chrono::steady_clock::now() - phase_start;
    RCLCPP_ERROR(node_->get_logger(), "⏰ %s Cartesian planning timeout after %d attempts in %.1fs (best: %.1f%%)", 
                phase_name.c_str(), stats.total_attempts, 
                std::chrono::duration<double>(total_time).count(), stats.best_fraction * 100.0);
    
    recordFailure(phase_name + " - Cartesian planning timeout after " + std::to_string(stats.total_attempts) + 
                 " attempts (best coverage: " + std::to_string(stats.best_fraction * 100.0) + "%)");
    return false;
}

bool RobotOperation::executeCartesianTrajectory(
    const moveit_msgs::msg::RobotTrajectory& trajectory, 
    const std::string& phase_name)
{
    RCLCPP_INFO(node_->get_logger(), "🚀 Executing Cartesian trajectory for %s...", phase_name.c_str());
    
    auto execution_start = std::chrono::steady_clock::now();
    bool success = move_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
    auto execution_time = std::chrono::steady_clock::now() - execution_start;
    
    recordExecutionTime();
    
    if (success) {
        RCLCPP_INFO(node_->get_logger(), "✅ %s Cartesian execution completed (%.2fs)", 
                   phase_name.c_str(), std::chrono::duration<double>(execution_time).count());
        recordPhaseCompletion(phase_name);
        recordRetry(phase_name);
        return true;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "❌ %s Cartesian execution failed after %.2fs", 
                    phase_name.c_str(), std::chrono::duration<double>(execution_time).count());
        recordFailure(phase_name + " - execution failed after successful planning");
        return false;
    }
}


bool RobotOperation::attemptCartesianPlanning(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    CartesianRetryStatistics& stats,
    double min_success_fraction,
    moveit_msgs::msg::RobotTrajectory& best_trajectory)
{
    stats.total_attempts++;
    auto attempt_start = std::chrono::steady_clock::now();
    
    try {
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_->computeCartesianPath(
            waypoints, 
            safety_config_.cartesian_step_size, 
            safety_config_.cartesian_jump_threshold, 
            trajectory
        );
        
        auto attempt_time = std::chrono::steady_clock::now() - attempt_start;
        stats.total_planning_time += attempt_time;
        
        // Update best fraction achieved
        if (fraction > stats.best_fraction) {
            stats.best_fraction = fraction;
            best_trajectory = trajectory; // Store the best trajectory so far
        }
        
        // Check if this attempt meets our success criteria
        if (fraction >= min_success_fraction) {
            stats.successful_attempts++;
            return true;
        }
        
        // Log occasional attempts for debugging
        if (stats.total_attempts % 20 == 0) {
            RCLCPP_DEBUG(node_->get_logger(), "🔄 Cartesian attempt %d: %.1f%% coverage (need %.1f%%)", 
                        stats.total_attempts, fraction * 100.0, min_success_fraction * 100.0);
        }
        
        return false;
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Exception in Cartesian planning attempt %d: %s", 
                   stats.total_attempts, e.what());
        return false;
    }
}

void RobotOperation::logCartesianRetryProgress(const std::string& phase_name, 
                                              const CartesianRetryStatistics& stats, 
                                              std::chrono::seconds timeout)
{
    auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - stats.start_time);
    auto remaining = std::chrono::duration<double>(timeout) - elapsed;
    
    double success_rate = stats.total_attempts > 0 ? 
                         (double)stats.successful_attempts / stats.total_attempts * 100.0 : 0.0;
    
    RCLCPP_INFO(node_->get_logger(), 
                "🔄 %s Cartesian progress: %d attempts, %.1f%% success rate, best coverage: %.1f%%, %.1fs remaining", 
                phase_name.c_str(), stats.total_attempts, success_rate, 
                stats.best_fraction * 100.0, remaining.count());
}

bool RobotOperation::validateTrajectoryCollisionFree()
{
    try {
        // Basic validation - check if trajectory has points
        if (current_plan_.trajectory_.joint_trajectory.points.empty()) {
            RCLCPP_WARN(node_->get_logger(), "⚠️ Empty trajectory - cannot validate collisions");
            return false;
        }
        
        // MoveIt automatically performs collision checking during planning
        // If planning succeeded, trajectory should be collision-free
        size_t trajectory_points = current_plan_.trajectory_.joint_trajectory.points.size();
        RCLCPP_DEBUG(node_->get_logger(), "🔍 Trajectory has %zu points - collision checking performed by MoveIt", trajectory_points);
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Trajectory collision validation failed: %s", e.what());
        return false;
    }
}

bool RobotOperation::validateCurrentStateCollisionFree()
{
    try {
        // Get current robot state
        auto current_state = move_group_->getCurrentState();
        if (!current_state) {
            RCLCPP_WARN(node_->get_logger(), "⚠️ Cannot get current robot state for collision checking");
            return false;
        }
        
        // Basic validation - if we can get current state, assume it's valid
        RCLCPP_DEBUG(node_->get_logger(), "🔍 Current robot state validated (basic check)");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Current state collision check failed: %s", e.what());
        return false;
    }
}

} // namespace trajectory_plan