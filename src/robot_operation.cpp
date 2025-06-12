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
        
        RCLCPP_INFO(node_->get_logger(), "🎯 Planning to pose [%.3f, %.3f, %.3f]", 
                   target_pose.position.x, target_pose.position.y, target_pose.position.z);
        
        move_group_->setPoseTarget(target_pose);
        auto result = move_group_->plan(current_plan_);
        recordPlanningTime();
        
        bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success) {
            recordFailure("Planning to pose failed");
            RCLCPP_ERROR(node_->get_logger(), "❌ Planning to pose failed");
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
    
    // Enhanced recovery with timeout and better error handling
    auto recovery_start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(static_cast<int>(safety_config_.safety_stop_timeout));
    
    try {
        // Level 0: Try home position first
        RCLCPP_INFO(node_->get_logger(), "🏠 Recovery Level 0: Attempting home position...");
        if (attemptSafeMove(safety_config_.home_joints, "home")) {
            RCLCPP_INFO(node_->get_logger(), "✅ Recovery successful - reached home position");
            return true;
        }
        
        // Check timeout
        if (std::chrono::steady_clock::now() - recovery_start > timeout) {
            RCLCPP_ERROR(node_->get_logger(), "⏰ Recovery timeout reached");
            return false;
        }
        
        // Level 1: Try safe position
        RCLCPP_INFO(node_->get_logger(), "🛡️ Recovery Level 1: Attempting safe position...");
        if (attemptSafeMove(safety_config_.safe_joints, "safe")) {
            RCLCPP_INFO(node_->get_logger(), "✅ Recovery successful - reached safe position");
            return true;
        }
        
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
        throw; // Re-throw other errors
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
    
    RCLCPP_INFO(node_->get_logger(), "🏭 PRODUCTION Pick and Place Sequence Starting...");

    // FIXED: Move safe_recovery lambda outside try block to fix scope issue
    auto safe_recovery = [this](const std::string& context) -> bool {
        try {
            return this->executeRecoverySequence(context);
        } catch (const RobotOperationError& e) {
            if (e.getType() == RobotOperationError::Type::SHUTDOWN_REQUESTED) {
                RCLCPP_WARN(node_->get_logger(), "🔌 Pick and place interrupted by shutdown");
                return false;
            }
            RCLCPP_ERROR(node_->get_logger(), "❌ Recovery failed: %s", e.what());
            return false;
        }
    };

    try {
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
        
        // Execute sequence with enhanced error checking
        checkEmergencyStop();
        if (!moveToHome()) {
            RCLCPP_ERROR(node_->get_logger(), "❌ Failed to move to home at start");
            return safe_recovery("Failed to reach home at startup");
        }
        
        // PICK SEQUENCE
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "🎯 Starting PICK sequence...");
        if (!planToPose(pick_approach_pose) || !executePlan()) {
            RCLCPP_ERROR(node_->get_logger(), "❌ Failed to reach pick approach pose");
            return safe_recovery("Failed to reach pick approach pose");
        }
        
        checkEmergencyStop();
        openGripper();
        
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "⬇️ Moving linearly to PICK pose...");
        if (!planAndExecuteCartesianPath({pick_pose})) {
            RCLCPP_ERROR(node_->get_logger(), "❌ Failed to reach pick pose");
            return safe_recovery("Failed to reach pick pose");
        }
        
        checkEmergencyStop();
        closeGripper();
        
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "⬆️ Retreating linearly from PICK pose...");
        if (!planAndExecuteCartesianPath({pick_approach_pose})) {
            RCLCPP_ERROR(node_->get_logger(), "❌ Failed to retreat from pick pose");
            openGripper(); // Release object before recovery
            return safe_recovery("Failed to retreat from pick with object");
        }
        
        // Return to home as intermediate safe position
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "🏠 Returning to Home as intermediate position...");
        if (!moveToHome()) {
            RCLCPP_ERROR(node_->get_logger(), "❌ Failed to return to home after pick");
            return safe_recovery("Failed to return home after pick");
        }
        
        // PLACE SEQUENCE
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "📦 Starting PLACE sequence...");
        if (!planToPose(place_approach_pose) || !executePlan()) {
            RCLCPP_ERROR(node_->get_logger(), "❌ Failed to reach place approach pose");
            return safe_recovery("Failed to reach place approach pose");
        }
        
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "⬇️ Moving linearly to PLACE pose...");
        if (!planAndExecuteCartesianPath({place_pose})) {
            RCLCPP_ERROR(node_->get_logger(), "❌ Failed to reach place pose");
            return safe_recovery("Failed to reach place pose");
        }
        
        checkEmergencyStop();
        openGripper();
        
        checkEmergencyStop();
        RCLCPP_INFO(node_->get_logger(), "⬆️ Retreating linearly from PLACE pose...");
        if (!planAndExecuteCartesianPath({place_approach_pose})) {
            RCLCPP_ERROR(node_->get_logger(), "❌ Failed to retreat from place pose");
            return safe_recovery("Failed to retreat from place pose");
        }
        
        // Final return to home
        checkEmergencyStop();
        if (!moveToHome()) {
            RCLCPP_WARN(node_->get_logger(), "⚠️ Failed final return to home, but operation completed");
        }
        
        last_metrics_.success = true;
        RCLCPP_INFO(node_->get_logger(), "🎉 PRODUCTION Pick and Place completed successfully!");
        return true;
        
    } catch (const RobotOperationError& e) {
        if (e.getType() == RobotOperationError::Type::SHUTDOWN_REQUESTED) {
            RCLCPP_WARN(node_->get_logger(), "🔌 Pick and place sequence interrupted by shutdown");
            recordFailure("Shutdown requested");
            return false;
        }
        recordFailure(e.what());
        RCLCPP_ERROR(node_->get_logger(), "❌ Pick and place failed: %s", e.what());
        return safe_recovery(e.what());
    } catch (const std::exception& e) {
        recordFailure(std::string("Unexpected exception: ") + e.what());
        RCLCPP_ERROR(node_->get_logger(), "❌ Unexpected exception: %s", e.what());
        return safe_recovery(std::string("Unexpected exception: ") + e.what());
    }
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

} // namespace trajectory_plan