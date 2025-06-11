#include "trajectory_plan/robot_operation.hpp"
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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
  // Set default planning parameters
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
  move_group_->setPlanningTime(5.0);
  move_group_->setNumPlanningAttempts(10);
  
  // Set default positions
  home_joint_values_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  safe_joint_values_ = {0.0, -0.5, -1.0, 0.0, 1.5, 0.0};
  
  initialized_ = true;
  
  RCLCPP_INFO(node_->get_logger(), "✅ RobotOperation initialized");
  RCLCPP_INFO(node_->get_logger(), "   Planning frame: %s", 
              move_group_->getPlanningFrame().c_str());
  RCLCPP_INFO(node_->get_logger(), "   End effector: %s", 
              move_group_->getEndEffectorLink().c_str());
  
  // Print available joints
  auto joint_names = move_group_->getJointNames();
  RCLCPP_INFO(node_->get_logger(), "   Available joints (%zu):", joint_names.size());
  for (size_t i = 0; i < joint_names.size(); ++i) {
    RCLCPP_INFO(node_->get_logger(), "     [%zu] %s", i, joint_names[i].c_str());
  }
}

// Configuration methods
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
    setVelocityScaling(0.2);      // 20% max velocity
    setAccelerationScaling(0.15); // 15% max acceleration
    RCLCPP_INFO(node_->get_logger(), "🐌 Smooth motion enabled (slow and safe)");
  } else {
    setVelocityScaling(0.5);      // 50% max velocity  
    setAccelerationScaling(0.3);  // 30% max acceleration
    RCLCPP_INFO(node_->get_logger(), "⚡ Normal motion enabled");
  }
}

void RobotOperation::setHomePosition(const std::vector<double>& joints)
{
  if (joints.size() == move_group_->getJointNames().size()) {
    home_joint_values_ = joints;
    RCLCPP_INFO(node_->get_logger(), "🏠 Home position updated");
  } else {
    RCLCPP_WARN(node_->get_logger(), "⚠️ Invalid home position size: %zu (expected %zu)", 
                joints.size(), move_group_->getJointNames().size());
  }
}

void RobotOperation::setSafePosition(const std::vector<double>& joints)
{
  if (joints.size() == move_group_->getJointNames().size()) {
    safe_joint_values_ = joints;
    RCLCPP_INFO(node_->get_logger(), "🛡️ Safe position updated");
  } else {
    RCLCPP_WARN(node_->get_logger(), "⚠️ Invalid safe position size: %zu (expected %zu)", 
                joints.size(), move_group_->getJointNames().size());
  }
}

// Planning methods
bool RobotOperation::planToPose(const geometry_msgs::msg::Pose& target_pose)
{
  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(), "❌ RobotOperation not initialized");
    return false;
  }
  
  RCLCPP_INFO(node_->get_logger(), "🎯 Planning to pose [%.3f, %.3f, %.3f]", 
              target_pose.position.x, target_pose.position.y, target_pose.position.z);
  
  move_group_->setPoseTarget(target_pose);
  
  bool success = (move_group_->plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success) {
    RCLCPP_INFO(node_->get_logger(), "✅ Planning successful");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "❌ Planning failed");
  }
  
  return success;
}

bool RobotOperation::planToJointPosition(const std::vector<double>& joint_values)
{
  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(), "❌ RobotOperation not initialized");
    return false;
  }
  
  auto joint_names = move_group_->getJointNames();
  
  if (joint_values.size() != joint_names.size()) {
    RCLCPP_ERROR(node_->get_logger(), 
                 "❌ Joint values size (%zu) doesn't match joints (%zu)", 
                 joint_values.size(), joint_names.size());
    return false;
  }
  
  // Create joint target map
  std::map<std::string, double> joint_target;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    joint_target[joint_names[i]] = joint_values[i];
  }
  
  move_group_->setJointValueTarget(joint_target);
  
  bool success = (move_group_->plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success) {
    RCLCPP_INFO(node_->get_logger(), "✅ Joint planning successful");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "❌ Joint planning failed");
  }
  
  return success;
}

// Execution methods
bool RobotOperation::executePlan()
{
  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(), "❌ RobotOperation not initialized");
    return false;
  }
  
  RCLCPP_INFO(node_->get_logger(), "🚀 Executing planned trajectory...");
  
  auto result = move_group_->execute(current_plan_);
  bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success) {
    RCLCPP_INFO(node_->get_logger(), "✅ Execution successful");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "❌ Execution failed");
  }
  
  return success;
}

// High-level operations
bool RobotOperation::moveToHome()
{
  RCLCPP_INFO(node_->get_logger(), "🏠 Moving to home position...");
  return planToJointPosition(home_joint_values_) && executePlan();
}

bool RobotOperation::moveToSafePosition()
{
  RCLCPP_INFO(node_->get_logger(), "🛡️ Moving to safe position...");
  return planToJointPosition(safe_joint_values_) && executePlan();
}

bool RobotOperation::executeProfessionalPickAndPlace(
  double pick_x_mm, double pick_y_mm, double pick_z_mm,
  double pick_roll_deg, double pick_pitch_deg, double pick_yaw_deg,
  double place_x_mm, double place_y_mm, double place_z_mm,
  double place_roll_deg, double place_pitch_deg, double place_yaw_deg,
  double clearance_height_mm)
{
  RCLCPP_INFO(node_->get_logger(), "🔄 PROFESSIONAL Pick and Place Sequence Starting...");
  
  // Phase 1: Preparation
  RCLCPP_INFO(node_->get_logger(), "📍 Phase 1: Checking current pose and moving to home");
  checkAndPrintCurrentPose();
  
  if (!moveToHome()) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Failed to move to home position");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "✅ Robot at home position");
  
  // Phase 2: Pick Operation
  RCLCPP_INFO(node_->get_logger(), "🎯 Phase 2: Pick operation");
  RCLCPP_INFO(node_->get_logger(), "   Target: [%.1f, %.1f, %.1f] mm, [%.1f°, %.1f°, %.1f°]", 
              pick_x_mm, pick_y_mm, pick_z_mm, pick_roll_deg, pick_pitch_deg, pick_yaw_deg);
  
  // Create pick pose from mm/degrees
  auto pick_pose = createPoseFromMmAndDegrees(pick_x_mm, pick_y_mm, pick_z_mm,
                                              pick_roll_deg, pick_pitch_deg, pick_yaw_deg);
  
  // Plan trajectory from home to pick
  RCLCPP_INFO(node_->get_logger(), "🗺️ Planning smooth path: Home → Pick position");
  if (!planToPose(pick_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Failed to plan path to pick position");
    return false;
  }
  
  // Open gripper BEFORE moving to pick
  RCLCPP_INFO(node_->get_logger(), "🔓 Opening gripper before pick movement");
  openGripper();
  
  // Execute movement to pick position
  RCLCPP_INFO(node_->get_logger(), "🚀 Executing movement to pick position");
  if (!executePlan()) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Failed to reach pick position");
    return false;
  }
  
  // Close gripper when arrived at pick target
  RCLCPP_INFO(node_->get_logger(), "🤏 Arrived at pick position - closing gripper");
  closeGripper();
  
  // Phase 3: Place Operation  
  RCLCPP_INFO(node_->get_logger(), "📦 Phase 3: Place operation");
  RCLCPP_INFO(node_->get_logger(), "   Target: [%.1f, %.1f, %.1f] mm, [%.1f°, %.1f°, %.1f°]", 
              place_x_mm, place_y_mm, place_z_mm, place_roll_deg, place_pitch_deg, place_yaw_deg);
  
  // Create place pose from mm/degrees
  auto place_pose = createPoseFromMmAndDegrees(place_x_mm, place_y_mm, place_z_mm,
                                               place_roll_deg, place_pitch_deg, place_yaw_deg);
  
  // Plan trajectory from pick to place
  RCLCPP_INFO(node_->get_logger(), "🗺️ Planning smooth path: Pick → Place position");
  if (!planToPose(place_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Failed to plan path to place position");
    return false;
  }
  
  // Execute movement to place position
  RCLCPP_INFO(node_->get_logger(), "🚀 Executing movement to place position");
  if (!executePlan()) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Failed to reach place position");
    return false;
  }
  
  // Open gripper to release object
  RCLCPP_INFO(node_->get_logger(), "🖐️ Arrived at place position - opening gripper");
  openGripper();
  
  // Move upward for clearance
  RCLCPP_INFO(node_->get_logger(), "⬆️ Moving upward %.1f mm for clearance", clearance_height_mm);
  auto clearance_pose = place_pose;
  clearance_pose.position.z += clearance_height_mm / 1000.0;  // Convert mm to meters
  
  if (!planToPose(clearance_pose) || !executePlan()) {
    RCLCPP_WARN(node_->get_logger(), "⚠️ Clearance move failed, but pick-place completed");
  } else {
    RCLCPP_INFO(node_->get_logger(), "✅ Clearance move completed");
  }
  
  RCLCPP_INFO(node_->get_logger(), "🎉 PROFESSIONAL Pick and Place completed successfully!");
  return true;
}

// Legacy pick-and-place (for backward compatibility)
bool RobotOperation::planPickAndPlace(
  const geometry_msgs::msg::Pose& pick_pose,
  const geometry_msgs::msg::Pose& place_pose,
  double approach_distance,
  double retreat_distance)
{
  RCLCPP_WARN(node_->get_logger(), "⚠️ Using legacy planPickAndPlace method");
  return executePickAndPlaceSequence(pick_pose, place_pose, approach_distance, retreat_distance);
}

bool RobotOperation::executePickAndPlaceSequence(
  const geometry_msgs::msg::Pose& pick_pose,
  const geometry_msgs::msg::Pose& place_pose,
  double approach_distance,
  double retreat_distance)
{
  // Suppress unused parameter warnings for legacy method
  (void)approach_distance;
  (void)retreat_distance;
  
  // Legacy method - redirects to professional method with default values
  RCLCPP_WARN(node_->get_logger(), "⚠️ Using legacy pick-and-place method. Consider using executeProfessionalPickAndPlace()");
  
  // Convert poses to mm/degrees (approximate conversion for legacy support)
  double pick_x_mm = pick_pose.position.x * 1000.0;
  double pick_y_mm = pick_pose.position.y * 1000.0;
  double pick_z_mm = pick_pose.position.z * 1000.0;
  
  double place_x_mm = place_pose.position.x * 1000.0;
  double place_y_mm = place_pose.position.y * 1000.0;
  double place_z_mm = place_pose.position.z * 1000.0;
  
  // Use professional method with default orientations
  return executeProfessionalPickAndPlace(
    pick_x_mm, pick_y_mm, pick_z_mm, 0.0, 90.0, 0.0,  // Point down
    place_x_mm, place_y_mm, place_z_mm, 0.0, 90.0, 0.0,  // Point down
    50.0  // 5cm clearance
  );
}

// Gripper control methods
void RobotOperation::openGripper()
{
  RCLCPP_INFO(node_->get_logger(), "🔓 GRIPPER: Opening... (simulated)");
  // TODO: Replace with actual gripper control
  // gripper_action_client_->send_goal(open_goal);
  waitForMotionComplete(1.0);
  RCLCPP_INFO(node_->get_logger(), "✅ GRIPPER: Opened");
}

void RobotOperation::closeGripper()
{
  RCLCPP_INFO(node_->get_logger(), "🤏 GRIPPER: Closing... (simulated)");
  // TODO: Replace with actual gripper control
  // gripper_action_client_->send_goal(close_goal);
  waitForMotionComplete(1.0);
  RCLCPP_INFO(node_->get_logger(), "✅ GRIPPER: Closed - object grasped");
}

// Utility methods
void RobotOperation::checkAndPrintCurrentPose()
{
  if (!initialized_) {
    RCLCPP_WARN(node_->get_logger(), "⚠️ RobotOperation not initialized");
    return;
  }
  
  auto current_pose = move_group_->getCurrentPose().pose;
  
  // Convert to mm and degrees for readability
  double x_mm = current_pose.position.x * 1000.0;
  double y_mm = current_pose.position.y * 1000.0;
  double z_mm = current_pose.position.z * 1000.0;
  
  // Convert quaternion to RPY
  tf2::Quaternion q(current_pose.orientation.x, current_pose.orientation.y, 
                    current_pose.orientation.z, current_pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  
  // Convert radians to degrees
  double roll_deg = roll * 180.0 / M_PI;
  double pitch_deg = pitch * 180.0 / M_PI;
  double yaw_deg = yaw * 180.0 / M_PI;
  
  RCLCPP_INFO(node_->get_logger(), "📍 CURRENT ROBOT POSE:");
  RCLCPP_INFO(node_->get_logger(), "   Position: [%.1f, %.1f, %.1f] mm", x_mm, y_mm, z_mm);
  RCLCPP_INFO(node_->get_logger(), "   Orientation: [%.1f°, %.1f°, %.1f°] (Roll, Pitch, Yaw)", 
              roll_deg, pitch_deg, yaw_deg);
  RCLCPP_INFO(node_->get_logger(), "   Frame: %s", move_group_->getPlanningFrame().c_str());
}

geometry_msgs::msg::Pose RobotOperation::createPoseFromMmAndDegrees(
  double x_mm, double y_mm, double z_mm,
  double roll_deg, double pitch_deg, double yaw_deg)
{
  geometry_msgs::msg::Pose pose;
  
  // Convert mm to meters
  pose.position.x = x_mm / 1000.0;
  pose.position.y = y_mm / 1000.0;
  pose.position.z = z_mm / 1000.0;
  
  // Convert degrees to radians and create quaternion
  double roll_rad = roll_deg * M_PI / 180.0;
  double pitch_rad = pitch_deg * M_PI / 180.0;
  double yaw_rad = yaw_deg * M_PI / 180.0;
  
  tf2::Quaternion q;
  q.setRPY(roll_rad, pitch_rad, yaw_rad);
  
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  
  RCLCPP_DEBUG(node_->get_logger(), "📐 Converted [%.1f, %.1f, %.1f] mm, [%.1f°, %.1f°, %.1f°] → ROS pose", 
               x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg);
  
  return pose;
}

geometry_msgs::msg::Pose RobotOperation::getCurrentPose()
{
  if (!initialized_) {
    RCLCPP_WARN(node_->get_logger(), "⚠️ RobotOperation not initialized");
    return geometry_msgs::msg::Pose{};
  }
  
  return move_group_->getCurrentPose().pose;
}

void RobotOperation::waitForMotionComplete(double seconds)
{
  RCLCPP_INFO(node_->get_logger(), "⏳ Waiting %.1f seconds...", seconds);
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
}

// Private utility methods
geometry_msgs::msg::Pose RobotOperation::createApproachPose(
  const geometry_msgs::msg::Pose& target_pose, double distance)
{
  auto approach_pose = target_pose;
  approach_pose.position.z += distance;
  return approach_pose;
}

geometry_msgs::msg::Pose RobotOperation::createRetreatPose(
  const geometry_msgs::msg::Pose& target_pose, double distance)
{
  auto retreat_pose = target_pose;
  retreat_pose.position.z += distance;
  return retreat_pose;
}

} // namespace trajectory_plan