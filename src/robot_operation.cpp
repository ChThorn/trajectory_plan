#include "trajectory_plan/robot_operation.hpp"
#include <thread>

namespace trajectory_plan
{
RobotOperation::RobotOperation(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group)
  : node_(node)
  , move_group_(move_group)
  , velocity_scaling_factor_(0.3)
  , acceleration_scaling_factor_(0.3)
{
  // Set planning parameters
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
  move_group_->setPlanningTime(5.0);
  move_group_->setNumPlanningAttempts(10);
  
  // Define home position (all joints at 0)
  home_joint_values_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  // Define safe position (joints in safe configuration)
  safe_joint_values_ = {0.0, -0.5, -1.0, 0.0, 1.5, 0.0};
  
  RCLCPP_INFO(node_->get_logger(), "RobotOperation initialized");
  RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", 
              move_group_->getPlanningFrame().c_str());
  RCLCPP_INFO(node_->get_logger(), "End effector frame: %s", 
              move_group_->getEndEffectorLink().c_str());
  
  // Print available joint names to verify
  auto joint_names = move_group_->getJointNames();
  RCLCPP_INFO(node_->get_logger(), "Available joints:");
  for (const auto& name : joint_names) {
    RCLCPP_INFO(node_->get_logger(), "  - %s", name.c_str());
  }
}

bool RobotOperation::planToPose(const geometry_msgs::msg::Pose& target_pose)
{
  move_group_->setPoseTarget(target_pose);
  
  // Plan with collision checking
  bool success = (move_group_->plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success) {
    RCLCPP_INFO(node_->get_logger(), "✅ Planning successful - trajectory avoids table collision");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "❌ Planning failed - may be due to table collision constraints");
    RCLCPP_INFO(node_->get_logger(), "💡 Try adjusting target pose to be further from table surface");
  }
  
  return success;
}

bool RobotOperation::executePlan()
{
  auto result = move_group_->execute(current_plan_);
  bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (success) {
    RCLCPP_INFO(node_->get_logger(), "Execution successful");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Execution failed");
  }
  
  return success;
}

bool RobotOperation::moveToHome()
{
  return planToJointPosition(home_joint_values_) && executePlan();
}

bool RobotOperation::moveToSafePosition()
{
  return planToJointPosition(safe_joint_values_) && executePlan();
}

bool RobotOperation::planToJointPosition(const std::vector<double>& joint_values)
{
  // Get the correct joint names from MoveIt group
  auto joint_names = move_group_->getJointNames();
  
  if (joint_values.size() != joint_names.size()) {
    RCLCPP_ERROR(node_->get_logger(), 
                 "Joint values size (%zu) doesn't match joint names size (%zu)", 
                 joint_values.size(), joint_names.size());
    return false;
  }
  
  // Set joint targets using correct joint names
  std::map<std::string, double> joint_target;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    joint_target[joint_names[i]] = joint_values[i];
  }
  
  move_group_->setJointValueTarget(joint_target);
  
  bool success = (move_group_->plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  
  return success;
}

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

void RobotOperation::setVelocityScaling(double scaling)
{
  velocity_scaling_factor_ = scaling;
  move_group_->setMaxVelocityScalingFactor(scaling);
}

void RobotOperation::setAccelerationScaling(double scaling)
{
  acceleration_scaling_factor_ = scaling;
  move_group_->setMaxAccelerationScalingFactor(scaling);
}

void RobotOperation::setSmoothMotion(bool enable)
{
  if (enable) {
    // Very smooth, safe motion
    setVelocityScaling(0.2);      // 20% max velocity
    setAccelerationScaling(0.15); // 15% max acceleration
    RCLCPP_INFO(node_->get_logger(), "🐌 Smooth motion mode enabled (slow and safe)");
  } else {
    // Normal motion
    setVelocityScaling(0.5);      // 50% max velocity  
    setAccelerationScaling(0.3);  // 30% max acceleration
    RCLCPP_INFO(node_->get_logger(), "⚡ Normal motion mode enabled");
  }
}

void RobotOperation::printCurrentPose()
{
  auto current_pose = move_group_->getCurrentPose().pose;
  RCLCPP_INFO(node_->get_logger(), "Current robot pose:");
  RCLCPP_INFO(node_->get_logger(), "  Position: [%.3f, %.3f, %.3f]", 
              current_pose.position.x, current_pose.position.y, current_pose.position.z);
  RCLCPP_INFO(node_->get_logger(), "  Orientation: [%.3f, %.3f, %.3f, %.3f]",
              current_pose.orientation.x, current_pose.orientation.y, 
              current_pose.orientation.z, current_pose.orientation.w);
}

void RobotOperation::waitForMotionComplete(double seconds)
{
  RCLCPP_INFO(node_->get_logger(), "⏳ Waiting %.1f seconds for motion to complete...", seconds);
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
}

bool RobotOperation::planPickAndPlace(
  const geometry_msgs::msg::Pose& pick_pose,
  const geometry_msgs::msg::Pose& place_pose,
  double approach_distance,
  double retreat_distance)
{
  RCLCPP_INFO(node_->get_logger(), "Planning pick and place trajectory");
  
  // 1. Move to approach pose above pick location
  auto pick_approach = createApproachPose(pick_pose, approach_distance);
  if (!planToPose(pick_approach)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to plan to pick approach pose");
    return false;
  }
  if (!executePlan()) return false;
  
  // 2. Move down to pick pose
  if (!planToPose(pick_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to plan to pick pose");
    return false;
  }
  if (!executePlan()) return false;
  
  // TODO: Add gripper close command here
  RCLCPP_INFO(node_->get_logger(), "Gripper would close here");
  rclcpp::sleep_for(std::chrono::milliseconds(1000));
  
  // 3. Retreat from pick location
  auto pick_retreat = createRetreatPose(pick_pose, retreat_distance);
  if (!planToPose(pick_retreat)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to plan pick retreat");
    return false;
  }
  if (!executePlan()) return false;
  
  // 4. Move to approach pose above place location
  auto place_approach = createApproachPose(place_pose, approach_distance);
  if (!planToPose(place_approach)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to plan to place approach pose");
    return false;
  }
  if (!executePlan()) return false;
  
  // 5. Move down to place pose
  if (!planToPose(place_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to plan to place pose");
    return false;
  }
  if (!executePlan()) return false;
  
  // TODO: Add gripper open command here
  RCLCPP_INFO(node_->get_logger(), "Gripper would open here");
  rclcpp::sleep_for(std::chrono::milliseconds(1000));
  
  // 6. Retreat from place location
  auto place_retreat = createRetreatPose(place_pose, retreat_distance);
  if (!planToPose(place_retreat)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to plan place retreat");
    return false;
  }
  if (!executePlan()) return false;
  
  RCLCPP_INFO(node_->get_logger(), "Pick and place trajectory completed successfully");
  return true;
}

bool RobotOperation::planMountedTablePickAndPlace(
  const geometry_msgs::msg::Pose& pick_pose,
  const geometry_msgs::msg::Pose& place_pose,
  double approach_distance,
  double retreat_distance)
{
  RCLCPP_INFO(node_->get_logger(), "🔧 Planning SIMPLIFIED MOUNTED TABLE pick and place");
  
  double pick_height_mm = pick_pose.position.z * 1000.0;
  double place_height_mm = place_pose.position.z * 1000.0;
  
  RCLCPP_INFO(node_->get_logger(), "   Pick: %.1fmm above table, Place: %.1fmm above table", 
              pick_height_mm, place_height_mm);
  
  // STEP 1: Moderate approach (not too high) - 12cm above pick
  auto pick_high_approach = pick_pose;
  pick_high_approach.position.z += 0.12; // 12cm above pick (more reasonable)
  
  RCLCPP_INFO(node_->get_logger(), "🎯 Planning to high approach: [%.3f, %.3f, %.3f]", 
              pick_high_approach.position.x, pick_high_approach.position.y, pick_high_approach.position.z);
  
  if (!planToPose(pick_high_approach)) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Failed high approach - trying direct approach...");
    
    // FALLBACK: Skip high approach, go directly to normal approach
    auto pick_approach = createApproachPose(pick_pose, approach_distance);
    if (!planToPose(pick_approach)) {
      RCLCPP_ERROR(node_->get_logger(), "❌ Failed direct approach too");
      return false;
    }
  }
  if (!executePlan()) return false;
  
  // STEP 2: Normal approach if we used high approach
  if (pick_high_approach.position.z > pick_pose.position.z + approach_distance + 0.01) {
    auto pick_approach = createApproachPose(pick_pose, approach_distance);
    if (!planToPose(pick_approach)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to plan pick approach");
      return false;
    }
    if (!executePlan()) return false;
  }
  
  // STEP 3: Pick
  RCLCPP_INFO(node_->get_logger(), "🎯 Moving to pick position");
  if (!planToPose(pick_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to plan to pick pose");
    return false;
  }
  if (!executePlan()) return false;
  
  // Gripper close
  RCLCPP_INFO(node_->get_logger(), "🤏 Gripper closing...");
  rclcpp::sleep_for(std::chrono::milliseconds(1000));
  
  // STEP 4: Retreat
  auto pick_retreat = createRetreatPose(pick_pose, retreat_distance);
  if (!planToPose(pick_retreat)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to plan pick retreat");
    return false;
  }
  if (!executePlan()) return false;
  
  // STEP 5: Move to place approach (simpler intermediate)
  auto place_approach = createApproachPose(place_pose, approach_distance);
  RCLCPP_INFO(node_->get_logger(), "🎯 Moving to place approach");
  if (!planToPose(place_approach)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to plan place approach");
    return false;
  }
  if (!executePlan()) return false;
  
  // STEP 6: Place
  RCLCPP_INFO(node_->get_logger(), "🎯 Moving to place position");
  if (!planToPose(place_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to plan to place pose");
    return false;
  }
  if (!executePlan()) return false;
  
  // Gripper open
  RCLCPP_INFO(node_->get_logger(), "🖐️ Gripper opening...");
  rclcpp::sleep_for(std::chrono::milliseconds(1000));
  
  // STEP 7: Final retreat
  auto place_retreat = createRetreatPose(place_pose, retreat_distance);
  if (!planToPose(place_retreat)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to plan place retreat");
    return false;
  }
  if (!executePlan()) return false;
  
  RCLCPP_INFO(node_->get_logger(), "🎉 Simplified mounted table pick and place completed!");
  return true;
}

} // namespace trajectory_plan