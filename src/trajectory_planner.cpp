// #include "trajectory_plan/trajectory_planner.hpp"
// #include <tf2/LinearMath/Quaternion.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <thread>
// #include <algorithm>

// namespace trajectory_plan
// {
// TrajectoryPlanner::TrajectoryPlanner(const rclcpp::Node::SharedPtr& node)
//   : node_(node)
//   , velocity_scaling_factor_(0.3)
//   , acceleration_scaling_factor_(0.3)
// {
//   // Initialize MoveIt interfaces
//   move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
//     node_, "mainpulation"); // Using the planning group from SRDF
  
//   planning_scene_interface_ = 
//     std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  
//   // Set planning parameters
//   move_group_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
//   move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
//   move_group_->setPlanningTime(5.0);
//   move_group_->setNumPlanningAttempts(10);
  
//   // Define home position (all joints at 0) - FIXED JOINT NAMES
//   home_joint_values_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
//   // Define safe position (joints in safe configuration) - FIXED JOINT NAMES  
//   safe_joint_values_ = {0.0, -0.5, -1.0, 0.0, 1.5, 0.0};
  
//   RCLCPP_INFO(node_->get_logger(), "TrajectoryPlanner initialized");
//   RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", 
//               move_group_->getPlanningFrame().c_str());
//   RCLCPP_INFO(node_->get_logger(), "End effector frame: %s", 
//               move_group_->getEndEffectorLink().c_str());
  
//   // Print available joint names to verify
//   auto joint_names = move_group_->getJointNames();
//   RCLCPP_INFO(node_->get_logger(), "Available joints:");
//   for (const auto& name : joint_names) {
//     RCLCPP_INFO(node_->get_logger(), "  - %s", name.c_str());
//   }

//   // Set up table collision prevention
//   setTableDimensions(1.2, 0.8, 0.02, 0.0, 0.0); // Default table: 1.2m x 0.8m x 2cm
  
//   // Add collision object after short delay to ensure planning scene is ready
//   auto setup_timer = node_->create_wall_timer(
//     std::chrono::seconds(3),
//     [this]() {
//       setupTableCollisionObject();
//       addWorkspaceConstraints();
//     });
  
//   RCLCPP_INFO(node_->get_logger(), "🛡️ Table collision prevention will be set up in 3 seconds...");
// }

// bool TrajectoryPlanner::planPickAndPlace(
//   const geometry_msgs::msg::Pose& pick_pose,
//   const geometry_msgs::msg::Pose& place_pose,
//   double approach_distance,
//   double retreat_distance)
// {
//   RCLCPP_INFO(node_->get_logger(), "Planning pick and place trajectory");
  
//   // 1. Move to approach pose above pick location
//   auto pick_approach = createApproachPose(pick_pose, approach_distance);
//   if (!planToPose(pick_approach)) {
//     RCLCPP_ERROR(node_->get_logger(), "Failed to plan to pick approach pose");
//     return false;
//   }
//   if (!executePlan()) return false;
  
//   // 2. Move down to pick pose
//   if (!planToPose(pick_pose)) {
//     RCLCPP_ERROR(node_->get_logger(), "Failed to plan to pick pose");
//     return false;
//   }
//   if (!executePlan()) return false;
  
//   // TODO: Add gripper close command here
//   RCLCPP_INFO(node_->get_logger(), "Gripper would close here");
//   rclcpp::sleep_for(std::chrono::milliseconds(1000));
  
//   // 3. Retreat from pick location
//   auto pick_retreat = createRetreatPose(pick_pose, retreat_distance);
//   if (!planToPose(pick_retreat)) {
//     RCLCPP_ERROR(node_->get_logger(), "Failed to plan pick retreat");
//     return false;
//   }
//   if (!executePlan()) return false;
  
//   // 4. Move to approach pose above place location
//   auto place_approach = createApproachPose(place_pose, approach_distance);
//   if (!planToPose(place_approach)) {
//     RCLCPP_ERROR(node_->get_logger(), "Failed to plan to place approach pose");
//     return false;
//   }
//   if (!executePlan()) return false;
  
//   // 5. Move down to place pose
//   if (!planToPose(place_pose)) {
//     RCLCPP_ERROR(node_->get_logger(), "Failed to plan to place pose");
//     return false;
//   }
//   if (!executePlan()) return false;
  
//   // TODO: Add gripper open command here
//   RCLCPP_INFO(node_->get_logger(), "Gripper would open here");
//   rclcpp::sleep_for(std::chrono::milliseconds(1000));
  
//   // 6. Retreat from place location
//   auto place_retreat = createRetreatPose(place_pose, retreat_distance);
//   if (!planToPose(place_retreat)) {
//     RCLCPP_ERROR(node_->get_logger(), "Failed to plan place retreat");
//     return false;
//   }
//   if (!executePlan()) return false;
  
//   RCLCPP_INFO(node_->get_logger(), "Pick and place trajectory completed successfully");
//   return true;
// }

// bool TrajectoryPlanner::planToPose(const geometry_msgs::msg::Pose& target_pose)
// {
//   move_group_->setPoseTarget(target_pose);
  
//   bool success = (move_group_->plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  
//   if (success) {
//     RCLCPP_INFO(node_->get_logger(), "Planning successful");
//   } else {
//     RCLCPP_ERROR(node_->get_logger(), "Planning failed");
//   }
  
//   return success;
// }

// bool TrajectoryPlanner::executePlan()
// {
//   auto result = move_group_->execute(current_plan_);
//   bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);
  
//   if (success) {
//     RCLCPP_INFO(node_->get_logger(), "Execution successful");
//   } else {
//     RCLCPP_ERROR(node_->get_logger(), "Execution failed");
//   }
  
//   return success;
// }

// bool TrajectoryPlanner::moveToHome()
// {
//   return planToJointPosition(home_joint_values_) && executePlan();
// }

// bool TrajectoryPlanner::moveToSafePosition()
// {
//   return planToJointPosition(safe_joint_values_) && executePlan();
// }

// bool TrajectoryPlanner::planToJointPosition(const std::vector<double>& joint_values)
// {
//   // Get the correct joint names from MoveIt group
//   auto joint_names = move_group_->getJointNames();
  
//   if (joint_values.size() != joint_names.size()) {
//     RCLCPP_ERROR(node_->get_logger(), 
//                  "Joint values size (%zu) doesn't match joint names size (%zu)", 
//                  joint_values.size(), joint_names.size());
//     return false;
//   }
  
//   // Set joint targets using correct joint names
//   std::map<std::string, double> joint_target;
//   for (size_t i = 0; i < joint_names.size(); ++i) {
//     joint_target[joint_names[i]] = joint_values[i];
//   }
  
//   move_group_->setJointValueTarget(joint_target);
  
//   bool success = (move_group_->plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);
  
//   return success;
// }

// geometry_msgs::msg::Pose TrajectoryPlanner::createApproachPose(
//   const geometry_msgs::msg::Pose& target_pose, double distance)
// {
//   auto approach_pose = target_pose;
//   approach_pose.position.z += distance;
//   return approach_pose;
// }

// geometry_msgs::msg::Pose TrajectoryPlanner::createRetreatPose(
//   const geometry_msgs::msg::Pose& target_pose, double distance)
// {
//   auto retreat_pose = target_pose;
//   retreat_pose.position.z += distance;
//   return retreat_pose;
// }

// void TrajectoryPlanner::setVelocityScaling(double scaling)
// {
//   velocity_scaling_factor_ = scaling;
//   move_group_->setMaxVelocityScalingFactor(scaling);
// }

// void TrajectoryPlanner::setAccelerationScaling(double scaling)
// {
//   acceleration_scaling_factor_ = scaling;
//   move_group_->setMaxAccelerationScalingFactor(scaling);
// }

// void TrajectoryPlanner::setSmoothMotion(bool enable)
// {
//   if (enable) {
//     // Very smooth, safe motion
//     setVelocityScaling(0.2);      // 20% max velocity
//     setAccelerationScaling(0.15); // 15% max acceleration
//     RCLCPP_INFO(node_->get_logger(), "🐌 Smooth motion mode enabled (slow and safe)");
//   } else {
//     // Normal motion
//     setVelocityScaling(0.5);      // 50% max velocity  
//     setAccelerationScaling(0.3);  // 30% max acceleration
//     RCLCPP_INFO(node_->get_logger(), "⚡ Normal motion mode enabled");
//   }
// }

// void TrajectoryPlanner::printCurrentPose()
// {
//   auto current_pose = move_group_->getCurrentPose().pose;
//   RCLCPP_INFO(node_->get_logger(), "Current robot pose:");
//   RCLCPP_INFO(node_->get_logger(), "  Position: [%.3f, %.3f, %.3f]", 
//               current_pose.position.x, current_pose.position.y, current_pose.position.z);
//   RCLCPP_INFO(node_->get_logger(), "  Orientation: [%.3f, %.3f, %.3f, %.3f]",
//               current_pose.orientation.x, current_pose.orientation.y, 
//               current_pose.orientation.z, current_pose.orientation.w);
// }

// void TrajectoryPlanner::waitForMotionComplete(double seconds)
// {
//   RCLCPP_INFO(node_->get_logger(), "⏳ Waiting %.1f seconds for motion to complete...", seconds);
//   std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
// }
// }

// bool TrajectoryPlanner::setupTableCollisionObject()
// {
//   RCLCPP_INFO(node_->get_logger(), "🛡️ Setting up table collision object...");
  
//   // Create collision object for the table surface
//   moveit_msgs::msg::CollisionObject table_object;
//   table_object.header.frame_id = move_group_->getPlanningFrame();
//   table_object.id = "table_surface";
  
//   // Define table as a box
//   shape_msgs::msg::SolidPrimitive primitive;
//   primitive.type = primitive.BOX;
//   primitive.dimensions.resize(3);
//   primitive.dimensions[primitive.BOX_X] = table_dims_.length;  // Length
//   primitive.dimensions[primitive.BOX_Y] = table_dims_.width;   // Width  
//   primitive.dimensions[primitive.BOX_Z] = table_dims_.height;  // Thickness
  
//   // Position the table surface
//   geometry_msgs::msg::Pose table_pose;
//   table_pose.position.x = table_dims_.x_offset;
//   table_pose.position.y = table_dims_.y_offset;
//   table_pose.position.z = table_dims_.z_position; // Below robot base
//   table_pose.orientation.w = 1.0;
  
//   table_object.primitives.push_back(primitive);
//   table_object.primitive_poses.push_back(table_pose);
//   table_object.operation = table_object.ADD;
  
//   // Add table to planning scene
//   std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
//   collision_objects.push_back(table_object);
  
//   bool success = planning_scene_interface_->addCollisionObjects(collision_objects);
  
//   if (success) {
//     RCLCPP_INFO(node_->get_logger(), "✅ Table collision object added successfully");
//     RCLCPP_INFO(node_->get_logger(), "   Table dimensions: %.2f x %.2f x %.2f m", 
//                 table_dims_.length, table_dims_.width, table_dims_.height);
//     RCLCPP_INFO(node_->get_logger(), "   Table position: [%.3f, %.3f, %.3f] m", 
//                 table_dims_.x_offset, table_dims_.y_offset, table_dims_.z_position);
    
//     // Wait for planning scene to update
//     rclcpp::sleep_for(std::chrono::seconds(1));
//     table_collision_setup_ = true;
//   } else {
//     RCLCPP_ERROR(node_->get_logger(), "❌ Failed to add table collision object");
//   }
  
//   return success;
// }

// void TrajectoryPlanner::setTableDimensions(double length, double width, double height,
//                                           double x_offset, double y_offset)
// {
//   table_dims_.length = length;
//   table_dims_.width = width;
//   table_dims_.height = height;
//   table_dims_.x_offset = x_offset;
//   table_dims_.y_offset = y_offset;
  
//   RCLCPP_INFO(node_->get_logger(), "🔧 Table dimensions updated:");
//   RCLCPP_INFO(node_->get_logger(), "   Size: %.2f x %.2f x %.2f m", length, width, height);
//   RCLCPP_INFO(node_->get_logger(), "   Offset: [%.3f, %.3f] m", x_offset, y_offset);
// }

// bool TrajectoryPlanner::addWorkspaceConstraints() 
// {
//   RCLCPP_INFO(node_->get_logger(), "📐 Adding workspace constraints...");
  
//   // Add position constraints for critical links to stay above table
//   moveit_msgs::msg::Constraints workspace_constraints;
  
//   // Get robot model to identify critical links
//   auto robot_model = move_group_->getRobotModel();
//   auto joint_group = robot_model->getJointModelGroup("mainpulation");
//   auto link_names = joint_group->getLinkModelNames();
  
//   // Add position constraints for links that could collide with table
//   // Focus on elbow and wrist links
//   std::vector<std::string> critical_links = {"link3", "link4", "link5"}; // Adjust based on your URDF
  
//   for (const auto& link_name : critical_links) {
//     // Check if link exists in robot model
//     if (std::find(link_names.begin(), link_names.end(), link_name) != link_names.end()) {
      
//       moveit_msgs::msg::PositionConstraint pos_constraint;
//       pos_constraint.header.frame_id = move_group_->getPlanningFrame();
//       pos_constraint.link_name = link_name;
      
//       // Create constraint region (above table)
//       shape_msgs::msg::SolidPrimitive constraint_region;
//       constraint_region.type = constraint_region.BOX;
//       constraint_region.dimensions = {2.0, 2.0, 1.0}; // Large working volume above table
      
//       geometry_msgs::msg::Pose constraint_pose;
//       constraint_pose.position.x = 0.0;
//       constraint_pose.position.y = 0.0; 
//       constraint_pose.position.z = 0.5;  // 50cm above robot base
//       constraint_pose.orientation.w = 1.0;
      
//       pos_constraint.constraint_region.primitives.push_back(constraint_region);
//       pos_constraint.constraint_region.primitive_poses.push_back(constraint_pose);
//       pos_constraint.weight = 1.0;
      
//       workspace_constraints.position_constraints.push_back(pos_constraint);
      
//       RCLCPP_INFO(node_->get_logger(), "   Added constraint for link: %s", link_name.c_str());
//     }
//   }
  
//   // Apply constraints to move group
//   move_group_->setPathConstraints(workspace_constraints);
  
//   RCLCPP_INFO(node_->get_logger(), "✅ Workspace constraints applied");
//   return true;
// }




// ------------------------------------------
#include "trajectory_plan/trajectory_planner.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <algorithm>

namespace trajectory_plan
{
TrajectoryPlanner::TrajectoryPlanner(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  , velocity_scaling_factor_(0.3)
  , acceleration_scaling_factor_(0.3)
{
  // Initialize MoveIt interfaces
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_, "mainpulation"); // Using the planning group from SRDF
  
  planning_scene_interface_ = 
    std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  
  // Set planning parameters
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
  move_group_->setPlanningTime(5.0);
  move_group_->setNumPlanningAttempts(10);
  
  // Define home position (all joints at 0) - FIXED JOINT NAMES
  home_joint_values_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  // Define safe position (joints in safe configuration) - FIXED JOINT NAMES  
  safe_joint_values_ = {0.0, -0.5, -1.0, 0.0, 1.5, 0.0};
  
  RCLCPP_INFO(node_->get_logger(), "TrajectoryPlanner initialized");
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

  // Set up table collision prevention
//   setTableDimensions(1.2, 0.8, 0.02, 0.0, 0.0); // Default table: 1.2m x 0.8m x 2cm
    setTableDimensions(1.2, 0.8, 0.08, 0.0, 0.0);
  
  // Add collision object after short delay to ensure planning scene is ready
  auto setup_timer = node_->create_wall_timer(
    std::chrono::seconds(3),
    [this]() {
      setupTableCollisionObject();
      addWorkspaceConstraints();
    });
  
  RCLCPP_INFO(node_->get_logger(), "🛡️ Table collision prevention will be set up in 3 seconds...");
}

bool TrajectoryPlanner::planPickAndPlace(
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

bool TrajectoryPlanner::planToPose(const geometry_msgs::msg::Pose& target_pose)
{
  // Ensure table collision object is set up
  if (!table_collision_setup_) {
    RCLCPP_WARN(node_->get_logger(), "⚠️ Table collision object not set up, setting up now...");
    setupTableCollisionObject();
  }
  
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

bool TrajectoryPlanner::executePlan()
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

bool TrajectoryPlanner::moveToHome()
{
  return planToJointPosition(home_joint_values_) && executePlan();
}

bool TrajectoryPlanner::moveToSafePosition()
{
  return planToJointPosition(safe_joint_values_) && executePlan();
}

bool TrajectoryPlanner::planToJointPosition(const std::vector<double>& joint_values)
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

geometry_msgs::msg::Pose TrajectoryPlanner::createApproachPose(
  const geometry_msgs::msg::Pose& target_pose, double distance)
{
  auto approach_pose = target_pose;
  approach_pose.position.z += distance;
  return approach_pose;
}

geometry_msgs::msg::Pose TrajectoryPlanner::createRetreatPose(
  const geometry_msgs::msg::Pose& target_pose, double distance)
{
  auto retreat_pose = target_pose;
  retreat_pose.position.z += distance;
  return retreat_pose;
}

void TrajectoryPlanner::setVelocityScaling(double scaling)
{
  velocity_scaling_factor_ = scaling;
  move_group_->setMaxVelocityScalingFactor(scaling);
}

void TrajectoryPlanner::setAccelerationScaling(double scaling)
{
  acceleration_scaling_factor_ = scaling;
  move_group_->setMaxAccelerationScalingFactor(scaling);
}

void TrajectoryPlanner::setSmoothMotion(bool enable)
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

void TrajectoryPlanner::printCurrentPose()
{
  auto current_pose = move_group_->getCurrentPose().pose;
  RCLCPP_INFO(node_->get_logger(), "Current robot pose:");
  RCLCPP_INFO(node_->get_logger(), "  Position: [%.3f, %.3f, %.3f]", 
              current_pose.position.x, current_pose.position.y, current_pose.position.z);
  RCLCPP_INFO(node_->get_logger(), "  Orientation: [%.3f, %.3f, %.3f, %.3f]",
              current_pose.orientation.x, current_pose.orientation.y, 
              current_pose.orientation.z, current_pose.orientation.w);
}

void TrajectoryPlanner::waitForMotionComplete(double seconds)
{
  RCLCPP_INFO(node_->get_logger(), "⏳ Waiting %.1f seconds for motion to complete...", seconds);
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
}

// NEW METHODS - Now properly inside the namespace
bool TrajectoryPlanner::setupTableCollisionObject()
{
  RCLCPP_INFO(node_->get_logger(), "🛡️ Setting up table collision object...");
  
  // Create collision object for the table surface
  moveit_msgs::msg::CollisionObject table_object;
  table_object.header.frame_id = move_group_->getPlanningFrame();
  table_object.id = "table_surface";
  
  // Define table as a box
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = table_dims_.length;  // Length
  primitive.dimensions[primitive.BOX_Y] = table_dims_.width;   // Width  
  primitive.dimensions[primitive.BOX_Z] = table_dims_.height;  // Thickness
  
  // Position the table surface
  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = table_dims_.x_offset;
  table_pose.position.y = table_dims_.y_offset;
  table_pose.position.z = table_dims_.z_position; // Below robot base
  table_pose.orientation.w = 1.0;
  
  table_object.primitives.push_back(primitive);
  table_object.primitive_poses.push_back(table_pose);
  table_object.operation = table_object.ADD;
  
  // Add table to planning scene
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(table_object);
  
  // Add collision objects (returns void in ROS 2 Humble)
  planning_scene_interface_->addCollisionObjects(collision_objects);
  
  // Assume success since no exception was thrown
  RCLCPP_INFO(node_->get_logger(), "✅ Table collision object added successfully");
  RCLCPP_INFO(node_->get_logger(), "   Table dimensions: %.2f x %.2f x %.2f m", 
              table_dims_.length, table_dims_.width, table_dims_.height);
  RCLCPP_INFO(node_->get_logger(), "   Table position: [%.3f, %.3f, %.3f] m", 
              table_dims_.x_offset, table_dims_.y_offset, table_dims_.z_position);
  
  // Wait for planning scene to update
  rclcpp::sleep_for(std::chrono::seconds(1));
  table_collision_setup_ = true;
  
  return true;
}

// void TrajectoryPlanner::setTableDimensions(double length, double width, double height,
//                                           double x_offset, double y_offset)
// {
//   table_dims_.length = length;
//   table_dims_.width = width;
//   table_dims_.height = height;
//   table_dims_.x_offset = x_offset;
//   table_dims_.y_offset = y_offset;
  
//   RCLCPP_INFO(node_->get_logger(), "🔧 Table dimensions updated:");
//   RCLCPP_INFO(node_->get_logger(), "   Size: %.2f x %.2f x %.2f m", length, width, height);
//   RCLCPP_INFO(node_->get_logger(), "   Offset: [%.3f, %.3f] m", x_offset, y_offset);
// }

// void TrajectoryPlanner::setTableDimensions(double length, double width, double height,
//                                           double x_offset, double y_offset)
// {
//   table_dims_.length = length;
//   table_dims_.width = width;
//   table_dims_.height = height;
//   table_dims_.x_offset = x_offset;
//   table_dims_.y_offset = y_offset;
  
//   // CRITICAL FIX: Table surface is AT robot base level for mounted table
//   table_dims_.z_position = height / 2.0;  // Table center is at half-thickness above base
  
//   RCLCPP_INFO(node_->get_logger(), "🔧 MOUNTED Table dimensions updated:");
//   RCLCPP_INFO(node_->get_logger(), "   Size: %.2f x %.2f x %.2f m", length, width, height);
//   RCLCPP_INFO(node_->get_logger(), "   Position: Table surface AT robot base level");
//   RCLCPP_INFO(node_->get_logger(), "   Objects can be placed from %.1f mm height", (height * 1000.0));
// }

// REPLACE setTableDimensions method in trajectory_planner.cpp:

void TrajectoryPlanner::setTableDimensions(double length, double width, double height,
    double x_offset, double y_offset)
{
table_dims_.length = length;
table_dims_.width = width;
// table_dims_.height = height;
table_dims_.height = 0.05;
table_dims_.x_offset = x_offset;
table_dims_.y_offset = y_offset;

// CRITICAL FIX: Table is mounted TO the robot base
// The top surface of the table is AT the robot base level (Z=0)
// So the collision object center should be BELOW the base level
// table_dims_.z_position = -height / 2.0;  // Table center is below base level
table_dims_.z_position = -0.027;

RCLCPP_INFO(node_->get_logger(), "🔧 MOUNTED Table dimensions updated:");
RCLCPP_INFO(node_->get_logger(), "   Size: %.2f x %.2f x %.2f m", length, width, height);
RCLCPP_INFO(node_->get_logger(), "   Collision center: [%.3f, %.3f, %.3f] m", 
x_offset, y_offset, table_dims_.z_position);
RCLCPP_INFO(node_->get_logger(), "   Table surface: AT Z=0 (robot base level)");
RCLCPP_INFO(node_->get_logger(), "   Objects can be placed from %.1f mm height", 5.0); // 5mm minimum
}

// Enhanced pick and place for mounted table with elbow control
// REPLACE the planMountedTablePickAndPlace method in trajectory_planner.cpp with this:

bool TrajectoryPlanner::planMountedTablePickAndPlace(
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

bool TrajectoryPlanner::addWorkspaceConstraints() 
{
  RCLCPP_INFO(node_->get_logger(), "📐 Adding workspace constraints...");
  
  // Add position constraints for critical links to stay above table
  moveit_msgs::msg::Constraints workspace_constraints;
  
  // Get robot model to identify critical links
  auto robot_model = move_group_->getRobotModel();
  auto joint_group = robot_model->getJointModelGroup("mainpulation");
  auto link_names = joint_group->getLinkModelNames();
  
  // Add position constraints for links that could collide with table
  // Focus on elbow and wrist links
  std::vector<std::string> critical_links = {"link3", "link4", "link5"}; // Adjust based on your URDF
  
  for (const auto& link_name : critical_links) {
    // Check if link exists in robot model
    if (std::find(link_names.begin(), link_names.end(), link_name) != link_names.end()) {
      
      moveit_msgs::msg::PositionConstraint pos_constraint;
      pos_constraint.header.frame_id = move_group_->getPlanningFrame();
      pos_constraint.link_name = link_name;
      
      // Create constraint region (above table)
      shape_msgs::msg::SolidPrimitive constraint_region;
      constraint_region.type = constraint_region.BOX;
      constraint_region.dimensions = {2.0, 2.0, 1.0}; // Large working volume above table
      
      geometry_msgs::msg::Pose constraint_pose;
      constraint_pose.position.x = 0.0;
      constraint_pose.position.y = 0.0; 
      constraint_pose.position.z = 0.0;  // 50cm above robot base
      constraint_pose.orientation.w = 1.0;
      
      pos_constraint.constraint_region.primitives.push_back(constraint_region);
      pos_constraint.constraint_region.primitive_poses.push_back(constraint_pose);
      pos_constraint.weight = 1.0;
      
      workspace_constraints.position_constraints.push_back(pos_constraint);
      
      RCLCPP_INFO(node_->get_logger(), "   Added constraint for link: %s", link_name.c_str());
    }
  }
  
  // Apply constraints to move group
  move_group_->setPathConstraints(workspace_constraints);
  
  RCLCPP_INFO(node_->get_logger(), "✅ Workspace constraints applied");
  return true;
}

// bool TrajectoryPlanner::addWorkspaceConstraints() 
// {
//   RCLCPP_INFO(node_->get_logger(), "📐 SKIPPING workspace constraints for testing...");
  
//   // TEMPORARILY DISABLED - Comment out the constraint application
//   /*
//   // Apply constraints to move group
//   move_group_->setPathConstraints(workspace_constraints);
//   */
  
//   RCLCPP_INFO(node_->get_logger(), "⚠️  Workspace constraints DISABLED for testing");
//   RCLCPP_INFO(node_->get_logger(), "    This allows testing low positions without constraint interference");
//   return true;
// }

bool TrajectoryPlanner::validateTrajectoryAgainstTable(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  // Suppress unused parameter warning
  (void)trajectory;
  
  // For now, just return true - the collision object will handle validation
  // This method can be enhanced later for additional safety checks
  RCLCPP_INFO(node_->get_logger(), "🔍 Trajectory validation - relying on collision objects");
  return true;
}

} // namespace trajectory_plan - IMPORTANT: Keep this closing brace here!