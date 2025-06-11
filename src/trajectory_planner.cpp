#include "trajectory_plan/trajectory_planner.hpp"
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>

namespace trajectory_plan
{

TrajectoryPlanner::TrajectoryPlanner(const rclcpp::Node::SharedPtr& node)
  : node_(node), initialized_(false)
{
  RCLCPP_INFO(node_->get_logger(), "🚀 TrajectoryPlanner created. Call initialize() to setup.");
  config_ = PlannerConfiguration{};
}

bool TrajectoryPlanner::initialize()
{
  if (initialized_) {
    RCLCPP_WARN(node_->get_logger(), "TrajectoryPlanner already initialized.");
    return true;
  }
  
  RCLCPP_INFO(node_->get_logger(), "🔧 Starting SYNCHRONOUS TrajectoryPlanner initialization...");

  // 1. Initialize MoveIt Interfaces
  try {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "mainpulation");
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    RCLCPP_INFO(node_->get_logger(), "✅ [1/4] MoveIt interfaces initialized.");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "❌ [1/4] Failed to initialize MoveIt: %s", e.what());
    return false;
  }

  // 2. Initialize RobotOperation
  try {
    robot_operation_ = std::make_unique<RobotOperation>(node_, move_group_);
    robot_operation_->setVelocityScaling(config_.robot.velocity_scaling);
    robot_operation_->setAccelerationScaling(config_.robot.acceleration_scaling);
    RCLCPP_INFO(node_->get_logger(), "✅ [2/4] RobotOperation initialized.");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "❌ [2/4] Failed to initialize RobotOperation: %s", e.what());
    return false;
  }

  // 3. Setup Collision Scene
  if (!setupCollisionScene()) {
      RCLCPP_ERROR(node_->get_logger(), "❌ [3/4] Failed to setup collision scene.");
      return false;
  }
  RCLCPP_INFO(node_->get_logger(), "✅ [3/4] Collision scene configured.");

  // 4. Add Workspace Constraints
  if (!addWorkspaceConstraints()) {
      RCLCPP_ERROR(node_->get_logger(), "❌ [4/4] Failed to add workspace constraints.");
      return false;
  }
  RCLCPP_INFO(node_->get_logger(), "✅ [4/4] Workspace constraints applied.");
  
  // Optional: Initialize Visualization
  if (config_.workspace.visualization_enabled) {
      enableWorkspaceVisualization(true);
  }

  initialized_ = true;
  RCLCPP_INFO(node_->get_logger(), "✅ TrajectoryPlanner initialized successfully.");
  return true;
}

void TrajectoryPlanner::updateConfiguration(const PlannerConfiguration& config)
{
  config_ = config;
  if (robot_operation_) {
    robot_operation_->setVelocityScaling(config_.robot.velocity_scaling);
    robot_operation_->setAccelerationScaling(config_.robot.acceleration_scaling);
  }
  // Re-apply scene and constraints if already initialized
  if(initialized_) {
      setupCollisionScene();
      addWorkspaceConstraints();
  }
  RCLCPP_INFO(node_->get_logger(), "🔧 Configuration updated.");
}

bool TrajectoryPlanner::executeProfessionalPickAndPlace(
  double pick_x_mm, double pick_y_mm, double pick_z_mm,
  double pick_roll_deg, double pick_pitch_deg, double pick_yaw_deg,
  double place_x_mm, double place_y_mm, double place_z_mm,
  double place_roll_deg, double place_pitch_deg, double place_yaw_deg,
  double clearance_height_mm)
{
  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(), "❌ TrajectoryPlanner not initialized.");
    return false;
  }
  
  // Pre-flight check: Validate poses against workspace before attempting to move
  auto pick_pose_for_validation = robot_operation_->createPoseFromMmAndDegrees(pick_x_mm, pick_y_mm, pick_z_mm, 0,0,0);
  auto place_pose_for_validation = robot_operation_->createPoseFromMmAndDegrees(place_x_mm, place_y_mm, place_z_mm, 0,0,0);
  if (!validatePoseInWorkspace(pick_pose_for_validation) || !validatePoseInWorkspace(place_pose_for_validation)) {
      RCLCPP_ERROR(node_->get_logger(), "❌ Aborting: One or more poses are outside the workspace.");
      return false;
  }

  // Delegate the entire sequence to RobotOperation
  return robot_operation_->executeProfessionalPickAndPlace(
    pick_x_mm, pick_y_mm, pick_z_mm, pick_roll_deg, pick_pitch_deg, pick_yaw_deg,
    place_x_mm, place_y_mm, place_z_mm, place_roll_deg, place_pitch_deg, place_yaw_deg,
    clearance_height_mm
  );
}

// --- Delegated Methods ---
bool TrajectoryPlanner::moveToHome() { return robot_operation_ ? robot_operation_->moveToHome() : false; }
bool TrajectoryPlanner::moveToSafePosition() { return robot_operation_ ? robot_operation_->moveToSafePosition() : false; }
void TrajectoryPlanner::setSmoothMotion(bool enable) { if (robot_operation_) robot_operation_->setSmoothMotion(enable); }
void TrajectoryPlanner::printCurrentPose() { if (robot_operation_) robot_operation_->checkAndPrintCurrentPose(); }

// --- Scene Management ---
bool TrajectoryPlanner::setupCollisionScene()
{
  RCLCPP_INFO(node_->get_logger(), "🛡️ Setting up table as collision object...");
  
  moveit_msgs::msg::CollisionObject table_object;
  table_object.header.frame_id = move_group_->getPlanningFrame();
  table_object.id = "table_surface";
  
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {config_.table.length, config_.table.width, config_.table.height};
  
  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = config_.table.x_offset;
  table_pose.position.y = config_.table.y_offset;
  table_pose.position.z = config_.table.z_position;
  table_pose.orientation.w = 1.0;
  
  table_object.primitives.push_back(primitive);
  table_object.primitive_poses.push_back(table_pose);
  table_object.operation = table_object.ADD;
  
  // [CRITICAL IMPROVEMENT] Use applyCollisionObjects for a synchronous update.
  // This blocks until the planning scene has processed the new object.
  return planning_scene_interface_->applyCollisionObjects({table_object});
}

bool TrajectoryPlanner::addWorkspaceConstraints()
{
    RCLCPP_INFO(node_->get_logger(), "📐 Applying workspace constraints...");
    
    moveit_msgs::msg::PositionConstraint pcm;
    pcm.header.frame_id = move_group_->getPlanningFrame();
    pcm.link_name = move_group_->getEndEffectorLink(); // Constrain the tool tip
    
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {config_.workspace.width, config_.workspace.depth, config_.workspace.height};
    
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = config_.workspace.x_position;
    box_pose.position.y = config_.workspace.y_position;
    box_pose.position.z = config_.workspace.z_position;
    box_pose.orientation.w = 1.0;

    pcm.constraint_region.primitives.push_back(box);
    pcm.constraint_region.primitive_poses.push_back(box_pose);
    pcm.weight = 1.0; // This is a hard constraint

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.position_constraints.push_back(pcm);
    
    move_group_->setPathConstraints(path_constraints);
    
    printWorkspaceLimits();
    return true;
}

// --- Validation and Visualization ---
bool TrajectoryPlanner::validatePoseInWorkspace(const geometry_msgs::msg::Pose& pose)
{
  double x_min = config_.workspace.x_position - config_.workspace.width / 2.0;
  double x_max = config_.workspace.x_position + config_.workspace.width / 2.0;
  double y_min = config_.workspace.y_position - config_.workspace.depth / 2.0;
  double y_max = config_.workspace.y_position + config_.workspace.depth / 2.0;
  double z_min = config_.workspace.z_position - config_.workspace.height / 2.0;
  double z_max = config_.workspace.z_position + config_.workspace.height / 2.0;

  bool valid = (pose.position.x >= x_min && pose.position.x <= x_max &&
                pose.position.y >= y_min && pose.position.y <= y_max &&
                pose.position.z >= z_min && pose.position.z <= z_max);

  if (!valid) {
    RCLCPP_WARN(node_->get_logger(), "❌ Pose [%.3f, %.3f, %.3f] is outside the defined workspace.", 
                 pose.position.x, pose.position.y, pose.position.z);
  } else {
    RCLCPP_INFO(node_->get_logger(), "✅ Pose [%.3f, %.3f, %.3f] is within the workspace.", 
                pose.position.x, pose.position.y, pose.position.z);
  }
  return valid;
}

void TrajectoryPlanner::enableWorkspaceVisualization(bool enable)
{
  if (enable && !marker_publisher_) {
    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("workspace_boundary", 10);
    visualization_timer_ = node_->create_wall_timer(std::chrono::seconds(2), [this](){ this->publishWorkspaceBoundary(); });
    RCLCPP_INFO(node_->get_logger(), "🔳 Workspace visualization ENABLED.");
  } else if (!enable && marker_publisher_) {
    if(visualization_timer_) visualization_timer_->cancel();
    marker_publisher_.reset(); // Release the publisher
    RCLCPP_INFO(node_->get_logger(), "🔲 Workspace visualization DISABLED.");
  }
}

void TrajectoryPlanner::publishWorkspaceBoundary()
{
    if (!marker_publisher_) return;

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker box_marker;
    box_marker.header.frame_id = move_group_->getPlanningFrame();
    box_marker.header.stamp = node_->now();
    box_marker.ns = "workspace_boundary";
    box_marker.id = 0;
    box_marker.type = visualization_msgs::msg::Marker::CUBE;
    box_marker.action = visualization_msgs::msg::Marker::ADD;

    box_marker.pose.position.x = config_.workspace.x_position;
    box_marker.pose.position.y = config_.workspace.y_position;
    box_marker.pose.position.z = config_.workspace.z_position;
    box_marker.pose.orientation.w = 1.0;

    box_marker.scale.x = config_.workspace.width;
    box_marker.scale.y = config_.workspace.depth;
    box_marker.scale.z = config_.workspace.height;

    box_marker.color.r = 1.0f; // Red
    box_marker.color.g = 0.0f;
    box_marker.color.b = 0.0f;
    box_marker.color.a = 0.2f; // Semi-transparent

    marker_array.markers.push_back(box_marker);
    marker_publisher_->publish(marker_array);
}

void TrajectoryPlanner::printWorkspaceLimits()
{
  double x_min = config_.workspace.x_position - config_.workspace.width / 2.0;
  double x_max = config_.workspace.x_position + config_.workspace.width / 2.0;
  double y_min = config_.workspace.y_position - config_.workspace.depth / 2.0;
  double y_max = config_.workspace.y_position + config_.workspace.depth / 2.0;
  double z_min = config_.workspace.z_position - config_.workspace.height / 2.0;
  double z_max = config_.workspace.z_position + config_.workspace.height / 2.0;
  
  RCLCPP_INFO(node_->get_logger(), "📐 Workspace Limits: X[%.3f, %.3f], Y[%.3f, %.3f], Z[%.3f, %.3f]",
              x_min, x_max, y_min, y_max, z_min, z_max);
}

} // namespace trajectory_plan
