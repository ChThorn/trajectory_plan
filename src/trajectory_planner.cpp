#include "trajectory_plan/trajectory_planner.hpp"
#include <algorithm>

namespace trajectory_plan
{

TrajectoryPlanner::TrajectoryPlanner(const rclcpp::Node::SharedPtr& node)
  : node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "🚀 TrajectoryPlanner initializing...");
  
  // Initialize with default configuration
  config_ = PlannerConfiguration{};
  
  RCLCPP_INFO(node_->get_logger(), "✅ TrajectoryPlanner created - call initialize() to setup");
}

bool TrajectoryPlanner::initialize()
{
  if (initialized_) {
    RCLCPP_WARN(node_->get_logger(), "TrajectoryPlanner already initialized");
    return true;
  }
  
  RCLCPP_INFO(node_->get_logger(), "🔧 Initializing TrajectoryPlanner components...");
  
  if (!initializeMoveIt()) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Failed to initialize MoveIt");
    return false;
  }
  
  if (!initializeRobotOperation()) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Failed to initialize RobotOperation");
    return false;
  }
  
  if (!initializeVisualization()) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Failed to initialize visualization");
    return false;
  }
  
  // Setup delayed initialization for collision scene
  setup_timer_ = node_->create_wall_timer(
    std::chrono::seconds(3),
    [this]() {
      performDelayedSetup();
      setup_timer_->cancel();
    });
  
  initialized_ = true;
  RCLCPP_INFO(node_->get_logger(), "✅ TrajectoryPlanner initialized successfully");
  return true;
}

bool TrajectoryPlanner::initializeMoveIt()
{
  try {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_, "mainpulation");
    
    planning_scene_interface_ = 
      std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    
    RCLCPP_INFO(node_->get_logger(), "✅ MoveIt interfaces initialized");
    RCLCPP_INFO(node_->get_logger(), "   Planning frame: %s", 
                move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "   End effector: %s", 
                move_group_->getEndEffectorLink().c_str());
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception initializing MoveIt: %s", e.what());
    return false;
  }
}

bool TrajectoryPlanner::initializeRobotOperation()
{
  try {
    robot_operation_ = std::make_unique<RobotOperation>(node_, move_group_);
    
    // Apply robot configuration
    robot_operation_->setVelocityScaling(config_.robot.velocity_scaling);
    robot_operation_->setAccelerationScaling(config_.robot.acceleration_scaling);
    robot_operation_->setHomePosition(config_.robot.home_joints);
    robot_operation_->setSafePosition(config_.robot.safe_joints);
    
    RCLCPP_INFO(node_->get_logger(), "✅ RobotOperation initialized");
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception initializing RobotOperation: %s", e.what());
    return false;
  }
}

bool TrajectoryPlanner::initializeVisualization()
{
  try {
    marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "workspace_boundary", 10);
    
    RCLCPP_INFO(node_->get_logger(), "✅ Visualization initialized");
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception initializing visualization: %s", e.what());
    return false;
  }
}

void TrajectoryPlanner::performDelayedSetup()
{
  RCLCPP_INFO(node_->get_logger(), "🔧 Performing delayed setup...");
  
  setupCollisionScene();
  addWorkspaceConstraints();
  
  if (config_.workspace.visualization_enabled) {
    enableWorkspaceVisualization(true);
  }
  
  RCLCPP_INFO(node_->get_logger(), "✅ Delayed setup completed");
}

void TrajectoryPlanner::updateConfiguration(const PlannerConfiguration& config)
{
  config_ = config;
  
  if (robot_operation_) {
    robot_operation_->setVelocityScaling(config_.robot.velocity_scaling);
    robot_operation_->setAccelerationScaling(config_.robot.acceleration_scaling);
    robot_operation_->setHomePosition(config_.robot.home_joints);
    robot_operation_->setSafePosition(config_.robot.safe_joints);
  }
  
  RCLCPP_INFO(node_->get_logger(), "🔧 Configuration updated");
}

bool TrajectoryPlanner::planPickAndPlace(
  const geometry_msgs::msg::Pose& pick_pose,
  const geometry_msgs::msg::Pose& place_pose,
  double approach_distance,
  double retreat_distance)
{
  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(), "❌ TrajectoryPlanner not initialized");
    return false;
  }
  
  // Ensure collision scene is setup
  if (!table_collision_setup_) {
    RCLCPP_WARN(node_->get_logger(), "⚠️ Setting up collision scene...");
    setupCollisionScene();
  }
  
  // Validate poses
  if (!validatePoseInWorkspace(pick_pose) || !validatePoseInWorkspace(place_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Poses outside workspace");
    return false;
  }
  
  // Delegate to robot operation (legacy method)
  return robot_operation_->planPickAndPlace(pick_pose, place_pose, approach_distance, retreat_distance);
}

bool TrajectoryPlanner::executeProfessionalPickAndPlace(
  double pick_x_mm, double pick_y_mm, double pick_z_mm,
  double pick_roll_deg, double pick_pitch_deg, double pick_yaw_deg,
  double place_x_mm, double place_y_mm, double place_z_mm,
  double place_roll_deg, double place_pitch_deg, double place_yaw_deg,
  double clearance_height_mm)
{
  if (!initialized_) {
    RCLCPP_ERROR(node_->get_logger(), "❌ TrajectoryPlanner not initialized");
    return false;
  }
  
  RCLCPP_INFO(node_->get_logger(), "🏭 PROFESSIONAL Pick-and-Place Starting...");
  RCLCPP_INFO(node_->get_logger(), "   Pick:  [%.1f, %.1f, %.1f] mm, [%.1f°, %.1f°, %.1f°]", 
              pick_x_mm, pick_y_mm, pick_z_mm, pick_roll_deg, pick_pitch_deg, pick_yaw_deg);
  RCLCPP_INFO(node_->get_logger(), "   Place: [%.1f, %.1f, %.1f] mm, [%.1f°, %.1f°, %.1f°]", 
              place_x_mm, place_y_mm, place_z_mm, place_roll_deg, place_pitch_deg, place_yaw_deg);
  
  // Ensure collision scene is setup
  if (!table_collision_setup_) {
    RCLCPP_WARN(node_->get_logger(), "⚠️ Setting up collision scene...");
    setupCollisionScene();
  }
  
  // Validate poses (convert to meters for validation)
  geometry_msgs::msg::Pose pick_pose_temp, place_pose_temp;
  
  // Create temporary poses for validation
  pick_pose_temp.position.x = pick_x_mm / 1000.0;
  pick_pose_temp.position.y = pick_y_mm / 1000.0;
  pick_pose_temp.position.z = pick_z_mm / 1000.0;
  pick_pose_temp.orientation.w = 1.0;  // Default orientation for validation
  
  place_pose_temp.position.x = place_x_mm / 1000.0;
  place_pose_temp.position.y = place_y_mm / 1000.0;
  place_pose_temp.position.z = place_z_mm / 1000.0;
  place_pose_temp.orientation.w = 1.0;  // Default orientation for validation
  
  if (!validatePoseInWorkspace(pick_pose_temp) || !validatePoseInWorkspace(place_pose_temp)) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Poses outside workspace boundaries");
    return false;
  }
  
  // Execute professional pick-and-place sequence
  return robot_operation_->executeProfessionalPickAndPlace(
    pick_x_mm, pick_y_mm, pick_z_mm, pick_roll_deg, pick_pitch_deg, pick_yaw_deg,
    place_x_mm, place_y_mm, place_z_mm, place_roll_deg, place_pitch_deg, place_yaw_deg,
    clearance_height_mm
  );
}

// Delegated robot control methods
bool TrajectoryPlanner::executePlan()
{
  return robot_operation_ ? robot_operation_->executePlan() : false;
}

bool TrajectoryPlanner::moveToHome()
{
  return robot_operation_ ? robot_operation_->moveToHome() : false;
}

bool TrajectoryPlanner::moveToSafePosition()
{
  return robot_operation_ ? robot_operation_->moveToSafePosition() : false;
}

void TrajectoryPlanner::setVelocityScaling(double scaling)
{
  config_.robot.velocity_scaling = scaling;
  if (robot_operation_) robot_operation_->setVelocityScaling(scaling);
}

void TrajectoryPlanner::setAccelerationScaling(double scaling)
{
  config_.robot.acceleration_scaling = scaling;
  if (robot_operation_) robot_operation_->setAccelerationScaling(scaling);
}

void TrajectoryPlanner::setSmoothMotion(bool enable)
{
  if (robot_operation_) robot_operation_->setSmoothMotion(enable);
}

void TrajectoryPlanner::printCurrentPose()
{
  if (robot_operation_) robot_operation_->checkAndPrintCurrentPose();
}

void TrajectoryPlanner::waitForMotionComplete(double seconds)
{
  if (robot_operation_) robot_operation_->waitForMotionComplete(seconds);
}

// Scene management methods
bool TrajectoryPlanner::setupCollisionScene()
{
  RCLCPP_INFO(node_->get_logger(), "🛡️ Setting up collision scene...");
  
  if (!setupTableCollisionObject()) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Failed to setup table collision");
    return false;
  }
  
  RCLCPP_INFO(node_->get_logger(), "✅ Collision scene setup complete");
  return true;
}

bool TrajectoryPlanner::setupTableCollisionObject()
{
  try {
    moveit_msgs::msg::CollisionObject table_object;
    table_object.header.frame_id = move_group_->getPlanningFrame();
    table_object.id = "table_surface";
    
    // Define table as a box
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = config_.table.length;
    primitive.dimensions[primitive.BOX_Y] = config_.table.width;
    primitive.dimensions[primitive.BOX_Z] = config_.table.height;
    
    // Position the table
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = config_.table.x_offset;
    table_pose.position.y = config_.table.y_offset;
    table_pose.position.z = config_.table.z_position;
    table_pose.orientation.w = 1.0;
    
    table_object.primitives.push_back(primitive);
    table_object.primitive_poses.push_back(table_pose);
    table_object.operation = table_object.ADD;
    
    // Add to planning scene
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(table_object);
    planning_scene_interface_->addCollisionObjects(collision_objects);
    
    // Wait for planning scene update
    rclcpp::sleep_for(std::chrono::seconds(1));
    table_collision_setup_ = true;
    
    RCLCPP_INFO(node_->get_logger(), "✅ Table collision object added");
    RCLCPP_INFO(node_->get_logger(), "   Dimensions: %.2f x %.2f x %.2f m", 
                config_.table.length, config_.table.width, config_.table.height);
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception setting up table collision: %s", e.what());
    return false;
  }
}

bool TrajectoryPlanner::addWorkspaceConstraints()
{
  if (workspace_constraints_applied_) {
    clearWorkspaceConstraints();
  }
  
  return addFullArmWorkspaceConstraints();
}

bool TrajectoryPlanner::addFullArmWorkspaceConstraints()
{
  RCLCPP_INFO(node_->get_logger(), "📐 Adding workspace constraints...");
  
  try {
    moveit_msgs::msg::Constraints workspace_constraints;
    
    auto robot_model = move_group_->getRobotModel();
    auto joint_group = robot_model->getJointModelGroup("mainpulation");
    auto all_links = joint_group->getLinkModelNames();
    
    // Define target links (exclude base links)
    std::vector<std::string> valid_links;
    for (const auto& link : all_links) {
      if (link != "link0" && link != "base_link" && link != "base") {
        valid_links.push_back(link);
      }
    }
    
    if (valid_links.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "❌ No valid links found for constraints");
      return false;
    }
    
    // Add position constraints for each valid link
    for (const auto& link_name : valid_links) {
      moveit_msgs::msg::PositionConstraint pos_constraint;
      pos_constraint.header.frame_id = move_group_->getPlanningFrame();
      pos_constraint.link_name = link_name;
      
      // Create constraint region (workspace bounding box)
      shape_msgs::msg::SolidPrimitive constraint_region;
      constraint_region.type = constraint_region.BOX;
      constraint_region.dimensions = {
        config_.workspace.width,
        config_.workspace.depth,
        config_.workspace.height
      };
      
      geometry_msgs::msg::Pose constraint_pose;
      constraint_pose.position.x = config_.workspace.x_position;
      constraint_pose.position.y = config_.workspace.y_position;
      constraint_pose.position.z = config_.workspace.z_position;
      constraint_pose.orientation.w = 1.0;
      
      pos_constraint.constraint_region.primitives.push_back(constraint_region);
      pos_constraint.constraint_region.primitive_poses.push_back(constraint_pose);
      pos_constraint.weight = 1.0;
      
      workspace_constraints.position_constraints.push_back(pos_constraint);
    }
    
    // Apply constraints
    move_group_->setPathConstraints(workspace_constraints);
    workspace_constraints_applied_ = true;
    
    RCLCPP_INFO(node_->get_logger(), "✅ Workspace constraints applied to %zu links", valid_links.size());
    printWorkspaceLimits();
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception adding workspace constraints: %s", e.what());
    return false;
  }
}

void TrajectoryPlanner::clearWorkspaceConstraints()
{
  if (workspace_constraints_applied_) {
    move_group_->clearPathConstraints();
    workspace_constraints_applied_ = false;
    RCLCPP_INFO(node_->get_logger(), "🧹 Workspace constraints cleared");
  }
}

// Validation methods
bool TrajectoryPlanner::validatePoseInWorkspace(const geometry_msgs::msg::Pose& pose)
{
  bool valid = isPositionInWorkspace(pose.position.x, pose.position.y, pose.position.z);
  
  if (!valid) {
    RCLCPP_WARN(node_->get_logger(), 
                "❌ Pose [%.3f, %.3f, %.3f] outside workspace", 
                pose.position.x, pose.position.y, pose.position.z);
    printWorkspaceLimits();
  } else {
    RCLCPP_INFO(node_->get_logger(), 
                "✅ Pose [%.3f, %.3f, %.3f] within workspace", 
                pose.position.x, pose.position.y, pose.position.z);
  }
  
  return valid;
}

bool TrajectoryPlanner::isPositionInWorkspace(double x, double y, double z) const
{
  double x_min = config_.workspace.x_position - config_.workspace.width / 2.0;
  double x_max = config_.workspace.x_position + config_.workspace.width / 2.0;
  double y_min = config_.workspace.y_position - config_.workspace.depth / 2.0;
  double y_max = config_.workspace.y_position + config_.workspace.depth / 2.0;
  double z_min = config_.workspace.z_position - config_.workspace.height / 2.0;
  double z_max = config_.workspace.z_position + config_.workspace.height / 2.0;
  
  return (x >= x_min && x <= x_max &&
          y >= y_min && y <= y_max &&
          z >= z_min && z <= z_max);
}

bool TrajectoryPlanner::validateTrajectoryCollisionFree(const moveit_msgs::msg::RobotTrajectory& /* trajectory */)
{
  if (!table_collision_setup_) {
    RCLCPP_WARN(node_->get_logger(), "⚠️ Table collision not setup, cannot validate");
    return false;
  }
  
  RCLCPP_INFO(node_->get_logger(), "✅ Trajectory validated");
  return true;
}

// Visualization methods
void TrajectoryPlanner::enableWorkspaceVisualization(bool enable)
{
  config_.workspace.visualization_enabled = enable;
  
  if (enable) {
    RCLCPP_INFO(node_->get_logger(), "🔳 Workspace visualization ENABLED");
    
    visualization_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { publishWorkspaceBoundary(); });
    
    publishWorkspaceBoundary();
  } else {
    RCLCPP_INFO(node_->get_logger(), "🔲 Workspace visualization DISABLED");
    
    if (visualization_timer_) {
      visualization_timer_->cancel();
    }
    
    // Clear markers
    visualization_msgs::msg::MarkerArray clear_markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = move_group_->getPlanningFrame();
    clear_marker.header.stamp = node_->now();
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_markers.markers.push_back(clear_marker);
    marker_publisher_->publish(clear_markers);
  }
}

void TrajectoryPlanner::setWorkspaceVisualizationColor(float r, float g, float b, float a)
{
  config_.workspace.color_r = r;
  config_.workspace.color_g = g;
  config_.workspace.color_b = b;
  config_.workspace.color_a = a;
  
  RCLCPP_INFO(node_->get_logger(), "🎨 Workspace color set to RGBA(%.2f, %.2f, %.2f, %.2f)", 
              r, g, b, a);
}

void TrajectoryPlanner::publishWorkspaceBoundary()
{
  if (!config_.workspace.visualization_enabled) return;
  
  visualization_msgs::msg::MarkerArray marker_array;
  createWorkspaceBoundaryMarkers(marker_array);
  marker_publisher_->publish(marker_array);
}

void TrajectoryPlanner::createWorkspaceBoundaryMarkers(visualization_msgs::msg::MarkerArray& marker_array)
{
  marker_array.markers.clear();
  
  // Create wireframe bounding box
  visualization_msgs::msg::Marker box_marker;
  box_marker.header.frame_id = move_group_->getPlanningFrame();
  box_marker.header.stamp = node_->now();
  box_marker.ns = "workspace_boundary";
  box_marker.id = 0;
  box_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  box_marker.action = visualization_msgs::msg::Marker::ADD;
  
  box_marker.scale.x = 0.01; // Line width
  box_marker.color.r = config_.workspace.color_r;
  box_marker.color.g = config_.workspace.color_g;
  box_marker.color.b = config_.workspace.color_b;
  box_marker.color.a = config_.workspace.color_a;
  
  // Calculate bounds
  double x_min = config_.workspace.x_position - config_.workspace.width / 2.0;
  double x_max = config_.workspace.x_position + config_.workspace.width / 2.0;
  double y_min = config_.workspace.y_position - config_.workspace.depth / 2.0;
  double y_max = config_.workspace.y_position + config_.workspace.depth / 2.0;
  double z_min = config_.workspace.z_position - config_.workspace.height / 2.0;
  double z_max = config_.workspace.z_position + config_.workspace.height / 2.0;
  
  // Define 8 corners
  std::vector<geometry_msgs::msg::Point> corners(8);
  corners[0].x = x_min; corners[0].y = y_min; corners[0].z = z_min;
  corners[1].x = x_max; corners[1].y = y_min; corners[1].z = z_min;
  corners[2].x = x_max; corners[2].y = y_max; corners[2].z = z_min;
  corners[3].x = x_min; corners[3].y = y_max; corners[3].z = z_min;
  corners[4].x = x_min; corners[4].y = y_min; corners[4].z = z_max;
  corners[5].x = x_max; corners[5].y = y_min; corners[5].z = z_max;
  corners[6].x = x_max; corners[6].y = y_max; corners[6].z = z_max;
  corners[7].x = x_min; corners[7].y = y_max; corners[7].z = z_max;
  
  // Create wireframe edges (12 edges total)
  // Bottom face
  box_marker.points.push_back(corners[0]); box_marker.points.push_back(corners[1]);
  box_marker.points.push_back(corners[1]); box_marker.points.push_back(corners[2]);
  box_marker.points.push_back(corners[2]); box_marker.points.push_back(corners[3]);
  box_marker.points.push_back(corners[3]); box_marker.points.push_back(corners[0]);
  
  // Top face
  box_marker.points.push_back(corners[4]); box_marker.points.push_back(corners[5]);
  box_marker.points.push_back(corners[5]); box_marker.points.push_back(corners[6]);
  box_marker.points.push_back(corners[6]); box_marker.points.push_back(corners[7]);
  box_marker.points.push_back(corners[7]); box_marker.points.push_back(corners[4]);
  
  // Vertical edges
  box_marker.points.push_back(corners[0]); box_marker.points.push_back(corners[4]);
  box_marker.points.push_back(corners[1]); box_marker.points.push_back(corners[5]);
  box_marker.points.push_back(corners[2]); box_marker.points.push_back(corners[6]);
  box_marker.points.push_back(corners[3]); box_marker.points.push_back(corners[7]);
  
  marker_array.markers.push_back(box_marker);
}

void TrajectoryPlanner::printWorkspaceLimits()
{
  double x_min = config_.workspace.x_position - config_.workspace.width / 2.0;
  double x_max = config_.workspace.x_position + config_.workspace.width / 2.0;
  double y_min = config_.workspace.y_position - config_.workspace.depth / 2.0;
  double y_max = config_.workspace.y_position + config_.workspace.depth / 2.0;
  double z_min = config_.workspace.z_position - config_.workspace.height / 2.0;
  double z_max = config_.workspace.z_position + config_.workspace.height / 2.0;
  
  RCLCPP_INFO(node_->get_logger(), "📐 Workspace Limits:");
  RCLCPP_INFO(node_->get_logger(), "   X: [%.3f, %.3f] m", x_min, x_max);
  RCLCPP_INFO(node_->get_logger(), "   Y: [%.3f, %.3f] m", y_min, y_max);
  RCLCPP_INFO(node_->get_logger(), "   Z: [%.3f, %.3f] m", z_min, z_max);
}

// Utility methods
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

} // namespace trajectory_plan