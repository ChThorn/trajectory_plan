#include "trajectory_plan/trajectory_planner.hpp"

namespace trajectory_plan
{
TrajectoryPlanner::TrajectoryPlanner(const rclcpp::Node::SharedPtr& node)
  : node_(node)
{
  // Initialize MoveIt interfaces
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_, "mainpulation"); // Fixed: Keep the working name from your original code
  
  planning_scene_interface_ = 
    std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  
  // Initialize modular components
  robot_operation_ = std::make_unique<RobotOperation>(node_, move_group_);
  table_config_ = std::make_unique<TableConfig>(node_, move_group_, planning_scene_interface_);
  workspace_config_ = std::make_unique<WorkspaceConfig>(node_, move_group_);
  
  // Set up table collision prevention with your working dimensions
  setTableDimensions(0.98, 0.49, 0.04, 0.0, 0.0);

  // Set up workspace with visualization
//   workspace_config_ = std::make_unique<WorkspaceConfig>(node_, move_group_);
  
  // Configure workspace dimensions (adjust these to your robot's reach)
  workspace_config_->setWorkspaceDimensions(1.4, 1.2, 1.0);  // 140cm x 120cm x 100cm (was 80x80x60)
  workspace_config_->setWorkspacePosition(0.4, 0.0, 0.5);
  
  // Enable visualization with blue transparent boundary
  workspace_config_->enableWorkspaceVisualization(true);
  workspace_config_->setWorkspaceBoundaryColor(0.0, 1.0, 1.0, 0.7); // RED with 100% opacity
  
  // Add collision object after short delay to ensure planning scene is ready
//   auto setup_timer = node_->create_wall_timer(
//     std::chrono::seconds(3),
//     [this]() {
//       setupTableCollisionObject();
//       addWorkspaceConstraints();
//     });

    setup_timer_ = node_->create_wall_timer(
        std::chrono::seconds(3),
        [this]() {
        setupTableCollisionObject();
        addWorkspaceConstraints();
        setup_timer_->cancel(); // Cancel after first execution
        });
  
  RCLCPP_INFO(node_->get_logger(), "🛡️ Table collision prevention will be set up in 3 seconds...");
}

bool TrajectoryPlanner::planPickAndPlace(
  const geometry_msgs::msg::Pose& pick_pose,
  const geometry_msgs::msg::Pose& place_pose,
  double approach_distance,
  double retreat_distance)
{
  // Ensure table collision object is set up
  if (!table_config_->isTableCollisionSetup()) {
    RCLCPP_WARN(node_->get_logger(), "⚠️ Table collision object not set up, setting up now...");
    setupTableCollisionObject();
  }
  
  // Delegate to robot operation module
  return robot_operation_->planPickAndPlace(pick_pose, place_pose, approach_distance, retreat_distance);
}

bool TrajectoryPlanner::planMountedTablePickAndPlace(
  const geometry_msgs::msg::Pose& pick_pose,
  const geometry_msgs::msg::Pose& place_pose,
  double approach_distance,
  double retreat_distance)
{
  // Ensure table collision object is set up
  if (!table_config_->isTableCollisionSetup()) {
    RCLCPP_WARN(node_->get_logger(), "⚠️ Table collision object not set up, setting up now...");
    setupTableCollisionObject();
  }
  
  // Delegate to robot operation module
  return robot_operation_->planMountedTablePickAndPlace(pick_pose, place_pose, approach_distance, retreat_distance);
}

// Delegate methods to RobotOperation
bool TrajectoryPlanner::executePlan()
{
  return robot_operation_->executePlan();
}

bool TrajectoryPlanner::moveToHome()
{
  return robot_operation_->moveToHome();
}

bool TrajectoryPlanner::moveToSafePosition()
{
  return robot_operation_->moveToSafePosition();
}

void TrajectoryPlanner::setVelocityScaling(double scaling)
{
  robot_operation_->setVelocityScaling(scaling);
}

void TrajectoryPlanner::setAccelerationScaling(double scaling)
{
  robot_operation_->setAccelerationScaling(scaling);
}

void TrajectoryPlanner::setSmoothMotion(bool enable)
{
  robot_operation_->setSmoothMotion(enable);
}

void TrajectoryPlanner::printCurrentPose()
{
  robot_operation_->printCurrentPose();
}

void TrajectoryPlanner::waitForMotionComplete(double seconds)
{
  robot_operation_->waitForMotionComplete(seconds);
}

// Delegate methods to TableConfig
bool TrajectoryPlanner::setupTableCollisionObject()
{
  table_collision_setup_ = table_config_->setupTableCollisionObject();
  return table_collision_setup_;
}

void TrajectoryPlanner::setTableDimensions(double length, double width, double height,
                                          double x_offset, double y_offset)
{
  // Update local dimensions for compatibility
  table_dims_.length = length;
  table_dims_.width = width;
  table_dims_.height = height;
  table_dims_.x_offset = x_offset;
  table_dims_.y_offset = y_offset;
  table_dims_.z_position = -0.027; // Keep your working value
  
  // Delegate to table config
  table_config_->setTableDimensions(length, width, height, x_offset, y_offset);
}

// Delegate methods to WorkspaceConfig
bool TrajectoryPlanner::addWorkspaceConstraints()
{
  return workspace_config_->applyWorkspaceConstraints();
}

// Validation method (delegated to TableConfig)
bool TrajectoryPlanner::validateTrajectoryAgainstTable(const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  return table_config_->validateTrajectoryAgainstTable(trajectory);
}

void TrajectoryPlanner::enableWorkspaceVisualization(bool enable)
{
  workspace_config_->enableWorkspaceVisualization(enable);
}

void TrajectoryPlanner::setWorkspaceVisualizationColor(float r, float g, float b, float a)
{
  workspace_config_->setWorkspaceBoundaryColor(r, g, b, a);
}

bool TrajectoryPlanner::validatePoseInWorkspace(const geometry_msgs::msg::Pose& pose)
{
  return workspace_config_->validatePoseInWorkspace(pose);
}

void TrajectoryPlanner::printWorkspaceLimits()
{
  workspace_config_->printWorkspaceLimits();
}

} // namespace trajectory_plan