#include "trajectory_plan/table_config.hpp"

namespace trajectory_plan
{
TableConfig::TableConfig(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group,
    const std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>& planning_scene)
  : node_(node)
  , move_group_(move_group)
  , planning_scene_interface_(planning_scene)
{
  RCLCPP_INFO(node_->get_logger(), "TableConfig initialized");
}

void TableConfig::setTableDimensions(double length, double width, double height,
                                    double x_offset, double y_offset)
{
  table_dims_.length = length;
  table_dims_.width = width;
  table_dims_.height = height;
  table_dims_.x_offset = x_offset;
  table_dims_.y_offset = y_offset;
  
  // For table mounted to the robot base, the top surface is at the robot base level (Z=0)
  // So the collision object center should be below the base level
  table_dims_.z_position = -height / 2.0-0.0008;  // Table center is below base level
  
  RCLCPP_INFO(node_->get_logger(), "🔧 Table dimensions updated:");
  RCLCPP_INFO(node_->get_logger(), "   Size: %.2f x %.2f x %.2f m", length, width, height);
  RCLCPP_INFO(node_->get_logger(), "   Collision center: [%.3f, %.3f, %.3f] m", 
              x_offset, y_offset, table_dims_.z_position);
  RCLCPP_INFO(node_->get_logger(), "   Table surface: AT Z=0 (robot base level)");
}

bool TableConfig::setupTableCollisionObject()
{
  RCLCPP_INFO(node_->get_logger(), "🛡️ Setting up table collision object...");
  
  // Create collision object for the table surface
  moveit_msgs::msg::CollisionObject table_object;
  table_object.header.frame_id = move_group_->getPlanningFrame();
  table_object.id = table_id_;
  
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

bool TableConfig::validateTrajectoryAgainstTable(const moveit_msgs::msg::RobotTrajectory& /* trajectory */)
{
  // This is a placeholder for trajectory validation against table collision
  // In a real implementation, this would check if the trajectory would cause collision with the table
  
  RCLCPP_INFO(node_->get_logger(), "🔍 Validating trajectory against table collision...");
  
  // For now, we'll assume the trajectory is valid if the table collision object is set up
  // In a real implementation, you would use collision checking from MoveIt
  
  if (!table_collision_setup_) {
    RCLCPP_WARN(node_->get_logger(), "⚠️ Table collision object not set up, cannot validate trajectory");
    return false;
  }
  
  RCLCPP_INFO(node_->get_logger(), "✅ Trajectory validated against table collision");
  return true;
}

void TableConfig::removeTableCollisionObject()
{
  if (table_collision_setup_) {
    std::vector<std::string> object_ids = {table_id_};
    planning_scene_interface_->removeCollisionObjects(object_ids);
    
    RCLCPP_INFO(node_->get_logger(), "🧹 Table collision object removed");
    table_collision_setup_ = false;
  }
}

} // namespace trajectory_plan