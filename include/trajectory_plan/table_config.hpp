#ifndef TABLE_CONFIG_HPP
#define TABLE_CONFIG_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace trajectory_plan
{
class TableConfig
{
public:
  TableConfig(const rclcpp::Node::SharedPtr& node,
             const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group,
             const std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>& planning_scene);
  
  // Set table dimensions and position
  void setTableDimensions(double length, double width, double height, 
                         double x_offset = 0.0, double y_offset = 0.0);
  
  // Setup table collision object in the planning scene
  bool setupTableCollisionObject();
  
  // Validate a trajectory against table collision
  bool validateTrajectoryAgainstTable(const moveit_msgs::msg::RobotTrajectory& trajectory);
  
  // Remove table collision object
  void removeTableCollisionObject();
  
  // Check if table collision setup is complete
  bool isTableCollisionSetup() const { return table_collision_setup_; }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  
  // Table dimensions and position
  struct TableDimensions {
    double length = 1.2;    // Table length (m)
    double width = 0.8;     // Table width (m) 
    double height = 0.02;   // Table thickness (m)
    double x_offset = 0.0;  // Table center X offset from robot base
    double y_offset = 0.0;  // Table center Y offset from robot base
    double z_position = 0.01; // Table surface Z position (below robot base)
  } table_dims_;
  
  // Flag to track if table collision object is set up
  bool table_collision_setup_ = false;
  
  // Table collision object ID
  const std::string table_id_ = "table_surface";
};
}

#endif // TABLE_CONFIG_HPP
