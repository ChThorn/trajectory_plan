// #ifndef TRAJECTORY_PLANNER_HPP
// #define TRAJECTORY_PLANNER_HPP

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/msg/display_robot_state.hpp>
// #include <moveit_msgs/msg/display_trajectory.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <tf2_eigen/tf2_eigen.hpp>

// namespace trajectory_plan
// {
// class TrajectoryPlanner
// {
// public:
//   TrajectoryPlanner(const rclcpp::Node::SharedPtr& node);
  
//   bool planPickAndPlace(
//     const geometry_msgs::msg::Pose& pick_pose,
//     const geometry_msgs::msg::Pose& place_pose,
//     double approach_distance = 0.1,
//     double retreat_distance = 0.1
//   );
  
//   bool executePlan();
  
//   bool moveToHome();
//   bool moveToSafePosition();
  
//   void setVelocityScaling(double scaling);
//   void setAccelerationScaling(double scaling);
  
//   // Additional motion control functions
//   void setSmoothMotion(bool enable = true);
//   void printCurrentPose();
//   void waitForMotionComplete(double seconds = 1.0);

// private:
//   rclcpp::Node::SharedPtr node_;
//   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
//   std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  
//   // Current plan storage
//   moveit::planning_interface::MoveGroupInterface::Plan current_plan_;
  
//   // Pre-defined poses
//   std::vector<double> home_joint_values_;
//   std::vector<double> safe_joint_values_;
  
//   // Planning parameters
//   double velocity_scaling_factor_;
//   double acceleration_scaling_factor_;
  
//   bool planToJointPosition(const std::vector<double>& joint_values);
//   bool planToPose(const geometry_msgs::msg::Pose& target_pose);
//   geometry_msgs::msg::Pose createApproachPose(const geometry_msgs::msg::Pose& target_pose, double distance);
//   geometry_msgs::msg::Pose createRetreatPose(const geometry_msgs::msg::Pose& target_pose, double distance);
// };
// }

// #endif





#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


// Add these includes at the top of trajectory_planner.hpp (after existing includes)
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace trajectory_plan
{
class TrajectoryPlanner
{
public:
  TrajectoryPlanner(const rclcpp::Node::SharedPtr& node);
  
  bool planPickAndPlace(
    const geometry_msgs::msg::Pose& pick_pose,
    const geometry_msgs::msg::Pose& place_pose,
    double approach_distance = 0.1,
    double retreat_distance = 0.1
  );
  
  bool executePlan();
  
  bool moveToHome();
  bool moveToSafePosition();
  
  void setVelocityScaling(double scaling);
  void setAccelerationScaling(double scaling);
  
  // Additional motion control functions
  void setSmoothMotion(bool enable = true);
  void printCurrentPose();
  void waitForMotionComplete(double seconds = 1.0);

  // NEW METHODS for collision prevention
  bool setupTableCollisionObject();
  bool addWorkspaceConstraints();
  void setTableDimensions(double length, double width, double height, 
                         double x_offset = 0.0, double y_offset = 0.0);
  bool validateTrajectoryAgainstTable(const moveit_msgs::msg::RobotTrajectory& trajectory);

  bool planMountedTablePickAndPlace(
    const geometry_msgs::msg::Pose& pick_pose,
    const geometry_msgs::msg::Pose& place_pose,
    double approach_distance = 0.08,
    double retreat_distance = 0.08
  );

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  
  // Current plan storage
  moveit::planning_interface::MoveGroupInterface::Plan current_plan_;
  
  // Pre-defined poses
  std::vector<double> home_joint_values_;
  std::vector<double> safe_joint_values_;
  
  // Planning parameters
  double velocity_scaling_factor_;
  double acceleration_scaling_factor_;
  
  bool planToJointPosition(const std::vector<double>& joint_values);
  bool planToPose(const geometry_msgs::msg::Pose& target_pose);
  geometry_msgs::msg::Pose createApproachPose(const geometry_msgs::msg::Pose& target_pose, double distance);
  geometry_msgs::msg::Pose createRetreatPose(const geometry_msgs::msg::Pose& target_pose, double distance);


  // NEW MEMBERS for table collision prevention
  struct TableDimensions {
    double length = 1.2;    // Table length (m)
    double width = 0.8;     // Table width (m) 
    double height = 0.02;   // Table thickness (m)
    double x_offset = 0.0;  // Table center X offset from robot base
    double y_offset = 0.0;  // Table center Y offset from robot base
    double z_position = 0.01; // Table surface Z position (below robot base)
  } table_dims_;
  
  bool table_collision_setup_ = false;
};
}

#endif
