#ifndef ROBOT_OPERATION_HPP
#define ROBOT_OPERATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace trajectory_plan
{
class RobotOperation
{
public:
  RobotOperation(const rclcpp::Node::SharedPtr& node,
                const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group);
  
  // Basic motion planning and execution
  bool planToPose(const geometry_msgs::msg::Pose& target_pose);
  bool planToJointPosition(const std::vector<double>& joint_values);
  bool executePlan();
  
  // Pre-defined positions
  bool moveToHome();
  bool moveToSafePosition();
  
  // Motion control parameters
  void setVelocityScaling(double scaling);
  void setAccelerationScaling(double scaling);
  void setSmoothMotion(bool enable = true);
  
  // Utility functions
  void printCurrentPose();
  void waitForMotionComplete(double seconds = 1.0);
  
  // Pick and place operations
  bool planPickAndPlace(
    const geometry_msgs::msg::Pose& pick_pose,
    const geometry_msgs::msg::Pose& place_pose,
    double approach_distance = 0.1,
    double retreat_distance = 0.1
  );
  
  bool planMountedTablePickAndPlace(
    const geometry_msgs::msg::Pose& pick_pose,
    const geometry_msgs::msg::Pose& place_pose,
    double approach_distance = 0.08,
    double retreat_distance = 0.08
  );
  
  // Helper functions for pick and place (PUBLIC for delegation)
  geometry_msgs::msg::Pose createApproachPose(const geometry_msgs::msg::Pose& target_pose, double distance);
  geometry_msgs::msg::Pose createRetreatPose(const geometry_msgs::msg::Pose& target_pose, double distance);
  
  // Get current plan
  const moveit::planning_interface::MoveGroupInterface::Plan& getCurrentPlan() const {
    return current_plan_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  
  // Current plan storage
  moveit::planning_interface::MoveGroupInterface::Plan current_plan_;
  
  // Pre-defined poses
  std::vector<double> home_joint_values_;
  std::vector<double> safe_joint_values_;
  
  // Planning parameters
  double velocity_scaling_factor_;
  double acceleration_scaling_factor_;
};
}

#endif // ROBOT_OPERATION_HPP