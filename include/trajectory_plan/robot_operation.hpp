#ifndef ROBOT_OPERATION_HPP
#define ROBOT_OPERATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <vector>

namespace trajectory_plan
{

class RobotOperation
{
public:
  RobotOperation(const rclcpp::Node::SharedPtr& node,
                const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group);
  
  // Configuration methods
  void setVelocityScaling(double scaling);
  void setAccelerationScaling(double scaling);
  void setSmoothMotion(bool enable = true);
  void setHomePosition(const std::vector<double>& joints);
  void setSafePosition(const std::vector<double>& joints);
  
  // Planning methods
  bool planToPose(const geometry_msgs::msg::Pose& target_pose);
  bool planToJointPosition(const std::vector<double>& joint_values);
  
  // Execution methods
  bool executePlan();
  
  // High-level operations
  bool moveToHome();
  bool moveToSafePosition();
  
  // Professional pick-and-place with mm/degrees input
  bool executeProfessionalPickAndPlace(
    double pick_x_mm, double pick_y_mm, double pick_z_mm,
    double pick_roll_deg, double pick_pitch_deg, double pick_yaw_deg,
    double place_x_mm, double place_y_mm, double place_z_mm,
    double place_roll_deg, double place_pitch_deg, double place_yaw_deg,
    double clearance_height_mm = 50.0  // 5cm default clearance
  );
  
  // Legacy pick-and-place (for backward compatibility)
  bool planPickAndPlace(
    const geometry_msgs::msg::Pose& pick_pose,
    const geometry_msgs::msg::Pose& place_pose,
    double approach_distance = 0.08,
    double retreat_distance = 0.08
  );
  
  // Gripper control
  void openGripper();
  void closeGripper();
  
  // Utility methods
  void checkAndPrintCurrentPose();
  void waitForMotionComplete(double seconds = 1.0);
  geometry_msgs::msg::Pose getCurrentPose();
  
  // Conversion utilities
  geometry_msgs::msg::Pose createPoseFromMmAndDegrees(
    double x_mm, double y_mm, double z_mm,
    double roll_deg, double pitch_deg, double yaw_deg
  );
  
  // State inquiry
  bool isReady() const { return initialized_; }

private:
  // Core components
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  
  // Configuration
  double velocity_scaling_factor_;
  double acceleration_scaling_factor_;
  std::vector<double> home_joint_values_;
  std::vector<double> safe_joint_values_;
  
  // State
  bool initialized_;
  moveit::planning_interface::MoveGroupInterface::Plan current_plan_;
  
  // Private utility methods
  geometry_msgs::msg::Pose createApproachPose(const geometry_msgs::msg::Pose& target_pose, double distance);
  geometry_msgs::msg::Pose createRetreatPose(const geometry_msgs::msg::Pose& target_pose, double distance);
  bool executePickAndPlaceSequence(
    const geometry_msgs::msg::Pose& pick_pose,
    const geometry_msgs::msg::Pose& place_pose,
    double approach_distance,
    double retreat_distance
  );
};

} // namespace trajectory_plan

#endif // ROBOT_OPERATION_HPP