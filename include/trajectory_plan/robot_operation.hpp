#ifndef ROBOT_OPERATION_HPP
#define ROBOT_OPERATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <vector>

namespace trajectory_plan
{

/**
 * @class RobotOperation
 * @brief Manages low-level robot motion primitives like planning, execution, and gripper control.
 *
 * This class abstracts the direct interaction with the MoveGroupInterface for common
 * robot tasks, providing a cleaner API for higher-level sequences like pick-and-place.
 */
class RobotOperation
{
public:
  /**
   * @brief Construct a new Robot Operation object.
   * @param node A shared pointer to the ROS 2 node.
   * @param move_group A shared pointer to the MoveGroupInterface instance.
   */
  RobotOperation(const rclcpp::Node::SharedPtr& node,
                const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group);
  
  // --- Configuration ---
  void setVelocityScaling(double scaling);
  void setAccelerationScaling(double scaling);
  void setSmoothMotion(bool enable = true);
  void setHomePosition(const std::vector<double>& joints);
  void setSafePosition(const std::vector<double>& joints);
  
  // --- Core Motion Primitives ---
  bool planToPose(const geometry_msgs::msg::Pose& target_pose);
  bool planToJointPosition(const std::vector<double>& joint_values);
  bool executePlan();
  
  // --- High-Level Sequences ---
  bool moveToHome();
  bool moveToSafePosition();
  
  /**
   * @brief Executes a robust, production-ready pick-and-place sequence.
   *
   * This sequence includes moving to approach/retreat poses using linear Cartesian motions
   * to ensure safe and precise operations.
   *
   * @param pick_x_mm X position for picking in millimeters.
   * @param pick_y_mm Y position for picking in millimeters.
   * @param pick_z_mm Z position for picking in millimeters.
   * @param pick_roll_deg Roll orientation for picking in degrees.
   * @param pick_pitch_deg Pitch orientation for picking in degrees.
   * @param pick_yaw_deg Yaw orientation for picking in degrees.
   * @param place_x_mm X position for placing in millimeters.
   * @param place_y_mm Y position for placing in millimeters.
   * @param place_z_mm Z position for placing in millimeters.
   * @param place_roll_deg Roll orientation for placing in degrees.
   * @param place_pitch_deg Pitch orientation for placing in degrees.
   * @param place_yaw_deg Yaw orientation for placing in degrees.
   * @param clearance_height_mm The height in mm for approach and retreat motions.
   * @return true if the entire sequence is successful, false otherwise.
   */
  bool executeProfessionalPickAndPlace(
    double pick_x_mm, double pick_y_mm, double pick_z_mm,
    double pick_roll_deg, double pick_pitch_deg, double pick_yaw_deg,
    double place_x_mm, double place_y_mm, double place_z_mm,
    double place_roll_deg, double place_pitch_deg, double place_yaw_deg,
    double clearance_height_mm = 50.0
  );
  
  // --- Gripper Control (Simulated) ---
  void openGripper();
  void closeGripper();
  
  // --- Utilities ---
  void checkAndPrintCurrentPose();
  void waitForMotionComplete(double seconds = 1.0);
  geometry_msgs::msg::Pose createPoseFromMmAndDegrees(
    double x_mm, double y_mm, double z_mm,
    double roll_deg, double pitch_deg, double yaw_deg
  );
  
private:
  // --- Core Components ---
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  
  // --- Configuration ---
  double velocity_scaling_factor_;
  double acceleration_scaling_factor_;
  std::vector<double> home_joint_values_;
  std::vector<double> safe_joint_values_;
  
  // --- State ---
  bool initialized_;
  moveit::planning_interface::MoveGroupInterface::Plan current_plan_;
  
  // --- Private Methods ---
  bool planAndExecuteCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints);
};

} // namespace trajectory_plan

#endif // ROBOT_OPERATION_HPP
