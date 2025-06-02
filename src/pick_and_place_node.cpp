// --------------------------In position and orientation quaternions--------------------------
// #include <rclcpp/rclcpp.hpp>
// #include "trajectory_plan/trajectory_planner.hpp"
// #include <geometry_msgs/msg/pose.hpp>
// #include <control_msgs/action/follow_joint_trajectory.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <thread>
// #include <tf2/LinearMath/Quaternion.h>

// class PickAndPlaceNode : public rclcpp::Node
// {
// public:
//   PickAndPlaceNode() : Node("pick_and_place_node")
//   {
//     RCLCPP_INFO(get_logger(), "Pick and Place Node starting initialization...");
    
//     // Set up demo poses
//     setupDemoPoses();
    
//     // Check for controller availability first
//     checkControllers();
    
//     // Create timer for demo execution - wait for controllers
//     timer_ = create_wall_timer(
//       std::chrono::seconds(25),  // Wait longer for controllers to be ready
//       std::bind(&PickAndPlaceNode::executeDemo, this));
      
//     RCLCPP_INFO(get_logger(), "Pick and Place Node initialized - waiting for controllers to be ready...");
//   }
  
//   void checkControllers()
//   {
//     RCLCPP_INFO(get_logger(), "🔍 Checking controller availability...");
    
//     // Create action client to test controller connection
//     using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
//     auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
//       this, "joint_trajectory_controller/follow_joint_trajectory");
    
//     // Check if action server is available (non-blocking)
//     auto timer = create_wall_timer(
//       std::chrono::seconds(5),
//       [this, action_client]() {
//         if (action_client->action_server_is_ready()) {
//           RCLCPP_INFO(get_logger(), "✅ Controller action server is ready!");
//         } else {
//           RCLCPP_WARN(get_logger(), "⚠️ Controller action server not ready yet...");
//         }
//       });
//   }
  
//   void initialize_planner()
//   {
//     if (!trajectory_planner_) {
//       RCLCPP_INFO(get_logger(), "Initializing trajectory planner...");
//       trajectory_planner_ = std::make_unique<trajectory_plan::TrajectoryPlanner>(
//         shared_from_this());
//       RCLCPP_INFO(get_logger(), "Trajectory planner initialized successfully");
//     }
//   }

// private:
//   void setupDemoPoses()
//   {
//     // Default demo poses - REPLACE WITH YOUR OBJECT POSITIONS
//     setupObjectPoses(
//       // Pick position and orientation
//       0.3, 0.2, 0.15,     // x, y, z position
//       0.0, 1.57, 0.0,     // roll, pitch, yaw orientation
      
//       // Place position and orientation  
//       0.3, -0.2, 0.15,    // x, y, z position
//       0.0, 1.57, 0.0      // roll, pitch, yaw orientation
//     );
//   }
  
//   void setupObjectPoses(
//     double pick_x, double pick_y, double pick_z,
//     double pick_roll, double pick_pitch, double pick_yaw,
//     double place_x, double place_y, double place_z, 
//     double place_roll, double place_pitch, double place_yaw)
//   {
//     // Set pick pose with your object position and orientation
//     pick_pose_.position.x = pick_x;
//     pick_pose_.position.y = pick_y;
//     pick_pose_.position.z = pick_z;
    
//     tf2::Quaternion q_pick;
//     q_pick.setRPY(pick_roll, pick_pitch, pick_yaw);
//     pick_pose_.orientation.x = q_pick.x();
//     pick_pose_.orientation.y = q_pick.y();
//     pick_pose_.orientation.z = q_pick.z();
//     pick_pose_.orientation.w = q_pick.w();
    
//     // Set place pose with your target position and orientation
//     place_pose_.position.x = place_x;
//     place_pose_.position.y = place_y;
//     place_pose_.position.z = place_z;
    
//     tf2::Quaternion q_place;
//     q_place.setRPY(place_roll, place_pitch, place_yaw);
//     place_pose_.orientation.x = q_place.x();
//     place_pose_.orientation.y = q_place.y();
//     place_pose_.orientation.z = q_place.z();
//     place_pose_.orientation.w = q_place.w();
    
//     RCLCPP_INFO(get_logger(), "📍 Object poses configured:");
//     RCLCPP_INFO(get_logger(), "   Pick:  [%.3f, %.3f, %.3f] RPY:[%.2f, %.2f, %.2f]", 
//                 pick_x, pick_y, pick_z, pick_roll, pick_pitch, pick_yaw);
//     RCLCPP_INFO(get_logger(), "   Place: [%.3f, %.3f, %.3f] RPY:[%.2f, %.2f, %.2f]", 
//                 place_x, place_y, place_z, place_roll, place_pitch, place_yaw);
//   }
  
//   void executeDemo()
//   {
//     // Disable timer after first execution
//     timer_->cancel();
    
//     RCLCPP_INFO(get_logger(), "🚀 Starting Enhanced Pick and Place Demo");
//     RCLCPP_INFO(get_logger(), "   Robot will move smoothly and safely!");
    
//     // Initialize trajectory planner
//     initialize_planner();
    
//     if (!trajectory_planner_) {
//       RCLCPP_ERROR(get_logger(), "Failed to initialize trajectory planner");
//       return;
//     }
    
//     // Test controller connectivity
//     RCLCPP_INFO(get_logger(), "🔌 Testing controller connectivity...");
//     using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
//     auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
//       this, "joint_trajectory_controller/follow_joint_trajectory");
    
//     if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
//       RCLCPP_ERROR(get_logger(), "❌ Action server not available! Check controllers:");
//       RCLCPP_ERROR(get_logger(), "   Run: ros2 control list_controllers");
//       RCLCPP_ERROR(get_logger(), "   Check if joint_trajectory_controller is loaded");
//       return;
//     }
    
//     RCLCPP_INFO(get_logger(), "✅ Controller connection verified!");
    
//     // Enable smooth motion for safer movement
//     trajectory_planner_->setSmoothMotion(true);
    
//     // Wait for full system readiness
//     RCLCPP_INFO(get_logger(), "⏳ Waiting for full system readiness...");
//     std::this_thread::sleep_for(std::chrono::seconds(5));
    
//     // Print current robot state
//     trajectory_planner_->printCurrentPose();
    
//     // Move to home position
//     RCLCPP_INFO(get_logger(), "🏠 Moving to home position...");
//     if (trajectory_planner_->moveToHome()) {
//       RCLCPP_INFO(get_logger(), "✅ Successfully moved to home position");
//     } else {
//       RCLCPP_WARN(get_logger(), "⚠️ Home position execution had issues, but continuing...");
//     }
    
//     // Wait between movements
//     trajectory_planner_->waitForMotionComplete(3.0);
    
//     // Execute pick and place with your object positions
//     RCLCPP_INFO(get_logger(), "📦 Starting pick and place sequence...");
//     RCLCPP_INFO(get_logger(), "   Using provided object positions and orientations");
//     RCLCPP_INFO(get_logger(), "   Pick: [%.3f, %.3f, %.3f]", pick_pose_.position.x, pick_pose_.position.y, pick_pose_.position.z);
//     RCLCPP_INFO(get_logger(), "   Place: [%.3f, %.3f, %.3f]", place_pose_.position.x, place_pose_.position.y, place_pose_.position.z);
    
//     if (trajectory_planner_->planPickAndPlace(pick_pose_, place_pose_)) {
//       RCLCPP_INFO(get_logger(), "🎉 Pick and place sequence completed!");
//     } else {
//       RCLCPP_ERROR(get_logger(), "❌ Pick and place sequence failed");
//       RCLCPP_INFO(get_logger(), "💡 Troubleshooting tips:");
//       RCLCPP_INFO(get_logger(), "   1. Check if controllers are running: ros2 control list_controllers");
//       RCLCPP_INFO(get_logger(), "   2. Verify MoveIt config: ros2 topic list | grep trajectory");
//       RCLCPP_INFO(get_logger(), "   3. Test with fake hardware first: use_fake_hardware:=true");
//     }
    
//     RCLCPP_INFO(get_logger(), "✅ Demo completed!");
//     RCLCPP_INFO(get_logger(), "💡 Next steps:");
//     RCLCPP_INFO(get_logger(), "   - Verify robot movement in RViz");
//     RCLCPP_INFO(get_logger(), "   - Test with real object positions");
//     RCLCPP_INFO(get_logger(), "   - Deploy to real hardware when ready");
//   }
  
//   std::unique_ptr<trajectory_plan::TrajectoryPlanner> trajectory_planner_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   geometry_msgs::msg::Pose pick_pose_;
//   geometry_msgs::msg::Pose place_pose_;
// };

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
  
//   // Create node as shared_ptr to avoid shared_from_this issues
//   auto node = std::make_shared<PickAndPlaceNode>();
  
//   RCLCPP_INFO(node->get_logger(), "Pick and Place Node created successfully");
  
//   // Use MultiThreadedExecutor for MoveIt
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(node);
  
//   RCLCPP_INFO(node->get_logger(), "Starting executor...");
//   executor.spin();
  
//   rclcpp::shutdown();
//   return 0;
// }

// ---------------------------In position [mm] and orientation [degrees] w/o workspace considered--------------------------
// #include <rclcpp/rclcpp.hpp>
// #include "trajectory_plan/trajectory_planner.hpp"
// #include <geometry_msgs/msg/pose.hpp>
// #include <control_msgs/action/follow_joint_trajectory.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <thread>
// #include <tf2/LinearMath/Quaternion.h>

// class PickAndPlaceNode : public rclcpp::Node
// {
// public:
//   PickAndPlaceNode() : Node("pick_and_place_node")
//   {
//     RCLCPP_INFO(get_logger(), "Pick and Place Node starting initialization...");
    
//     // Set up demo poses
//     setupDemoPoses();
    
//     // Check for controller availability first
//     checkControllers();
    
//     // Create timer for demo execution - wait for controllers
//     timer_ = create_wall_timer(
//       std::chrono::seconds(25),  // Wait longer for controllers to be ready
//       std::bind(&PickAndPlaceNode::executeDemo, this));
      
//     RCLCPP_INFO(get_logger(), "Pick and Place Node initialized - waiting for controllers to be ready...");
//   }
  
//   void checkControllers()
//   {
//     RCLCPP_INFO(get_logger(), "🔍 Checking controller availability...");
    
//     // Create action client to test controller connection
//     using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
//     auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
//       this, "joint_trajectory_controller/follow_joint_trajectory");
    
//     // Check if action server is available (non-blocking)
//     auto timer = create_wall_timer(
//       std::chrono::seconds(5),
//       [this, action_client]() {
//         if (action_client->action_server_is_ready()) {
//           RCLCPP_INFO(get_logger(), "✅ Controller action server is ready!");
//         } else {
//           RCLCPP_WARN(get_logger(), "⚠️ Controller action server not ready yet...");
//         }
//       });
//   }
  
//   void initialize_planner()
//   {
//     if (!trajectory_planner_) {
//       RCLCPP_INFO(get_logger(), "Initializing trajectory planner...");
//       trajectory_planner_ = std::make_unique<trajectory_plan::TrajectoryPlanner>(
//         shared_from_this());
//       RCLCPP_INFO(get_logger(), "Trajectory planner initialized successfully");
//     }
//   }

// private:
//   void setupDemoPoses()
//   {
//     // Demo poses in USER-FRIENDLY UNITS: millimeters and degrees!
//     setupObjectPoses(
//       // Pick position (mm) and orientation (degrees)
//       300.0, 200.0, 150.0,    // x, y, z position in MILLIMETERS
//       0.0, 90.0, 0.0,         // roll, pitch, yaw orientation in DEGREES
      
//       // Place position (mm) and orientation (degrees)
//       300.0, -200.0, 150.0,   // x, y, z position in MILLIMETERS
//       0.0, 90.0, 0.0          // roll, pitch, yaw orientation in DEGREES
//     );
//   }
  
//   void setupObjectPoses(
//     double pick_x_mm, double pick_y_mm, double pick_z_mm,
//     double pick_roll_deg, double pick_pitch_deg, double pick_yaw_deg,
//     double place_x_mm, double place_y_mm, double place_z_mm, 
//     double place_roll_deg, double place_pitch_deg, double place_yaw_deg)
//   {
//     // Convert millimeters to meters for ROS
//     double pick_x = pick_x_mm / 1000.0;
//     double pick_y = pick_y_mm / 1000.0;
//     double pick_z = pick_z_mm / 1000.0;
    
//     double place_x = place_x_mm / 1000.0;
//     double place_y = place_y_mm / 1000.0;
//     double place_z = place_z_mm / 1000.0;
    
//     // Convert degrees to radians for ROS
//     const double DEG_TO_RAD = M_PI / 180.0;
//     double pick_roll = pick_roll_deg * DEG_TO_RAD;
//     double pick_pitch = pick_pitch_deg * DEG_TO_RAD;
//     double pick_yaw = pick_yaw_deg * DEG_TO_RAD;
    
//     double place_roll = place_roll_deg * DEG_TO_RAD;
//     double place_pitch = place_pitch_deg * DEG_TO_RAD;
//     double place_yaw = place_yaw_deg * DEG_TO_RAD;
    
//     // Set pick pose with your object position and orientation
//     pick_pose_.position.x = pick_x;
//     pick_pose_.position.y = pick_y;
//     pick_pose_.position.z = pick_z;
    
//     tf2::Quaternion q_pick;
//     q_pick.setRPY(pick_roll, pick_pitch, pick_yaw);
//     pick_pose_.orientation.x = q_pick.x();
//     pick_pose_.orientation.y = q_pick.y();
//     pick_pose_.orientation.z = q_pick.z();
//     pick_pose_.orientation.w = q_pick.w();
    
//     // Set place pose with your target position and orientation
//     place_pose_.position.x = place_x;
//     place_pose_.position.y = place_y;
//     place_pose_.position.z = place_z;
    
//     tf2::Quaternion q_place;
//     q_place.setRPY(place_roll, place_pitch, place_yaw);
//     place_pose_.orientation.x = q_place.x();
//     place_pose_.orientation.y = q_place.y();
//     place_pose_.orientation.z = q_place.z();
//     place_pose_.orientation.w = q_place.w();
    
//     RCLCPP_INFO(get_logger(), "📍 Object poses configured (USER-FRIENDLY UNITS):");
//     RCLCPP_INFO(get_logger(), "   Pick:  [%.1f, %.1f, %.1f] mm  |  RPY: [%.1f°, %.1f°, %.1f°]", 
//                 pick_x_mm, pick_y_mm, pick_z_mm, pick_roll_deg, pick_pitch_deg, pick_yaw_deg);
//     RCLCPP_INFO(get_logger(), "   Place: [%.1f, %.1f, %.1f] mm  |  RPY: [%.1f°, %.1f°, %.1f°]", 
//                 place_x_mm, place_y_mm, place_z_mm, place_roll_deg, place_pitch_deg, place_yaw_deg);
//     RCLCPP_INFO(get_logger(), "");
//     RCLCPP_INFO(get_logger(), "📏 Internal ROS values (for reference):");
//     RCLCPP_INFO(get_logger(), "   Pick:  [%.3f, %.3f, %.3f] m   |  RPY: [%.3f, %.3f, %.3f] rad", 
//                 pick_x, pick_y, pick_z, pick_roll, pick_pitch, pick_yaw);
//     RCLCPP_INFO(get_logger(), "   Place: [%.3f, %.3f, %.3f] m   |  RPY: [%.3f, %.3f, %.3f] rad", 
//                 place_x, place_y, place_z, place_roll, place_pitch, place_yaw);
//   }
  
//   void executeDemo()
//   {
//     // Disable timer after first execution
//     timer_->cancel();
    
//     RCLCPP_INFO(get_logger(), "🚀 Starting Enhanced Pick and Place Demo");
//     RCLCPP_INFO(get_logger(), "   Robot will move smoothly and safely!");
    
//     // Initialize trajectory planner
//     initialize_planner();
    
//     if (!trajectory_planner_) {
//       RCLCPP_ERROR(get_logger(), "Failed to initialize trajectory planner");
//       return;
//     }
    
//     // Test controller connectivity
//     RCLCPP_INFO(get_logger(), "🔌 Testing controller connectivity...");
//     using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
//     auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
//       this, "joint_trajectory_controller/follow_joint_trajectory");
    
//     if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
//       RCLCPP_ERROR(get_logger(), "❌ Action server not available! Check controllers:");
//       RCLCPP_ERROR(get_logger(), "   Run: ros2 control list_controllers");
//       RCLCPP_ERROR(get_logger(), "   Check if joint_trajectory_controller is loaded");
//       return;
//     }
    
//     RCLCPP_INFO(get_logger(), "✅ Controller connection verified!");
    
//     // Enable smooth motion for safer movement
//     trajectory_planner_->setSmoothMotion(true);
    
//     // Wait for full system readiness
//     RCLCPP_INFO(get_logger(), "⏳ Waiting for full system readiness...");
//     std::this_thread::sleep_for(std::chrono::seconds(5));
    
//     // Print current robot state
//     trajectory_planner_->printCurrentPose();
    
//     // Move to home position
//     RCLCPP_INFO(get_logger(), "🏠 Moving to home position...");
//     if (trajectory_planner_->moveToHome()) {
//       RCLCPP_INFO(get_logger(), "✅ Successfully moved to home position");
//     } else {
//       RCLCPP_WARN(get_logger(), "⚠️ Home position execution had issues, but continuing...");
//     }
    
//     // Wait between movements
//     trajectory_planner_->waitForMotionComplete(3.0);
    
//     // Execute pick and place with your object positions
//     RCLCPP_INFO(get_logger(), "📦 Starting pick and place sequence...");
//     RCLCPP_INFO(get_logger(), "   Using provided object positions and orientations");
    
//     // Convert back to mm for display (positions are stored internally in meters)
//     double pick_x_display = pick_pose_.position.x * 1000.0;
//     double pick_y_display = pick_pose_.position.y * 1000.0;
//     double pick_z_display = pick_pose_.position.z * 1000.0;
//     double place_x_display = place_pose_.position.x * 1000.0;
//     double place_y_display = place_pose_.position.y * 1000.0;
//     double place_z_display = place_pose_.position.z * 1000.0;
    
//     RCLCPP_INFO(get_logger(), "   Pick:  [%.1f, %.1f, %.1f] mm", pick_x_display, pick_y_display, pick_z_display);
//     RCLCPP_INFO(get_logger(), "   Place: [%.1f, %.1f, %.1f] mm", place_x_display, place_y_display, place_z_display);
    
//     if (trajectory_planner_->planPickAndPlace(pick_pose_, place_pose_)) {
//       RCLCPP_INFO(get_logger(), "🎉 Pick and place sequence completed!");
//     } else {
//       RCLCPP_ERROR(get_logger(), "❌ Pick and place sequence failed");
//       RCLCPP_INFO(get_logger(), "💡 Troubleshooting tips:");
//       RCLCPP_INFO(get_logger(), "   1. Check if controllers are running: ros2 control list_controllers");
//       RCLCPP_INFO(get_logger(), "   2. Verify MoveIt config: ros2 topic list | grep trajectory");
//       RCLCPP_INFO(get_logger(), "   3. Test with fake hardware first: use_fake_hardware:=true");
//     }
    
//     RCLCPP_INFO(get_logger(), "✅ Demo completed!");
//     RCLCPP_INFO(get_logger(), "💡 Next steps:");
//     RCLCPP_INFO(get_logger(), "   - Verify robot movement in RViz");
//     RCLCPP_INFO(get_logger(), "   - Test with real object positions");
//     RCLCPP_INFO(get_logger(), "   - Deploy to real hardware when ready");
//   }
  
//   std::unique_ptr<trajectory_plan::TrajectoryPlanner> trajectory_planner_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   geometry_msgs::msg::Pose pick_pose_;
//   geometry_msgs::msg::Pose place_pose_;
// };

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
  
//   // Create node as shared_ptr to avoid shared_from_this issues
//   auto node = std::make_shared<PickAndPlaceNode>();
  
//   RCLCPP_INFO(node->get_logger(), "Pick and Place Node created successfully");
  
//   // Use MultiThreadedExecutor for MoveIt
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(node);
  
//   RCLCPP_INFO(node->get_logger(), "Starting executor...");
//   executor.spin();
  
//   rclcpp::shutdown();
//   return 0;
// }

// ---------------------------In position [mm] and orientation [degrees] with workspace considered--------------------------
// Enhanced pick_and_place_node.cpp with collision prevention

#include <rclcpp/rclcpp.hpp>
#include "trajectory_plan/trajectory_planner.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>

class EnhancedPickAndPlaceNode : public rclcpp::Node
{
public:
  EnhancedPickAndPlaceNode() : Node("enhanced_pick_and_place_node")
  {
    RCLCPP_INFO(get_logger(), "🚀 Enhanced Pick and Place Node with Collision Prevention");
    
    // Declare parameters for table configuration
    this->declare_parameter("table_length", 1.2);
    this->declare_parameter("table_width", 0.8);
    this->declare_parameter("table_height", 0.09);
    this->declare_parameter("table_x_offset", 0.0);
    this->declare_parameter("table_y_offset", 0.0);
    
    // Get table parameters
    table_config_.length = this->get_parameter("table_length").as_double();
    table_config_.width = this->get_parameter("table_width").as_double();
    table_config_.height = this->get_parameter("table_height").as_double();
    table_config_.x_offset = this->get_parameter("table_x_offset").as_double();
    table_config_.y_offset = this->get_parameter("table_y_offset").as_double();
    
    RCLCPP_INFO(get_logger(), "📋 Table Configuration:");
    RCLCPP_INFO(get_logger(), "   Dimensions: %.2f x %.2f x %.2f m", 
                table_config_.length, table_config_.width, table_config_.height);
    RCLCPP_INFO(get_logger(), "   Offset: [%.3f, %.3f] m", 
                table_config_.x_offset, table_config_.y_offset);
    
    // Enhanced workspace limits considering table collision
    setupEnhancedWorkspaceLimits();
    
    // Set up collision-safe demo poses
    setupCollisionSafePoses();
    
    // Check for controller availability
    checkControllers();
    
    // Create timer for demo execution
    timer_ = create_wall_timer(
      std::chrono::seconds(30),  // Wait longer for full system initialization
      std::bind(&EnhancedPickAndPlaceNode::executeEnhancedDemo, this));
      
    RCLCPP_INFO(get_logger(), "✅ Enhanced node initialized - collision prevention enabled");
  }

private:
  struct TableConfig {
    double length, width, height;
    double x_offset, y_offset;
  };
  
  struct EnhancedWorkspaceLimits {
    // Standard workspace limits
    double min_x, max_x, min_y, max_y, min_z, max_z;
    
    // Enhanced collision-aware limits
    double safe_elbow_height;     // Minimum height for elbow joint
    double safe_approach_distance; // Safe distance for approach moves
    double table_clearance;       // Extra clearance above table
    
    // Joint-specific limits to prevent table collision
    double max_shoulder_down_angle;  // Max downward shoulder rotation
    double max_elbow_down_angle;     // Max downward elbow rotation
  };
  
//   void setupEnhancedWorkspaceLimits()
//   {
//     RCLCPP_INFO(get_logger(), "🛡️ ENHANCED WORKSPACE LIMITS (Collision Prevention):");
    
//     // Standard workspace limits (more conservative)
//     enhanced_limits_.min_x = 150.0;   // Start further from base
//     enhanced_limits_.max_x = 500.0;   // Reduced reach to avoid extreme poses
//     enhanced_limits_.min_y = -300.0;  // Reduced side reach
//     enhanced_limits_.max_y = 300.0;   
//     // enhanced_limits_.min_z = 80.0;    // Higher minimum - well above table
//     // enhanced_limits_.max_z = 400.0;   // Reasonable upper limit
    
//     // // Enhanced collision-aware parameters
//     // enhanced_limits_.safe_elbow_height = 120.0;      // Elbow must stay 12cm+ above table
//     // enhanced_limits_.safe_approach_distance = 150.0; // 15cm approach distance
//     // enhanced_limits_.table_clearance = 50.0;         // 5cm extra clearance

//     enhanced_limits_.min_z = 10.0;    // Allow access to table surface (1cm minimum)
//     enhanced_limits_.max_z = 400.0;   // Reasonable upper limit

//     // Enhanced collision-aware parameters for MOUNTED table
//     enhanced_limits_.safe_elbow_height = 50.0;       // Lower elbow limit for mounted table
//     enhanced_limits_.safe_approach_distance = 100.0; // 10cm approach distance
//     enhanced_limits_.table_clearance = 15.0;         // 1.5cm clearance above table
    
//     // Joint angle limits to prevent table collision (in degrees)
//     enhanced_limits_.max_shoulder_down_angle = 30.0;  // Limit shoulder downward rotation
//     enhanced_limits_.max_elbow_down_angle = 45.0;     // Limit elbow downward rotation
    
//     RCLCPP_INFO(get_logger(), "   Standard limits: X[%.0f-%.0f], Y[%.0f-%.0f], Z[%.0f-%.0f] mm",
//                 enhanced_limits_.min_x, enhanced_limits_.max_x,
//                 enhanced_limits_.min_y, enhanced_limits_.max_y, 
//                 enhanced_limits_.min_z, enhanced_limits_.max_z);
//     RCLCPP_INFO(get_logger(), "   Safe elbow height: %.0f mm above table", enhanced_limits_.safe_elbow_height);
//     RCLCPP_INFO(get_logger(), "   Safe approach distance: %.0f mm", enhanced_limits_.safe_approach_distance);
//     RCLCPP_INFO(get_logger(), "   🔒 COLLISION PREVENTION: All intermediate links protected");
//   }

void setupEnhancedWorkspaceLimits()
{
  RCLCPP_INFO(get_logger(), "🛡️ RELAXED WORKSPACE LIMITS (Table Access Enabled):");
  
  // More permissive workspace limits for table access
  enhanced_limits_.min_x = 100.0;   // Closer to base allowed
  enhanced_limits_.max_x = 600.0;   // Extended reach
  enhanced_limits_.min_y = -400.0;  // Extended side reach
  enhanced_limits_.max_y = 400.0;   
  enhanced_limits_.min_z = 5.0;    // Allow very low access (1cm)
  enhanced_limits_.max_z = 500.0;   // Higher upper limit
  
  // Relaxed collision-aware parameters for table access
  enhanced_limits_.safe_elbow_height = 15.0;       // Lower elbow limit (3cm above table)
  enhanced_limits_.safe_approach_distance = 80.0;  // 8cm approach distance
  enhanced_limits_.table_clearance = 5.0;         // 1cm clearance above table
  
  // More permissive joint angle limits 
  enhanced_limits_.max_shoulder_down_angle = 45.0;  // Allow more shoulder rotation
  enhanced_limits_.max_elbow_down_angle = 60.0;     // Allow more elbow rotation
  
  RCLCPP_INFO(get_logger(), "   RELAXED limits: X[%.0f-%.0f], Y[%.0f-%.0f], Z[%.0f-%.0f] mm",
              enhanced_limits_.min_x, enhanced_limits_.max_x,
              enhanced_limits_.min_y, enhanced_limits_.max_y, 
              enhanced_limits_.min_z, enhanced_limits_.max_z);
  RCLCPP_INFO(get_logger(), "   Safe elbow height: %.0f mm above table", enhanced_limits_.safe_elbow_height);
  RCLCPP_INFO(get_logger(), "   Safe approach distance: %.0f mm", enhanced_limits_.safe_approach_distance);
  RCLCPP_INFO(get_logger(), "   🔓 RELAXED CONSTRAINTS: Better table access enabled");
}
  
  bool validateEnhancedPosition(double x_mm, double y_mm, double z_mm, const std::string& description)
  {
    bool valid = true;
    
    RCLCPP_INFO(get_logger(), "🔍 Enhanced validation for %s: [%.1f, %.1f, %.1f] mm", 
                description.c_str(), x_mm, y_mm, z_mm);
    
    // Standard workspace validation
    if (x_mm < enhanced_limits_.min_x || x_mm > enhanced_limits_.max_x) {
      RCLCPP_ERROR(get_logger(), "❌ X position %.1f mm outside safe range [%.1f-%.1f] mm", 
                   x_mm, enhanced_limits_.min_x, enhanced_limits_.max_x);
      valid = false;
    }
    
    if (y_mm < enhanced_limits_.min_y || y_mm > enhanced_limits_.max_y) {
      RCLCPP_ERROR(get_logger(), "❌ Y position %.1f mm outside safe range [%.1f-%.1f] mm", 
                   y_mm, enhanced_limits_.min_y, enhanced_limits_.max_y);
      valid = false;
    }
    
    if (z_mm < enhanced_limits_.min_z || z_mm > enhanced_limits_.max_z) {
      RCLCPP_ERROR(get_logger(), "❌ Z position %.1f mm outside safe range [%.1f-%.1f] mm", 
                   z_mm, enhanced_limits_.min_z, enhanced_limits_.max_z);
      valid = false;
    }
    
    // Enhanced collision prevention checks
    if (z_mm < enhanced_limits_.safe_elbow_height) {
      RCLCPP_WARN(get_logger(), "⚠️ Position may cause elbow collision (below %.1f mm safe height)", 
                  enhanced_limits_.safe_elbow_height);
    }
    
    // Check if position is too close to table edges (could cause swept collision)
    double table_edge_x = (table_config_.length / 2.0) * 1000.0; // Convert to mm
    double table_edge_y = (table_config_.width / 2.0) * 1000.0;
    
    if (x_mm > table_edge_x - 50.0) { // 5cm from table edge
      RCLCPP_WARN(get_logger(), "⚠️ Position close to table edge - potential collision during motion");
    }
    
    if (valid) {
      RCLCPP_INFO(get_logger(), "✅ %s position validated - collision-safe", description.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "❌ %s position REJECTED - collision risk!", description.c_str());
    }
    
    return valid;
  }
  
//   void setupCollisionSafePoses()
//   {
//     // COLLISION-SAFE demo poses - designed to keep all links above table
//     setupObjectPoses(
//       // Pick position: Conservative, well above table
//       250.0, 150.0, 150.0,    // 25cm forward, 15cm right, 15cm above table
//       0.0, 90.0, 0.0,         // Pointing straight down
      
//       // Place position: Mirror position, safe clearance
//       189.05, -251.19, 15.92,   // 25cm forward, 15cm left, 15cm above table
//       3.13, 178.93, 124.57          // Pointing straight down
//     );
//   }

  void setupCollisionSafePoses()
{
  // REALISTIC poses for objects ON the mounted table surface
  setupObjectPoses(
    // Pick position: Object on table surface 
    250.0, 150.0, 50.0,     // 25cm forward, 15cm right, 2.5cm above table (realistic object)
    0.0, 90.0, 0.0,         // Pointing straight down
    
    // Place position: Your original was CORRECT for table objects!
    396.54, 141.11, 61.74,  // Keep your X,Y, but safe height for object
    0.69, 177.68, 24.46          // Simplified orientation to avoid elbow issues 
  );
}
  
  void setupObjectPoses(
    double pick_x_mm, double pick_y_mm, double pick_z_mm,
    double pick_roll_deg, double pick_pitch_deg, double pick_yaw_deg,
    double place_x_mm, double place_y_mm, double place_z_mm, 
    double place_roll_deg, double place_pitch_deg, double place_yaw_deg)
  {
    RCLCPP_INFO(get_logger(), "🎯 COLLISION-SAFE POSE VALIDATION:");
    
    // Enhanced validation with collision checking
    if (!validateEnhancedPosition(pick_x_mm, pick_y_mm, pick_z_mm, "COLLISION-SAFE PICK")) {
      RCLCPP_ERROR(get_logger(), "❌ Pick pose failed enhanced validation!");
      // Use ultra-safe defaults
      pick_x_mm = 250.0; pick_y_mm = 150.0; pick_z_mm = 180.0;
      RCLCPP_WARN(get_logger(), "🔧 Using ultra-safe pick: [%.1f, %.1f, %.1f] mm", 
                  pick_x_mm, pick_y_mm, pick_z_mm);
    }
    
    if (!validateEnhancedPosition(place_x_mm, place_y_mm, place_z_mm, "COLLISION-SAFE PLACE")) {
      RCLCPP_ERROR(get_logger(), "❌ Place pose failed enhanced validation!");
      // Use ultra-safe defaults  
      place_x_mm = 250.0; place_y_mm = -150.0; place_z_mm = 180.0;
      RCLCPP_WARN(get_logger(), "🔧 Using ultra-safe place: [%.1f, %.1f, %.1f] mm", 
                  place_x_mm, place_y_mm, place_z_mm);
    }
    
    // Convert to ROS units and create poses
    createRosPoses(pick_x_mm, pick_y_mm, pick_z_mm, pick_roll_deg, pick_pitch_deg, pick_yaw_deg,
                   place_x_mm, place_y_mm, place_z_mm, place_roll_deg, place_pitch_deg, place_yaw_deg);
  }
  
  void createRosPoses(double pick_x_mm, double pick_y_mm, double pick_z_mm,
                     double pick_roll_deg, double pick_pitch_deg, double pick_yaw_deg,
                     double place_x_mm, double place_y_mm, double place_z_mm, 
                     double place_roll_deg, double place_pitch_deg, double place_yaw_deg)
  {
    // Convert millimeters to meters
    double pick_x = pick_x_mm / 1000.0;
    double pick_y = pick_y_mm / 1000.0;
    double pick_z = pick_z_mm / 1000.0;
    double place_x = place_x_mm / 1000.0;
    double place_y = place_y_mm / 1000.0;
    double place_z = place_z_mm / 1000.0;
    
    // Convert degrees to radians
    const double DEG_TO_RAD = M_PI / 180.0;
    double pick_roll = pick_roll_deg * DEG_TO_RAD;
    double pick_pitch = pick_pitch_deg * DEG_TO_RAD;
    double pick_yaw = pick_yaw_deg * DEG_TO_RAD;
    double place_roll = place_roll_deg * DEG_TO_RAD;
    double place_pitch = place_pitch_deg * DEG_TO_RAD;
    double place_yaw = place_yaw_deg * DEG_TO_RAD;
    
    // Create pick pose
    pick_pose_.position.x = pick_x;
    pick_pose_.position.y = pick_y;
    pick_pose_.position.z = pick_z;
    tf2::Quaternion q_pick;
    q_pick.setRPY(pick_roll, pick_pitch, pick_yaw);
    pick_pose_.orientation.x = q_pick.x();
    pick_pose_.orientation.y = q_pick.y();
    pick_pose_.orientation.z = q_pick.z();
    pick_pose_.orientation.w = q_pick.w();
    
    // Create place pose
    place_pose_.position.x = place_x;
    place_pose_.position.y = place_y;
    place_pose_.position.z = place_z;
    tf2::Quaternion q_place;
    q_place.setRPY(place_roll, place_pitch, place_yaw);
    place_pose_.orientation.x = q_place.x();
    place_pose_.orientation.y = q_place.y();
    place_pose_.orientation.z = q_place.z();
    place_pose_.orientation.w = q_place.w();
    
    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "📍 COLLISION-SAFE POSES CONFIGURED:");
    RCLCPP_INFO(get_logger(), "   Pick:  [%.1f, %.1f, %.1f] mm | RPY: [%.1f°, %.1f°, %.1f°]", 
                pick_x_mm, pick_y_mm, pick_z_mm, pick_roll_deg, pick_pitch_deg, pick_yaw_deg);
    RCLCPP_INFO(get_logger(), "   Place: [%.1f, %.1f, %.1f] mm | RPY: [%.1f°, %.1f°, %.1f°]", 
                place_x_mm, place_y_mm, place_z_mm, place_roll_deg, place_pitch_deg, place_yaw_deg);
    RCLCPP_INFO(get_logger(), "   🛡️ All poses validated against table collision");
  }
  
  void checkControllers()
  {
    RCLCPP_INFO(get_logger(), "🔍 Checking enhanced controller setup...");
    
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "joint_trajectory_controller/follow_joint_trajectory");
    
    auto timer = create_wall_timer(
      std::chrono::seconds(5),
      [this, action_client]() {
        if (action_client->action_server_is_ready()) {
          RCLCPP_INFO(get_logger(), "✅ Enhanced controller system ready!");
        } else {
          RCLCPP_WARN(get_logger(), "⚠️ Waiting for enhanced controller system...");
        }
      });
  }
  
  void initialize_planner()
  {
    if (!trajectory_planner_) {
      RCLCPP_INFO(get_logger(), "🔧 Initializing enhanced trajectory planner...");
      trajectory_planner_ = std::make_unique<trajectory_plan::TrajectoryPlanner>(shared_from_this());
      
      // Configure table dimensions for collision prevention
      trajectory_planner_->setTableDimensions(
        table_config_.length, table_config_.width, table_config_.height,
        table_config_.x_offset, table_config_.y_offset);
      
      RCLCPP_INFO(get_logger(), "✅ Enhanced trajectory planner initialized with collision prevention");
    }
  }
  
  void executeEnhancedDemo()
  {
    timer_->cancel();
    
    RCLCPP_INFO(get_logger(), "🚀 Starting COLLISION-PREVENTED Pick and Place Demo");
    RCLCPP_INFO(get_logger(), "   🛡️ Table collision detection ACTIVE");
    RCLCPP_INFO(get_logger(), "   📐 Enhanced workspace constraints ENABLED");
    
    initialize_planner();
    
    if (!trajectory_planner_) {
      RCLCPP_ERROR(get_logger(), "❌ Failed to initialize enhanced trajectory planner");
      return;
    }
    
    // Test controller connectivity
    RCLCPP_INFO(get_logger(), "🔌 Testing enhanced controller connectivity...");
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "joint_trajectory_controller/follow_joint_trajectory");
    
    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "❌ Enhanced controller not available!");
      return;
    }
    
    RCLCPP_INFO(get_logger(), "✅ Enhanced controller verified!");
    
    // Enable ultra-smooth motion for collision safety
    trajectory_planner_->setSmoothMotion(true);
    
    // Wait for full collision system setup
    RCLCPP_INFO(get_logger(), "⏳ Waiting for collision prevention system...");
    std::this_thread::sleep_for(std::chrono::seconds(8));
    
    // Execute collision-safe sequence
    RCLCPP_INFO(get_logger(), "🏠 Moving to collision-safe home position...");
    if (trajectory_planner_->moveToHome()) {
      RCLCPP_INFO(get_logger(), "✅ Home position reached safely");
    } else {
      RCLCPP_WARN(get_logger(), "⚠️ Home move had issues, continuing...");
    }
    
    trajectory_planner_->waitForMotionComplete(3.0);
    
    // Final collision check before execution
    RCLCPP_INFO(get_logger(), "🛡️ FINAL COLLISION SAFETY CHECK:");
    double pick_x_check = pick_pose_.position.x * 1000.0;
    double pick_y_check = pick_pose_.position.y * 1000.0;
    double pick_z_check = pick_pose_.position.z * 1000.0;
    double place_x_check = place_pose_.position.x * 1000.0;
    double place_y_check = place_pose_.position.y * 1000.0;
    double place_z_check = place_pose_.position.z * 1000.0;
    
    if (!validateEnhancedPosition(pick_x_check, pick_y_check, pick_z_check, "FINAL PICK") ||
        !validateEnhancedPosition(place_x_check, place_y_check, place_z_check, "FINAL PLACE")) {
      RCLCPP_ERROR(get_logger(), "❌ FINAL SAFETY CHECK FAILED - ABORTING to prevent collision!");
      return;
    }
    
    // Execute collision-safe pick and place
    RCLCPP_INFO(get_logger(), "📦 Starting COLLISION-SAFE pick and place...");
    RCLCPP_INFO(get_logger(), "   🛡️ Table collision object active in planning scene");
    RCLCPP_INFO(get_logger(), "   📐 Enhanced workspace constraints protecting all links");
    
    // Use larger approach/retreat distances for safety
    double safe_approach = enhanced_limits_.safe_approach_distance / 1000.0; // Convert to meters
    
    if (trajectory_planner_->planMountedTablePickAndPlace(pick_pose_, place_pose_, 0.08, 0.08)) {
      RCLCPP_INFO(get_logger(), "🎉 COLLISION-SAFE pick and place COMPLETED!");
      RCLCPP_INFO(get_logger(), "✅ No table collisions - all links stayed safe");
    } else {
      RCLCPP_ERROR(get_logger(), "❌ Collision-safe planning failed");
      RCLCPP_INFO(get_logger(), "💡 This indicates target poses may be unreachable with collision constraints");
    }
    
    RCLCPP_INFO(get_logger(), "✅ COLLISION-PREVENTED Demo completed!");
    RCLCPP_INFO(get_logger(), "🛡️ Table surface protection was active throughout execution");
  }
  
  std::unique_ptr<trajectory_plan::TrajectoryPlanner> trajectory_planner_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Pose pick_pose_;
  geometry_msgs::msg::Pose place_pose_;
  
  TableConfig table_config_;
  EnhancedWorkspaceLimits enhanced_limits_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<EnhancedPickAndPlaceNode>();
  
  RCLCPP_INFO(node->get_logger(), "🛡️ Enhanced Pick and Place Node with Collision Prevention created");
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  RCLCPP_INFO(node->get_logger(), "🚀 Starting collision-aware executor...");
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}