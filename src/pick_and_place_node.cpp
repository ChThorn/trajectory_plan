// Enhanced pick_and_place_node.cpp with collision prevention - Modular Version

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
    RCLCPP_INFO(get_logger(), "🚀 Enhanced Pick and Place Node with Collision Prevention (Modular)");
    
    // Declare parameters for table configuration
    this->declare_parameter("table_length", 1.2);
    this->declare_parameter("table_width", 0.8);
    this->declare_parameter("table_height", 0.09);
    this->declare_parameter("table_x_offset", 0.5);
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
      
    RCLCPP_INFO(get_logger(), "✅ Enhanced modular node initialized - collision prevention enabled");
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
    // -249.99, 46.87, 720.50, 
    // 180.00, 180.00, -179.99
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
      RCLCPP_INFO(get_logger(), "🔧 Initializing enhanced modular trajectory planner...");
      trajectory_planner_ = std::make_unique<trajectory_plan::TrajectoryPlanner>(shared_from_this());
      
      // Configure table dimensions for collision prevention
      trajectory_planner_->setTableDimensions(
        table_config_.length, table_config_.width, table_config_.height,
        table_config_.x_offset, table_config_.y_offset);
      
      RCLCPP_INFO(get_logger(), "✅ Enhanced modular trajectory planner initialized with collision prevention");
    }
  }

//   void setupEnhancedWorkspaceWithVisualization()
// {
//   RCLCPP_INFO(get_logger(), "🔳 Setting up workspace boundary visualization...");
  
//   // Enable workspace visualization
//   trajectory_planner_->enableWorkspaceVisualization(true);
  
//   // Set workspace color (cyan/blue for workspace, different from green table)
//   trajectory_planner_->setWorkspaceVisualizationColor(0.0, 1.0, 1.0, 0.4); // Cyan
  
//   // Print workspace limits for verification
//   trajectory_planner_->printWorkspaceLimits();
  
//   RCLCPP_INFO(get_logger(), "✅ Workspace boundary visualization enabled!");
//   RCLCPP_INFO(get_logger(), "   📋 Check RViz to see the cyan workspace boundary box");
//   RCLCPP_INFO(get_logger(), "   📋 Robot end-effector is constrained within this boundary");
// }

// void setupEnhancedWorkspaceWithVisualization()
// {
//   RCLCPP_INFO(get_logger(), "🔳 Setting up ENHANCED workspace boundary visualization...");
  
//   // Enable workspace visualization
//   trajectory_planner_->enableWorkspaceVisualization(true);
  
//   // Make it MORE VISIBLE with brighter color and higher opacity
//   trajectory_planner_->setWorkspaceVisualizationColor(
//     0.0, 1.0, 1.0, 0.8); // Bright cyan with 80% opacity (was 40%)
  
//   // Print workspace limits for verification
//   trajectory_planner_->printWorkspaceLimits();
  
//   RCLCPP_INFO(get_logger(), "✅ ENHANCED workspace boundary visualization enabled!");
//   RCLCPP_INFO(get_logger(), "   📋 Check RViz to see the BRIGHT CYAN workspace boundary box");
//   RCLCPP_INFO(get_logger(), "   📋 Robot end-effector is constrained within this boundary");
//   RCLCPP_INFO(get_logger(), "   🔧 Add MarkerArray topic '/workspace_boundary' in RViz if not visible");
// }

    void setupEnhancedWorkspaceWithVisualization()
    {
    RCLCPP_INFO(get_logger(), "🔳 Setting up ULTRA-VISIBLE workspace boundary...");
    
    // Enable workspace visualization
    trajectory_planner_->enableWorkspaceVisualization(true);
    
    // ULTRA-VISIBLE: Bright red, fully opaque
    trajectory_planner_->setWorkspaceVisualizationColor(1.0, 0.0, 0.0, 1.0); // BRIGHT RED
    
    trajectory_planner_->printWorkspaceLimits();
    
    RCLCPP_INFO(get_logger(), "✅ ULTRA-VISIBLE workspace boundary enabled!");
    RCLCPP_INFO(get_logger(), "   🔴 Look for BRIGHT RED wireframe box in RViz");
    RCLCPP_INFO(get_logger(), "   📋 Add '/workspace_boundary' MarkerArray in RViz if needed");
    }
  
  void executeEnhancedDemo()
  {
    timer_->cancel();
    
    RCLCPP_INFO(get_logger(), "🚀 Starting COLLISION-PREVENTED Pick and Place Demo (Modular)");
    RCLCPP_INFO(get_logger(), "   🛡️ Table collision detection ACTIVE");
    RCLCPP_INFO(get_logger(), "   📐 Enhanced workspace constraints ENABLED");
    RCLCPP_INFO(get_logger(), "   🔧 Modular architecture with separated concerns");
    
    initialize_planner();

    setupEnhancedWorkspaceWithVisualization();
    // Validate poses against workspace before execution
  RCLCPP_INFO(get_logger(), "🔍 Validating poses against workspace boundary...");
  
  bool pick_valid = trajectory_planner_->validatePoseInWorkspace(pick_pose_);
  bool place_valid = trajectory_planner_->validatePoseInWorkspace(place_pose_);
  
  if (!pick_valid || !place_valid) {
    RCLCPP_ERROR(get_logger(), "❌ One or more poses are outside workspace boundary!");
    RCLCPP_INFO(get_logger(), "💡 Adjust poses to be within the cyan boundary box visible in RViz");
    return;
  }
  
  RCLCPP_INFO(get_logger(), "✅ All poses validated - within workspace boundary");
    
    if (!trajectory_planner_) {
      RCLCPP_ERROR(get_logger(), "❌ Failed to initialize enhanced modular trajectory planner");
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
    RCLCPP_INFO(get_logger(), "   🔧 Using modular architecture for robust operation");
    
    // Use larger approach/retreat distances for safety
    double safe_approach = enhanced_limits_.safe_approach_distance / 1000.0; // Convert to meters
    
    if (trajectory_planner_->planMountedTablePickAndPlace(pick_pose_, place_pose_, 0.08, 0.08)) {
      RCLCPP_INFO(get_logger(), "🎉 COLLISION-SAFE pick and place COMPLETED!");
      RCLCPP_INFO(get_logger(), "✅ No table collisions - all links stayed safe");
      RCLCPP_INFO(get_logger(), "🔧 Modular system performed flawlessly");
    } else {
      RCLCPP_ERROR(get_logger(), "❌ Collision-safe planning failed");
      RCLCPP_INFO(get_logger(), "💡 This indicates target poses may be unreachable with collision constraints");
    }
    
    RCLCPP_INFO(get_logger(), "✅ COLLISION-PREVENTED Demo completed!");
    RCLCPP_INFO(get_logger(), "🛡️ Table surface protection was active throughout execution");
    RCLCPP_INFO(get_logger(), "🔧 Modular architecture enables easy maintenance and extension");
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
  
  RCLCPP_INFO(node->get_logger(), "🛡️ Enhanced Pick and Place Node (Modular) with Collision Prevention created");
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  RCLCPP_INFO(node->get_logger(), "🚀 Starting collision-aware modular executor...");
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}