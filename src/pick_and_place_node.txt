#include <rclcpp/rclcpp.hpp>
#include "trajectory_plan/trajectory_planner.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class PickAndPlaceNode : public rclcpp::Node
{
public:
  PickAndPlaceNode() : Node("pick_and_place_node")
  {
    RCLCPP_INFO(get_logger(), "🚀 Pick and Place Node starting...");
    
    // Load configuration from parameters
    loadConfiguration();
    
    // Print configuration
    printConfiguration();
    
    // Set up demo poses
    setupDemoPoses();
    
    // Initialize planner (will be done in timer callback)
    trajectory_planner_ = nullptr;
    
    // Create timer for delayed initialization and demo execution
    timer_ = create_wall_timer(
      std::chrono::seconds(5),  // Give system time to initialize
      std::bind(&PickAndPlaceNode::initializeAndExecuteDemo, this));
      
    RCLCPP_INFO(get_logger(), "✅ Node initialized - demo will start in 5 seconds");
  }

private:
  void loadConfiguration()
  {
    // Declare and get parameters with defaults
    declare_parameter("table.length", 0.98);
    declare_parameter("table.width", 0.49);
    declare_parameter("table.height", 0.04);
    declare_parameter("table.x_offset", 0.0);
    declare_parameter("table.y_offset", 0.0);
    
    declare_parameter("workspace.width", 1.4);
    declare_parameter("workspace.depth", 1.2);
    declare_parameter("workspace.height", 1.0);
    declare_parameter("workspace.x_position", 0.4);
    declare_parameter("workspace.y_position", 0.0);
    declare_parameter("workspace.z_position", 0.5);
    declare_parameter("workspace.visualization_enabled", true);
    
    declare_parameter("robot.velocity_scaling", 0.3);
    declare_parameter("robot.acceleration_scaling", 0.3);
    declare_parameter("robot.smooth_motion", true);
    
    declare_parameter("demo.pick_x_mm", 250.0);  // Changed to mm
    declare_parameter("demo.pick_y_mm", 150.0);
    declare_parameter("demo.pick_z_mm", 50.0);
    declare_parameter("demo.pick_roll_deg", 0.0);  // Added orientation in degrees
    declare_parameter("demo.pick_pitch_deg", 90.0);
    declare_parameter("demo.pick_yaw_deg", 0.0);
    
    declare_parameter("demo.place_x_mm", 350.0);  // Changed to mm
    declare_parameter("demo.place_y_mm", -150.0);
    declare_parameter("demo.place_z_mm", 50.0);
    declare_parameter("demo.place_roll_deg", 0.0);  // Added orientation in degrees
    declare_parameter("demo.place_pitch_deg", 90.0);
    declare_parameter("demo.place_yaw_deg", 0.0);
    
    declare_parameter("demo.clearance_height_mm", 50.0);  // Added clearance height
    
    // Load configuration
    config_.table.length = get_parameter("table.length").as_double();
    config_.table.width = get_parameter("table.width").as_double();
    config_.table.height = get_parameter("table.height").as_double();
    config_.table.x_offset = get_parameter("table.x_offset").as_double();
    config_.table.y_offset = get_parameter("table.y_offset").as_double();
    
    config_.workspace.width = get_parameter("workspace.width").as_double();
    config_.workspace.depth = get_parameter("workspace.depth").as_double();
    config_.workspace.height = get_parameter("workspace.height").as_double();
    config_.workspace.x_position = get_parameter("workspace.x_position").as_double();
    config_.workspace.y_position = get_parameter("workspace.y_position").as_double();
    config_.workspace.z_position = get_parameter("workspace.z_position").as_double();
    config_.workspace.visualization_enabled = get_parameter("workspace.visualization_enabled").as_bool();
    
    config_.robot.velocity_scaling = get_parameter("robot.velocity_scaling").as_double();
    config_.robot.acceleration_scaling = get_parameter("robot.acceleration_scaling").as_double();
    
    // Load demo poses (now in mm/degrees)
    demo_config_.pick_x_mm = get_parameter("demo.pick_x_mm").as_double();
    demo_config_.pick_y_mm = get_parameter("demo.pick_y_mm").as_double();
    demo_config_.pick_z_mm = get_parameter("demo.pick_z_mm").as_double();
    demo_config_.pick_roll_deg = get_parameter("demo.pick_roll_deg").as_double();
    demo_config_.pick_pitch_deg = get_parameter("demo.pick_pitch_deg").as_double();
    demo_config_.pick_yaw_deg = get_parameter("demo.pick_yaw_deg").as_double();
    
    demo_config_.place_x_mm = get_parameter("demo.place_x_mm").as_double();
    demo_config_.place_y_mm = get_parameter("demo.place_y_mm").as_double();
    demo_config_.place_z_mm = get_parameter("demo.place_z_mm").as_double();
    demo_config_.place_roll_deg = get_parameter("demo.place_roll_deg").as_double();
    demo_config_.place_pitch_deg = get_parameter("demo.place_pitch_deg").as_double();
    demo_config_.place_yaw_deg = get_parameter("demo.place_yaw_deg").as_double();
    
    demo_config_.clearance_height_mm = get_parameter("demo.clearance_height_mm").as_double();
    demo_config_.smooth_motion = get_parameter("robot.smooth_motion").as_bool();
  }
  
  void printConfiguration()
  {
    RCLCPP_INFO(get_logger(), "📋 Configuration loaded:");
    RCLCPP_INFO(get_logger(), "   Table: %.2f x %.2f x %.2f m at [%.2f, %.2f]", 
                config_.table.length, config_.table.width, config_.table.height,
                config_.table.x_offset, config_.table.y_offset);
    RCLCPP_INFO(get_logger(), "   Workspace: %.2f x %.2f x %.2f m at [%.2f, %.2f, %.2f]", 
                config_.workspace.width, config_.workspace.depth, config_.workspace.height,
                config_.workspace.x_position, config_.workspace.y_position, config_.workspace.z_position);
    RCLCPP_INFO(get_logger(), "   Robot: vel=%.2f, acc=%.2f", 
                config_.robot.velocity_scaling, config_.robot.acceleration_scaling);
    RCLCPP_INFO(get_logger(), "   Demo: pick=[%.1f, %.1f, %.1f] mm, [%.1f°, %.1f°, %.1f°]", 
                demo_config_.pick_x_mm, demo_config_.pick_y_mm, demo_config_.pick_z_mm,
                demo_config_.pick_roll_deg, demo_config_.pick_pitch_deg, demo_config_.pick_yaw_deg);
    RCLCPP_INFO(get_logger(), "         place=[%.1f, %.1f, %.1f] mm, [%.1f°, %.1f°, %.1f°]", 
                demo_config_.place_x_mm, demo_config_.place_y_mm, demo_config_.place_z_mm,
                demo_config_.place_roll_deg, demo_config_.place_pitch_deg, demo_config_.place_yaw_deg);
    RCLCPP_INFO(get_logger(), "         clearance=%.1f mm", demo_config_.clearance_height_mm);
  }
  
  void setupDemoPoses()
  {
    RCLCPP_INFO(get_logger(), "🎯 Demo poses configured:");
    RCLCPP_INFO(get_logger(), "   Pick:  [%.1f, %.1f, %.1f] mm, [%.1f°, %.1f°, %.1f°]", 
                demo_config_.pick_x_mm, demo_config_.pick_y_mm, demo_config_.pick_z_mm,
                demo_config_.pick_roll_deg, demo_config_.pick_pitch_deg, demo_config_.pick_yaw_deg);
    RCLCPP_INFO(get_logger(), "   Place: [%.1f, %.1f, %.1f] mm, [%.1f°, %.1f°, %.1f°]", 
                demo_config_.place_x_mm, demo_config_.place_y_mm, demo_config_.place_z_mm,
                demo_config_.place_roll_deg, demo_config_.place_pitch_deg, demo_config_.place_yaw_deg);
    RCLCPP_INFO(get_logger(), "   Clearance: %.1f mm", demo_config_.clearance_height_mm);
  }
  
  void initializeAndExecuteDemo()
  {
    timer_->cancel();  // Run only once
    
    RCLCPP_INFO(get_logger(), "🔧 Initializing trajectory planner...");
    
    // Create and initialize trajectory planner
    trajectory_planner_ = std::make_unique<trajectory_plan::TrajectoryPlanner>(shared_from_this());
    
    if (!trajectory_planner_->initialize()) {
      RCLCPP_ERROR(get_logger(), "❌ Failed to initialize trajectory planner");
      return;
    }
    
    // Update configuration
    trajectory_planner_->updateConfiguration(config_);
    
    // Enable workspace visualization
    if (config_.workspace.visualization_enabled) {
      trajectory_planner_->enableWorkspaceVisualization(true);
      trajectory_planner_->setWorkspaceVisualizationColor(1.0, 0.0, 0.0, 1.0); // Red
    }
    
    // Wait for controller to be ready
    if (!waitForController()) {
      RCLCPP_ERROR(get_logger(), "❌ Controller not ready");
      return;
    }
    
    // Execute the demo
    executeDemo();
  }
  
  bool waitForController()
  {
    RCLCPP_INFO(get_logger(), "🔌 Waiting for controller...");
    
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "joint_trajectory_controller/follow_joint_trajectory");
    
    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "❌ Controller not available");
      return false;
    }
    
    RCLCPP_INFO(get_logger(), "✅ Controller ready");
    return true;
  }
  
  void executeDemo()
  {
    RCLCPP_INFO(get_logger(), "🚀 Starting pick and place demo...");
    
    // Configure motion settings
    if (demo_config_.smooth_motion) {
      trajectory_planner_->setSmoothMotion(true);
    }
    
    // Print workspace limits
    trajectory_planner_->printWorkspaceLimits();
    
    // Use professional pick-and-place with mm/degrees input
    RCLCPP_INFO(get_logger(), "📦 Executing PROFESSIONAL pick and place...");
    bool success = trajectory_planner_->executeProfessionalPickAndPlace(
      demo_config_.pick_x_mm, demo_config_.pick_y_mm, demo_config_.pick_z_mm,
      demo_config_.pick_roll_deg, demo_config_.pick_pitch_deg, demo_config_.pick_yaw_deg,
      demo_config_.place_x_mm, demo_config_.place_y_mm, demo_config_.place_z_mm,
      demo_config_.place_roll_deg, demo_config_.place_pitch_deg, demo_config_.place_yaw_deg,
      demo_config_.clearance_height_mm
    );
    
    if (success) {
      RCLCPP_INFO(get_logger(), "🎉 PROFESSIONAL pick and place demo completed successfully!");
    } else {
      RCLCPP_ERROR(get_logger(), "❌ PROFESSIONAL pick and place demo failed");
    }
    
    RCLCPP_INFO(get_logger(), "✅ Demo sequence completed");
  }
  
  // Configuration structures
  trajectory_plan::PlannerConfiguration config_;
  
  struct DemoConfiguration {
    double pick_x_mm, pick_y_mm, pick_z_mm;
    double pick_roll_deg, pick_pitch_deg, pick_yaw_deg;
    double place_x_mm, place_y_mm, place_z_mm;
    double place_roll_deg, place_pitch_deg, place_yaw_deg;
    double clearance_height_mm;
    bool smooth_motion;
  } demo_config_;
  
  // Components
  std::unique_ptr<trajectory_plan::TrajectoryPlanner> trajectory_planner_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<PickAndPlaceNode>();
  
  RCLCPP_INFO(node->get_logger(), "🚀 Pick and Place Node created");
  
  // Use multi-threaded executor for better performance
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  RCLCPP_INFO(node->get_logger(), "🔄 Executor starting...");
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}