#include <rclcpp/rclcpp.hpp>
#include "trajectory_plan/trajectory_planner.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

/**
 * @class PickAndPlaceNode
 * @brief The main node for running the pick and place demonstration.
 *
 * This node is responsible for loading parameters, initializing the TrajectoryPlanner,
 * and triggering the main pick and place sequence.
 */
class PickAndPlaceNode : public rclcpp::Node
{
public:
  PickAndPlaceNode() : Node("pick_and_place_node")
  {
    RCLCPP_INFO(get_logger(), "🚀 Pick and Place Node starting...");
    
    // Load parameters from the config file
    loadConfiguration();
    printConfiguration();
    
    // A timer ensures that initialization doesn't start until the rest of the ROS 2 system
    // (e.g., robot_state_publisher, MoveIt servers) has had time to launch.
    timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&PickAndPlaceNode::main_task, this));
  }

private:
  /**
   * @brief Loads all necessary parameters from the ROS parameter server.
   */
  void loadConfiguration()
  {
    // Declare and read parameters with default values
    this->declare_parameter<double>("table.length", 0.98);
    this->declare_parameter<double>("table.width", 0.49);
    this->declare_parameter<double>("table.height", 0.04);
    this->declare_parameter<double>("table.x_offset", 0.0);
    this->declare_parameter<double>("table.y_offset", 0.0);
    this->declare_parameter<double>("table.z_position", -0.027);

    this->declare_parameter<double>("workspace.width", 1.4);
    this->declare_parameter<double>("workspace.depth", 1.2);
    this->declare_parameter<double>("workspace.height", 1.0);
    this->declare_parameter<double>("workspace.x_position", 0.4);
    this->declare_parameter<double>("workspace.y_position", 0.0);
    this->declare_parameter<double>("workspace.z_position", 0.5);
    this->declare_parameter<bool>("workspace.visualization_enabled", true);
    
    this->declare_parameter<double>("robot.velocity_scaling", 0.3);
    this->declare_parameter<double>("robot.acceleration_scaling", 0.3);
    this->declare_parameter<bool>("robot.smooth_motion", true);
    
    this->declare_parameter<double>("demo.pick_x_mm", 250.0);
    this->declare_parameter<double>("demo.pick_y_mm", 150.0);
    this->declare_parameter<double>("demo.pick_z_mm", 50.0);
    this->declare_parameter<double>("demo.pick_roll_deg", 0.0);
    this->declare_parameter<double>("demo.pick_pitch_deg", 90.0);
    this->declare_parameter<double>("demo.pick_yaw_deg", 0.0);
    
    this->declare_parameter<double>("demo.place_x_mm", 350.0);
    this->declare_parameter<double>("demo.place_y_mm", -150.0);
    this->declare_parameter<double>("demo.place_z_mm", 50.0);
    this->declare_parameter<double>("demo.place_roll_deg", 0.0);
    this->declare_parameter<double>("demo.place_pitch_deg", 90.0);
    this->declare_parameter<double>("demo.place_yaw_deg", 0.0);
    
    this->declare_parameter<double>("demo.clearance_height_mm", 50.0);
    
    // Read parameters into the config structs
    config_.table.length = this->get_parameter("table.length").as_double();
    config_.table.width = this->get_parameter("table.width").as_double();
    config_.table.height = this->get_parameter("table.height").as_double();
    config_.table.x_offset = this->get_parameter("table.x_offset").as_double();
    config_.table.y_offset = this->get_parameter("table.y_offset").as_double();
    config_.table.z_position = this->get_parameter("table.z_position").as_double();
    
    config_.workspace.width = this->get_parameter("workspace.width").as_double();
    config_.workspace.depth = this->get_parameter("workspace.depth").as_double();
    config_.workspace.height = this->get_parameter("workspace.height").as_double();
    config_.workspace.x_position = this->get_parameter("workspace.x_position").as_double();
    config_.workspace.y_position = this->get_parameter("workspace.y_position").as_double();
    config_.workspace.z_position = this->get_parameter("workspace.z_position").as_double();
    config_.workspace.visualization_enabled = this->get_parameter("workspace.visualization_enabled").as_bool();
    
    config_.robot.velocity_scaling = this->get_parameter("robot.velocity_scaling").as_double();
    config_.robot.acceleration_scaling = this->get_parameter("robot.acceleration_scaling").as_double();
    
    demo_config_.pick_x_mm = this->get_parameter("demo.pick_x_mm").as_double();
    demo_config_.pick_y_mm = this->get_parameter("demo.pick_y_mm").as_double();
    demo_config_.pick_z_mm = this->get_parameter("demo.pick_z_mm").as_double();
    demo_config_.pick_roll_deg = this->get_parameter("demo.pick_roll_deg").as_double();
    demo_config_.pick_pitch_deg = this->get_parameter("demo.pick_pitch_deg").as_double();
    demo_config_.pick_yaw_deg = this->get_parameter("demo.pick_yaw_deg").as_double();
    
    demo_config_.place_x_mm = this->get_parameter("demo.place_x_mm").as_double();
    demo_config_.place_y_mm = this->get_parameter("demo.place_y_mm").as_double();
    demo_config_.place_z_mm = this->get_parameter("demo.place_z_mm").as_double();
    demo_config_.place_roll_deg = this->get_parameter("demo.place_roll_deg").as_double();
    demo_config_.place_pitch_deg = this->get_parameter("demo.place_pitch_deg").as_double();
    demo_config_.place_yaw_deg = this->get_parameter("demo.place_yaw_deg").as_double();
    
    demo_config_.clearance_height_mm = this->get_parameter("demo.clearance_height_mm").as_double();
    demo_config_.smooth_motion = this->get_parameter("robot.smooth_motion").as_bool();
  }
  
  /**
   * @brief Prints the loaded configuration to the console.
   */
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
  
  /**
   * @brief The main task triggered by the timer to initialize and run the demo.
   */
  void main_task()
  {
    timer_->cancel(); // Ensure this task runs only once
    
    RCLCPP_INFO(get_logger(), "🔌 Checking for controller action server...");
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "joint_trajectory_controller/follow_joint_trajectory");
      
    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "❌ Controller action server not available after 10s. Shutting down.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "✅ Controller is ready.");
    
    // 1. Create the planner instance
    trajectory_planner_ = std::make_unique<trajectory_plan::TrajectoryPlanner>(shared_from_this());
    
    // 2. Pass the loaded configuration to the planner
    trajectory_planner_->updateConfiguration(config_);

    // 3. The initialize() method handles all setup sequentially (MoveIt, collision scene, constraints).
    if (!trajectory_planner_->initialize()) {
        RCLCPP_ERROR(get_logger(), "❌ Failed to initialize TrajectoryPlanner. Shutting down.");
        rclcpp::shutdown();
        return;
    }
    
    // 4. Now that initialization is complete, it's safe to execute the demo.
    executeDemo();
  }
  
  /**
   * @brief Executes the main pick and place demonstration sequence.
   */
  void executeDemo()
  {
    RCLCPP_INFO(get_logger(), "🚀 Starting production-safe pick and place demo...");
    
    if (demo_config_.smooth_motion) {
      trajectory_planner_->setSmoothMotion(true);
    }
    
    bool success = trajectory_planner_->executeProfessionalPickAndPlace(
      demo_config_.pick_x_mm, demo_config_.pick_y_mm, demo_config_.pick_z_mm,
      demo_config_.pick_roll_deg, demo_config_.pick_pitch_deg, demo_config_.pick_yaw_deg,
      demo_config_.place_x_mm, demo_config_.place_y_mm, demo_config_.place_z_mm,
      demo_config_.place_roll_deg, demo_config_.place_pitch_deg, demo_config_.place_yaw_deg,
      demo_config_.clearance_height_mm
    );
    
    if (success) {
      RCLCPP_INFO(get_logger(), "🎉 Demo completed successfully!");
    } else {
      RCLCPP_ERROR(get_logger(), "❌ Demo failed.");
    }
    
    // Shutdown the node after the demo is complete
    rclcpp::shutdown();
  }
  
  // --- Configuration Structs ---
  trajectory_plan::PlannerConfiguration config_;
  struct DemoConfiguration {
    double pick_x_mm, pick_y_mm, pick_z_mm;
    double pick_roll_deg, pick_pitch_deg, pick_yaw_deg;
    double place_x_mm, place_y_mm, place_z_mm;
    double place_roll_deg, place_pitch_deg, place_yaw_deg;
    double clearance_height_mm;
    bool smooth_motion;
  } demo_config_;
  
  // --- Components ---
  std::unique_ptr<trajectory_plan::TrajectoryPlanner> trajectory_planner_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickAndPlaceNode>();
  // A multi-threaded executor is crucial for nodes that handle complex, parallel tasks like MoveIt.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  // rclcpp::shutdown() is called within the node logic after the demo completes.
  return 0;
}
