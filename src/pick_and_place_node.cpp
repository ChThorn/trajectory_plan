#include <rclcpp/rclcpp.hpp>
#include "trajectory_plan/trajectory_planner.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <signal.h>
#include <atomic>

class PickAndPlaceNode : public rclcpp::Node
{
public:
  PickAndPlaceNode() : Node("pick_and_place_node"), shutdown_requested_(false)
  {
    RCLCPP_INFO(get_logger(), "🚀 Pick and Place Node starting...");
    
    loadConfiguration();
    printConfiguration();
    
    // --- [ENHANCED] ROBUST SHUTDOWN HANDLER ---
    // Create an enhanced shutdown callback with timeout
    auto on_shutdown_callback = [this]() {
        RCLCPP_INFO(get_logger(), "🔌 Shutdown signal received. Stopping robot motion...");
        shutdown_requested_.store(true);
        
        if (trajectory_planner_) {
            // Create a separate thread for shutdown operations with timeout
            std::thread shutdown_thread([this]() {
                try {
                    trajectory_planner_->stop();
                    RCLCPP_INFO(get_logger(), "✅ Robot motion stopped successfully.");
                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(), "⚠️ Exception during shutdown: %s", e.what());
                }
            });
            
            // Give shutdown operations 2 seconds, then force exit
            if (shutdown_thread.joinable()) {
                auto future = std::async(std::launch::async, [&shutdown_thread]() {
                    shutdown_thread.join();
                });
                
                if (future.wait_for(std::chrono::seconds(2)) == std::future_status::timeout) {
                    RCLCPP_WARN(get_logger(), "⏰ Shutdown timeout reached. Force terminating...");
                    shutdown_thread.detach(); // Detach instead of waiting
                }
            }
        }
        
        // Force shutdown after timeout
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::exit(0); // Force exit if normal shutdown fails
    };
    rclcpp::on_shutdown(on_shutdown_callback);

    timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&PickAndPlaceNode::main_task, this));
  }

private:
  std::atomic<bool> shutdown_requested_;
  
  void loadConfiguration()
  {
    this->declare_parameter<double>("table.length", 0.98);
    this->declare_parameter<double>("table.width", 0.49);
    this->declare_parameter<double>("table.height", 0.04);
    this->declare_parameter<double>("table.x_offset", 0.45);
    this->declare_parameter<double>("table.y_offset", 0.0);
    this->declare_parameter<double>("table.z_position", -0.024);

    this->declare_parameter<double>("workspace.width", 1.0);
    this->declare_parameter<double>("workspace.depth", 1.0);
    this->declare_parameter<double>("workspace.height", 1.0);
    this->declare_parameter<double>("workspace.x_position", 0.25);
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

    // Validate after loading
    if (!validateConfiguration()) {
        RCLCPP_ERROR(get_logger(), "❌ Configuration validation failed. Check parameters.");
        rclcpp::shutdown();
        return;
    }
  }
  
  void printConfiguration()
  {
    RCLCPP_INFO(get_logger(), "📋 Configuration loaded:");
    RCLCPP_INFO(get_logger(), "   Table: %.2f x %.2f x %.2f m at [%.2f, %.2f, %.2f]", 
                config_.table.length, config_.table.width, config_.table.height,
                config_.table.x_offset, config_.table.y_offset, config_.table.z_position);
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

  void main_task()
  {
    // Cancel timer first, but check if it exists
    if (timer_) {
        timer_->cancel();
        timer_.reset(); // Explicitly reset to prevent reuse
    }
    
    // Early return if shutdown requested
    if (shutdown_requested_.load() || !rclcpp::ok()) {
        RCLCPP_WARN(get_logger(), "Shutdown requested before task execution");
        return;
    }
    
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
    
    try {
        trajectory_planner_ = std::make_unique<trajectory_plan::TrajectoryPlanner>(shared_from_this());
        trajectory_planner_->updateConfiguration(config_);

        if (!trajectory_planner_->initialize()) {
            RCLCPP_ERROR(get_logger(), "❌ Failed to initialize TrajectoryPlanner. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        setupRobotSafety();
        
        executeDemo();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "❌ Exception in main_task: %s", e.what());
        rclcpp::shutdown();
    }
}

bool validateConfiguration()
{
    bool valid = true;
    
    // Validate table dimensions
    if (config_.table.length <= 0 || config_.table.width <= 0 || config_.table.height <= 0) {
        RCLCPP_ERROR(get_logger(), "❌ Invalid table dimensions");
        valid = false;
    }
    
    // Validate workspace dimensions
    if (config_.workspace.width <= 0 || config_.workspace.depth <= 0 || config_.workspace.height <= 0) {
        RCLCPP_ERROR(get_logger(), "❌ Invalid workspace dimensions");
        valid = false;
    }
    
    // Validate robot scaling factors
    if (config_.robot.velocity_scaling < 0.01 || config_.robot.velocity_scaling > 1.0) {
        RCLCPP_ERROR(get_logger(), "❌ Invalid velocity scaling: %.2f (must be 0.01-1.0)", 
                    config_.robot.velocity_scaling);
        valid = false;
    }
    
    if (config_.robot.acceleration_scaling < 0.01 || config_.robot.acceleration_scaling > 1.0) {
        RCLCPP_ERROR(get_logger(), "❌ Invalid acceleration scaling: %.2f (must be 0.01-1.0)", 
                    config_.robot.acceleration_scaling);
        valid = false;
    }
    
    // Validate demo positions are within workspace
    auto pick_pose = geometry_msgs::msg::Pose();
    pick_pose.position.x = demo_config_.pick_x_mm / 1000.0;
    pick_pose.position.y = demo_config_.pick_y_mm / 1000.0;
    pick_pose.position.z = demo_config_.pick_z_mm / 1000.0;
    
    auto place_pose = geometry_msgs::msg::Pose();
    place_pose.position.x = demo_config_.place_x_mm / 1000.0;
    place_pose.position.y = demo_config_.place_y_mm / 1000.0;
    place_pose.position.z = demo_config_.place_z_mm / 1000.0;
    
    // Basic workspace bounds check (simplified)
    double x_min = config_.workspace.x_position - config_.workspace.width / 2.0;
    double x_max = config_.workspace.x_position + config_.workspace.width / 2.0;
    double y_min = config_.workspace.y_position - config_.workspace.depth / 2.0;
    double y_max = config_.workspace.y_position + config_.workspace.depth / 2.0;
    double z_min = config_.workspace.z_position - config_.workspace.height / 2.0;
    double z_max = config_.workspace.z_position + config_.workspace.height / 2.0;
    
    if (pick_pose.position.x < x_min || pick_pose.position.x > x_max ||
        pick_pose.position.y < y_min || pick_pose.position.y > y_max ||
        pick_pose.position.z < z_min || pick_pose.position.z > z_max) {
        RCLCPP_ERROR(get_logger(), "❌ Pick position [%.1f, %.1f, %.1f] mm outside workspace", 
                    demo_config_.pick_x_mm, demo_config_.pick_y_mm, demo_config_.pick_z_mm);
        valid = false;
    }
    
    if (place_pose.position.x < x_min || place_pose.position.x > x_max ||
        place_pose.position.y < y_min || place_pose.position.y > y_max ||
        place_pose.position.z < z_min || place_pose.position.z > z_max) {
        RCLCPP_ERROR(get_logger(), "❌ Place position [%.1f, %.1f, %.1f] mm outside workspace", 
                    demo_config_.place_x_mm, demo_config_.place_y_mm, demo_config_.place_z_mm);
        valid = false;
    }
    
    if (valid) {
        RCLCPP_INFO(get_logger(), "✅ Configuration validation passed");
    }
    
    return valid;
}
  
  void executeDemo()
  {
    // Check for shutdown before starting
    if (shutdown_requested_.load() || !rclcpp::ok()) {
        RCLCPP_WARN(get_logger(), "Shutdown requested before demo execution");
        return;
    }
    
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
    
    // Check for shutdown again after operation
    if (shutdown_requested_.load() || !rclcpp::ok()) {
        RCLCPP_WARN(get_logger(), "Execution was cancelled by shutdown request.");
        return;
    }

    if (success) {
      RCLCPP_INFO(get_logger(), "🎉 Demo completed successfully!");
    } else {
      RCLCPP_ERROR(get_logger(), "❌ Demo failed.");
    }
    
    // Graceful shutdown
    if (rclcpp::ok()) {
        RCLCPP_INFO(get_logger(), "🏁 Demo finished. Initiating clean shutdown...");
        rclcpp::shutdown();
    }
  }
  
  trajectory_plan::PlannerConfiguration config_;
  struct DemoConfiguration {
    double pick_x_mm, pick_y_mm, pick_z_mm;
    double pick_roll_deg, pick_pitch_deg, pick_yaw_deg;
    double place_x_mm, place_y_mm, place_z_mm;
    double place_roll_deg, place_pitch_deg, place_yaw_deg;
    double clearance_height_mm;
    bool smooth_motion;
  } demo_config_;
  
  std::unique_ptr<trajectory_plan::TrajectoryPlanner> trajectory_planner_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  void setupRobotSafety()
  {
    std::vector<double> emergency_joints = {0.0, -0.2, -0.6, 0.0, 0.8, 0.0};
    
    if (trajectory_planner_ && trajectory_planner_->getRobotOperation()) {
        trajectory_planner_->getRobotOperation()->setEmergencySafePosition(emergency_joints);
        RCLCPP_INFO(get_logger(), "🚨 Emergency safe position configured");
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Create node with error handling
  try {
    auto node = std::make_shared<PickAndPlaceNode>();
    
    // Use SingleThreadedExecutor for better shutdown control
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    // Spin with timeout to allow periodic shutdown checks
    while (rclcpp::ok()) {
        executor.spin_some(std::chrono::milliseconds(100));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
  }
  
  RCLCPP_INFO(rclcpp::get_logger("main"), "🏁 Application terminated cleanly.");
  return 0;
}