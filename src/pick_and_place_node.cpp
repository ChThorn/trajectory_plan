#include <rclcpp/rclcpp.hpp>
#include "trajectory_plan/trajectory_planner.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <signal.h>
#include <atomic>
#include <mutex>
#include <future>

class PickAndPlaceNode : public rclcpp::Node
{
public:
  PickAndPlaceNode() : Node("pick_and_place_node")
  {
    RCLCPP_INFO(get_logger(), "🚀 Pick and Place Node starting...");
    
    // Initialize thread-safe shutdown handling
    setupShutdownHandler();
    
    loadConfiguration();
    printConfiguration();
    
    timer_ = create_wall_timer(
      std::chrono::seconds(3),
      std::bind(&PickAndPlaceNode::main_task, this));
  }
  
  ~PickAndPlaceNode()
  {
    safeShutdown();
  }

private:
  // --- Thread Safety ---
  std::atomic<bool> shutdown_requested_{false};
  std::atomic<bool> shutdown_in_progress_{false};
  std::mutex shutdown_mutex_;
  std::mutex trajectory_planner_mutex_;
  
  // --- Configuration ---
  trajectory_plan::PlannerConfiguration config_;
  trajectory_plan::SafetyConfiguration safety_config_;
  
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
  
  void setupShutdownHandler()
  {
    // Enhanced shutdown callback with proper thread safety
    auto on_shutdown_callback = [this]() {
        RCLCPP_INFO(get_logger(), "🔌 Shutdown signal received. Initiating safe shutdown...");
        this->safeShutdown();
    };
    rclcpp::on_shutdown(on_shutdown_callback);
  }
  
  void safeShutdown()
  {
    // Thread-safe shutdown with timeout
    std::lock_guard<std::mutex> lock(shutdown_mutex_);
    
    if (shutdown_in_progress_.exchange(true)) {
        return; // Already shutting down
    }
    
    shutdown_requested_.store(true);
    
    // Cancel timer first
    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }
    
    // Shutdown trajectory planner with timeout
    std::future<void> shutdown_future = std::async(std::launch::async, [this]() {
        std::lock_guard<std::mutex> tp_lock(trajectory_planner_mutex_);
        if (trajectory_planner_) {
            try {
                trajectory_planner_->stop();
                RCLCPP_INFO(get_logger(), "✅ Robot motion stopped successfully.");
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "⚠️ Exception during shutdown: %s", e.what());
            }
        }
    });
    
    // Wait for shutdown with timeout
    if (shutdown_future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
        RCLCPP_WARN(get_logger(), "⏰ Shutdown timeout reached. Force terminating...");
    }
    
    RCLCPP_INFO(get_logger(), "🏁 Safe shutdown completed.");
  }
  
  void loadConfiguration()
  {
    // --- Table Configuration ---
    this->declare_parameter<double>("table.length", 0.98);
    this->declare_parameter<double>("table.width", 0.49);
    this->declare_parameter<double>("table.height", 0.04);
    this->declare_parameter<double>("table.x_offset", 0.43);
    this->declare_parameter<double>("table.y_offset", 0.0);
    this->declare_parameter<double>("table.z_position", -0.023);

    // --- Workspace Configuration ---
    this->declare_parameter<double>("workspace.width", 1.0);
    this->declare_parameter<double>("workspace.depth", 1.0);
    this->declare_parameter<double>("workspace.height", 1.0);
    this->declare_parameter<double>("workspace.x_position", 0.25);
    this->declare_parameter<double>("workspace.y_position", 0.0);
    this->declare_parameter<double>("workspace.z_position", 0.5);
    this->declare_parameter<bool>("workspace.visualization_enabled", true);
    
    // --- Robot Configuration (Enhanced) ---
    this->declare_parameter<double>("robot.velocity_scaling", 0.3);
    this->declare_parameter<double>("robot.acceleration_scaling", 0.3);
    this->declare_parameter<double>("robot.planning_time", 5.0);
    this->declare_parameter<int>("robot.planning_attempts", 10);
    this->declare_parameter<double>("robot.cartesian_step_size", 0.01);
    this->declare_parameter<double>("robot.cartesian_jump_threshold", 0.0);
    this->declare_parameter<bool>("robot.smooth_motion", true);
    
    // --- Safety Configuration (NEW) ---
    this->declare_parameter<std::vector<double>>("safety.home_joints", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("safety.safe_joints", {0.0, -0.5, -1.0, 0.0, 1.5, 0.0});
    this->declare_parameter<std::vector<double>>("safety.emergency_joints", {0.0, -0.3, -0.8, 0.0, 1.1, 0.0});
    this->declare_parameter<double>("safety.max_joint_velocity", 1.0);
    this->declare_parameter<double>("safety.max_joint_acceleration", 2.0);
    this->declare_parameter<double>("safety.safety_stop_timeout", 2.0);
    this->declare_parameter<bool>("safety.validate_joint_limits", true);
    this->declare_parameter<bool>("safety.validate_reachability", true);
    this->declare_parameter<bool>("safety.require_manual_recovery", false);
    
    // --- Demo Configuration ---
    this->declare_parameter<double>("demo.pick_x_mm", 250.0);
    this->declare_parameter<double>("demo.pick_y_mm", 150.0);
    this->declare_parameter<double>("demo.pick_z_mm", 50.0);
    this->declare_parameter<double>("demo.pick_roll_deg", 0.0);
    this->declare_parameter<double>("demo.pick_pitch_deg", 90.0);
    this->declare_parameter<double>("demo.pick_yaw_deg", 12.0);
    
    this->declare_parameter<double>("demo.place_x_mm", 350.0);
    this->declare_parameter<double>("demo.place_y_mm", -150.0);
    this->declare_parameter<double>("demo.place_z_mm", 50.0);
    this->declare_parameter<double>("demo.place_roll_deg", 0.0);
    this->declare_parameter<double>("demo.place_pitch_deg", 90.0);
    this->declare_parameter<double>("demo.place_yaw_deg", 13.0);
    
    this->declare_parameter<double>("demo.clearance_height_mm", 50.0);
    
    // Load parameters into structures
    loadTableConfig();
    loadWorkspaceConfig();
    loadRobotConfig();
    loadSafetyConfig();
    loadDemoConfig();

    // Validate after loading
    if (!validateConfiguration()) {
        RCLCPP_ERROR(get_logger(), "❌ Configuration validation failed. Check parameters.");
        throw std::runtime_error("Invalid configuration");
    }
  }
  
  void loadTableConfig()
  {
    config_.table.length = this->get_parameter("table.length").as_double();
    config_.table.width = this->get_parameter("table.width").as_double();
    config_.table.height = this->get_parameter("table.height").as_double();
    config_.table.x_offset = this->get_parameter("table.x_offset").as_double();
    config_.table.y_offset = this->get_parameter("table.y_offset").as_double();
    config_.table.z_position = this->get_parameter("table.z_position").as_double();
  }
  
  void loadWorkspaceConfig()
  {
    config_.workspace.width = this->get_parameter("workspace.width").as_double();
    config_.workspace.depth = this->get_parameter("workspace.depth").as_double();
    config_.workspace.height = this->get_parameter("workspace.height").as_double();
    config_.workspace.x_position = this->get_parameter("workspace.x_position").as_double();
    config_.workspace.y_position = this->get_parameter("workspace.y_position").as_double();
    config_.workspace.z_position = this->get_parameter("workspace.z_position").as_double();
    config_.workspace.visualization_enabled = this->get_parameter("workspace.visualization_enabled").as_bool();
  }
  
  void loadRobotConfig()
  {
    config_.robot.velocity_scaling = this->get_parameter("robot.velocity_scaling").as_double();
    config_.robot.acceleration_scaling = this->get_parameter("robot.acceleration_scaling").as_double();
    config_.robot.planning_time = this->get_parameter("robot.planning_time").as_double();
    config_.robot.planning_attempts = this->get_parameter("robot.planning_attempts").as_int();
    config_.robot.cartesian_step_size = this->get_parameter("robot.cartesian_step_size").as_double();
    config_.robot.cartesian_jump_threshold = this->get_parameter("robot.cartesian_jump_threshold").as_double();
  }
  
  void loadSafetyConfig()
  {
    safety_config_.home_joints = this->get_parameter("safety.home_joints").as_double_array();
    safety_config_.safe_joints = this->get_parameter("safety.safe_joints").as_double_array();
    safety_config_.emergency_joints = this->get_parameter("safety.emergency_joints").as_double_array();
    safety_config_.max_joint_velocity = this->get_parameter("safety.max_joint_velocity").as_double();
    safety_config_.max_joint_acceleration = this->get_parameter("safety.max_joint_acceleration").as_double();
    safety_config_.safety_stop_timeout = this->get_parameter("safety.safety_stop_timeout").as_double();
    safety_config_.planning_timeout = config_.robot.planning_time;
    safety_config_.max_planning_attempts = config_.robot.planning_attempts;
    safety_config_.cartesian_step_size = config_.robot.cartesian_step_size;
    safety_config_.cartesian_jump_threshold = config_.robot.cartesian_jump_threshold;
    safety_config_.validate_joint_limits = this->get_parameter("safety.validate_joint_limits").as_bool();
    safety_config_.validate_reachability = this->get_parameter("safety.validate_reachability").as_bool();
    safety_config_.require_manual_recovery = this->get_parameter("safety.require_manual_recovery").as_bool();
  }
  
  void loadDemoConfig()
  {
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
  
  void printConfiguration()
  {
    RCLCPP_INFO(get_logger(), "📋 Configuration loaded:");
    RCLCPP_INFO(get_logger(), "   Table: %.2f x %.2f x %.2f m at [%.2f, %.2f, %.2f]", 
                config_.table.length, config_.table.width, config_.table.height,
                config_.table.x_offset, config_.table.y_offset, config_.table.z_position);
    RCLCPP_INFO(get_logger(), "   Workspace: %.2f x %.2f x %.2f m at [%.2f, %.2f, %.2f]", 
                config_.workspace.width, config_.workspace.depth, config_.workspace.height,
                config_.workspace.x_position, config_.workspace.y_position, config_.workspace.z_position);
    RCLCPP_INFO(get_logger(), "   Robot: vel=%.2f, acc=%.2f, planning_time=%.1fs, attempts=%d", 
                config_.robot.velocity_scaling, config_.robot.acceleration_scaling,
                config_.robot.planning_time, config_.robot.planning_attempts);
    RCLCPP_INFO(get_logger(), "   Safety: validate_limits=%s, validate_reach=%s, timeout=%.1fs",
                safety_config_.validate_joint_limits ? "true" : "false",
                safety_config_.validate_reachability ? "true" : "false",
                safety_config_.safety_stop_timeout);
    RCLCPP_INFO(get_logger(), "   Demo: pick=[%.1f, %.1f, %.1f] mm, place=[%.1f, %.1f, %.1f] mm", 
                demo_config_.pick_x_mm, demo_config_.pick_y_mm, demo_config_.pick_z_mm,
                demo_config_.place_x_mm, demo_config_.place_y_mm, demo_config_.place_z_mm);
  }

  void main_task()
  {
    // Cancel timer first to prevent re-entry
    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }
    
    // Early return if shutdown requested
    if (shutdown_requested_.load() || !rclcpp::ok()) {
        RCLCPP_WARN(get_logger(), "Shutdown requested before task execution");
        return;
    }
    
    try {
        // Check controller availability
        if (!waitForController()) {
            return;
        }
        
        // Initialize trajectory planner with thread safety
        if (!initializeTrajectoryPlanner()) {
            return;
        }
        
        // Execute demo
        executeDemo();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "❌ Exception in main_task: %s", e.what());
        safeShutdown();
    }
  }
  
  bool waitForController()
  {
    RCLCPP_INFO(get_logger(), "🔌 Checking for controller action server...");
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
        this, "joint_trajectory_controller/follow_joint_trajectory");
        
    if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(get_logger(), "❌ Controller action server not available after 10s. Shutting down.");
        rclcpp::shutdown();
        return false;
    }
    RCLCPP_INFO(get_logger(), "✅ Controller is ready.");
    return true;
  }
  
  bool initializeTrajectoryPlanner()
  {
    std::lock_guard<std::mutex> lock(trajectory_planner_mutex_);
    
    try {
        trajectory_planner_ = std::make_unique<trajectory_plan::TrajectoryPlanner>(shared_from_this());
        trajectory_planner_->updateConfiguration(config_);

        if (!trajectory_planner_->initialize()) {
            RCLCPP_ERROR(get_logger(), "❌ Failed to initialize TrajectoryPlanner. Shutting down.");
            rclcpp::shutdown();
            return false;
        }

        // Configure safety settings
        if (trajectory_planner_->getRobotOperation()) {
            trajectory_planner_->getRobotOperation()->updateSafetyConfiguration(safety_config_);
            RCLCPP_INFO(get_logger(), "🚨 Safety configuration applied");
        }
        
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "❌ Exception initializing trajectory planner: %s", e.what());
        return false;
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
    
    // Validate planning parameters
    if (config_.robot.planning_time <= 0 || config_.robot.planning_time > 30.0) {
        RCLCPP_ERROR(get_logger(), "❌ Invalid planning time: %.1f (must be 0-30s)", 
                    config_.robot.planning_time);
        valid = false;
    }
    
    if (config_.robot.planning_attempts < 1 || config_.robot.planning_attempts > 50) {
        RCLCPP_ERROR(get_logger(), "❌ Invalid planning attempts: %d (must be 1-50)", 
                    config_.robot.planning_attempts);
        valid = false;
    }
    
    // Validate safety configuration
    if (safety_config_.home_joints.size() != 6 || 
        safety_config_.safe_joints.size() != 6 || 
        safety_config_.emergency_joints.size() != 6) {
        RCLCPP_ERROR(get_logger(), "❌ Safety joint positions must have exactly 6 values");
        valid = false;
    }
    
    if (safety_config_.safety_stop_timeout <= 0 || safety_config_.safety_stop_timeout > 10.0) {
        RCLCPP_ERROR(get_logger(), "❌ Invalid safety stop timeout: %.1f (must be 0-10s)", 
                    safety_config_.safety_stop_timeout);
        valid = false;
    }
    
    // Validate demo positions are within workspace
    if (!validateDemoPositions()) {
        valid = false;
    }
    
    if (valid) {
        RCLCPP_INFO(get_logger(), "✅ Configuration validation passed");
    }
    
    return valid;
  }
  
  bool validateDemoPositions()
  {
    auto pick_pose = geometry_msgs::msg::Pose();
    pick_pose.position.x = demo_config_.pick_x_mm / 1000.0;
    pick_pose.position.y = demo_config_.pick_y_mm / 1000.0;
    pick_pose.position.z = demo_config_.pick_z_mm / 1000.0;
    
    auto place_pose = geometry_msgs::msg::Pose();
    place_pose.position.x = demo_config_.place_x_mm / 1000.0;
    place_pose.position.y = demo_config_.place_y_mm / 1000.0;
    place_pose.position.z = demo_config_.place_z_mm / 1000.0;
    
    // Basic workspace bounds check
    double x_min = config_.workspace.x_position - config_.workspace.width / 2.0;
    double x_max = config_.workspace.x_position + config_.workspace.width / 2.0;
    double y_min = config_.workspace.y_position - config_.workspace.depth / 2.0;
    double y_max = config_.workspace.y_position + config_.workspace.depth / 2.0;
    double z_min = config_.workspace.z_position - config_.workspace.height / 2.0;
    double z_max = config_.workspace.z_position + config_.workspace.height / 2.0;
    
    bool pick_valid = (pick_pose.position.x >= x_min && pick_pose.position.x <= x_max &&
                       pick_pose.position.y >= y_min && pick_pose.position.y <= y_max &&
                       pick_pose.position.z >= z_min && pick_pose.position.z <= z_max);
    
    bool place_valid = (place_pose.position.x >= x_min && place_pose.position.x <= x_max &&
                        place_pose.position.y >= y_min && place_pose.position.y <= y_max &&
                        place_pose.position.z >= z_min && place_pose.position.z <= z_max);
    
    if (!pick_valid) {
        RCLCPP_ERROR(get_logger(), "❌ Pick position [%.1f, %.1f, %.1f] mm outside workspace", 
                    demo_config_.pick_x_mm, demo_config_.pick_y_mm, demo_config_.pick_z_mm);
    }
    
    if (!place_valid) {
        RCLCPP_ERROR(get_logger(), "❌ Place position [%.1f, %.1f, %.1f] mm outside workspace", 
                    demo_config_.place_x_mm, demo_config_.place_y_mm, demo_config_.place_z_mm);
    }
    
    return pick_valid && place_valid;
  }
  
  void executeDemo()
  {
    // Check for shutdown before starting
    if (shutdown_requested_.load() || !rclcpp::ok()) {
        RCLCPP_WARN(get_logger(), "Shutdown requested before demo execution");
        return;
    }
    
    RCLCPP_INFO(get_logger(), "🚀 Starting production-safe pick and place demo...");
    
    try {
        std::lock_guard<std::mutex> lock(trajectory_planner_mutex_);
        
        if (!trajectory_planner_) {
            RCLCPP_ERROR(get_logger(), "❌ Trajectory planner not available");
            return;
        }
        
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
            
            // Print performance metrics
            if (trajectory_planner_->getRobotOperation()) {
                auto metrics = trajectory_planner_->getRobotOperation()->getLastOperationMetrics();
                RCLCPP_INFO(get_logger(), "📊 Performance: Planning=%.2fs, Execution=%.2fs, Recovery=%zu attempts",
                           metrics.planning_time.count(), metrics.execution_time.count(), metrics.recovery_attempts);
            }
        } else {
            RCLCPP_ERROR(get_logger(), "❌ Demo failed.");
        }
        
    } catch (const trajectory_plan::RobotOperationError& e) {
        if (e.getType() == trajectory_plan::RobotOperationError::Type::SHUTDOWN_REQUESTED) {
            RCLCPP_WARN(get_logger(), "🔌 Demo interrupted by shutdown request");
        } else {
            RCLCPP_ERROR(get_logger(), "❌ Robot operation error: %s", e.what());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "❌ Unexpected error in demo: %s", e.what());
    }
    
    // Graceful shutdown
    if (rclcpp::ok()) {
        RCLCPP_INFO(get_logger(), "🏁 Demo finished. Initiating clean shutdown...");
        rclcpp::shutdown();
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
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
    return 1;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("main"), "🏁 Application terminated cleanly.");
  return 0;
}