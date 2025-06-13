#include "trajectory_plan/trajectory_planner.hpp"
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <thread> 

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>  // ADD THIS LINE

namespace trajectory_plan
{

TrajectoryPlanner::TrajectoryPlanner(const rclcpp::Node::SharedPtr& node)
    : node_(node), initialized_(false)
{
    RCLCPP_INFO(node_->get_logger(), "🚀 TrajectoryPlanner created. Call initialize() to setup.");
    config_ = PlannerConfiguration{};
    operational_.store(false);
}

bool TrajectoryPlanner::initialize()
{
    if (initialized_) {
        RCLCPP_WARN(node_->get_logger(), "TrajectoryPlanner already initialized.");
        return true;
    }
    
    RCLCPP_INFO(node_->get_logger(), "🔧 Starting SYNCHRONOUS TrajectoryPlanner initialization...");

    try {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "mainpulation");
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        RCLCPP_INFO(node_->get_logger(), "✅ [1/4] MoveIt interfaces initialized.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❌ [1/4] Failed to initialize MoveIt: %s", e.what());
        return false;
    }

    // Configure collision checking parameters
    if (move_group_) {
        move_group_->setGoalPositionTolerance(0.01);      // 1cm position tolerance
        move_group_->setGoalOrientationTolerance(0.1);    // ~6 degree orientation tolerance
        move_group_->setMaxVelocityScalingFactor(config_.robot.velocity_scaling);
        move_group_->setMaxAccelerationScalingFactor(config_.robot.acceleration_scaling);
        
        RCLCPP_INFO(node_->get_logger(), "🛡️ Enhanced collision checking configured");
        RCLCPP_INFO(node_->get_logger(), "   Planning group: %s", move_group_->getName().c_str());
        RCLCPP_INFO(node_->get_logger(), "   End-effector: %s", move_group_->getEndEffectorLink().c_str());
        RCLCPP_INFO(node_->get_logger(), "   Position tolerance: 1cm, Orientation tolerance: ~6°");
    }

    try {
        robot_operation_ = std::make_unique<RobotOperation>(node_, move_group_);
        robot_operation_->setVelocityScaling(config_.robot.velocity_scaling);
        robot_operation_->setAccelerationScaling(config_.robot.acceleration_scaling);
        RCLCPP_INFO(node_->get_logger(), "✅ [2/4] RobotOperation initialized.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❌ [2/4] Failed to initialize RobotOperation: %s", e.what());
        return false;
    }

    if (!setupCollisionScene()) {
        RCLCPP_ERROR(node_->get_logger(), "❌ [3/4] Failed to setup collision scene.");
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "✅ [3/4] Collision scene configured.");

    if (!addWorkspaceConstraints()) {
        RCLCPP_ERROR(node_->get_logger(), "❌ [4/4] Failed to add workspace constraints.");
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "✅ [4/4] Workspace constraints applied.");
    
    // FIXED: Enable visualization AFTER full initialization to prevent hanging
    initialized_ = true;
    operational_.store(true);
    RCLCPP_INFO(node_->get_logger(), "✅ TrajectoryPlanner initialized successfully.");
    
    // Now safely enable visualization
    if (config_.workspace.visualization_enabled) {
        enableWorkspaceVisualization(true);
    }

    return true;
}

// --- [FIXED] MISSING SAFE SHUTDOWN IMPLEMENTATION ---
void TrajectoryPlanner::safeShutdown()
{
    std::lock_guard<std::mutex> lock(operation_mutex_);
    
    if (shutdown_requested_.exchange(true)) {
        return; // Already shutting down
    }
    
    operational_.store(false);
    
    try {
        // Stop any ongoing robot motion
        if (move_group_) {
            move_group_->stop();
            RCLCPP_INFO(node_->get_logger(), "🛑 Robot motion stopped");
        }
        
        // Stop robot operation
        if (robot_operation_) {
            robot_operation_->emergencyStop();
            RCLCPP_INFO(node_->get_logger(), "🛑 Robot operation stopped");
        }
        
        // Cancel visualization timer
        if (visualization_timer_) {
            visualization_timer_->cancel();
            visualization_timer_.reset();
            RCLCPP_DEBUG(node_->get_logger(), "🔲 Visualization timer cancelled");
        }
        
        // Reset publisher
        if (marker_publisher_) {
            marker_publisher_.reset();
            RCLCPP_DEBUG(node_->get_logger(), "🔲 Marker publisher reset");
        }
        
        RCLCPP_INFO(node_->get_logger(), "✅ TrajectoryPlanner safe shutdown completed");
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Exception during TrajectoryPlanner shutdown: %s", e.what());
    }
}

void TrajectoryPlanner::stop()
{
    if (!initialized_ || !move_group_) {
        RCLCPP_WARN(node_->get_logger(), "Cannot stop, planner not initialized.");
        return;
    }
    RCLCPP_INFO(node_->get_logger(), "🛑 Sending stop signal to MoveGroup to cancel all actions.");
    move_group_->stop(); // This is the crucial call to MoveIt
}

void TrajectoryPlanner::updateConfiguration(const PlannerConfiguration& config)
{
    std::lock_guard<std::mutex> lock(operation_mutex_);
    
    config_ = config;
    invalidateWorkspaceCache(); // Clear cache when config changes
    
    if (robot_operation_) {
        robot_operation_->setVelocityScaling(config_.robot.velocity_scaling);
        robot_operation_->setAccelerationScaling(config_.robot.acceleration_scaling);
    }
    if(initialized_) {
        setupCollisionScene();
        addWorkspaceConstraints();
    }
    RCLCPP_INFO(node_->get_logger(), "🔧 Configuration updated.");
}

bool TrajectoryPlanner::executeProfessionalPickAndPlace(
    double pick_x_mm, double pick_y_mm, double pick_z_mm,
    double pick_roll_deg, double pick_pitch_deg, double pick_yaw_deg,
    double place_x_mm, double place_y_mm, double place_z_mm,
    double place_roll_deg, double place_pitch_deg, double place_yaw_deg,
    double clearance_height_mm)
{
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "❌ TrajectoryPlanner not initialized.");
        return false;
    }
    
    if (!operational_.load()) {
        RCLCPP_ERROR(node_->get_logger(), "❌ TrajectoryPlanner not operational.");
        return false;
    }
    
    auto pick_pose_for_validation = robot_operation_->createPoseFromMmAndDegrees(pick_x_mm, pick_y_mm, pick_z_mm, 0,0,0);
    auto place_pose_for_validation = robot_operation_->createPoseFromMmAndDegrees(place_x_mm, place_y_mm, place_z_mm, 0,0,0);
    if (!validatePoseInWorkspace(pick_pose_for_validation) || !validatePoseInWorkspace(place_pose_for_validation)) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Aborting: One or more poses are outside the workspace.");
        return false;
    }

    return robot_operation_->executeProfessionalPickAndPlace(
        pick_x_mm, pick_y_mm, pick_z_mm, pick_roll_deg, pick_pitch_deg, pick_yaw_deg,
        place_x_mm, place_y_mm, place_z_mm, place_roll_deg, place_pitch_deg, place_yaw_deg,
        clearance_height_mm
    );
}

bool TrajectoryPlanner::moveToHome() { 
    return operational_.load() && robot_operation_ ? robot_operation_->moveToHome() : false; 
}

bool TrajectoryPlanner::moveToSafePosition() { 
    return operational_.load() && robot_operation_ ? robot_operation_->moveToSafePosition() : false; 
}

void TrajectoryPlanner::setSmoothMotion(bool enable) { 
    if (operational_.load() && robot_operation_) robot_operation_->setSmoothMotion(enable); 
}

void TrajectoryPlanner::printCurrentPose() { 
    if (operational_.load() && robot_operation_) robot_operation_->checkAndPrintCurrentPose(); 
}

// === REPLACE setupCollisionScene() in trajectory_planner.cpp ===
bool TrajectoryPlanner::setupCollisionScene()
{
    RCLCPP_INFO(node_->get_logger(), "🛡️ Setting up ENHANCED collision scene with safety margins...");
    
    moveit_msgs::msg::CollisionObject table_object;
    table_object.header.frame_id = move_group_->getPlanningFrame();
    table_object.id = "table_surface";
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    
    // 🔥 CRITICAL: ADD 5CM SAFETY MARGINS
    const double SAFETY_MARGIN = 0.02; // 5cm safety buffer
    primitive.dimensions = {
        config_.table.length + 2 * SAFETY_MARGIN,   // Expand in X
        config_.table.width + 2 * SAFETY_MARGIN,    // Expand in Y  
        config_.table.height + SAFETY_MARGIN        // Expand upward only
    };
    
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = config_.table.x_offset;
    table_pose.position.y = config_.table.y_offset;
    table_pose.position.z = config_.table.z_position + SAFETY_MARGIN/2; // Adjust for margin
    table_pose.orientation.w = 1.0;
    
    table_object.primitives.push_back(primitive);
    table_object.primitive_poses.push_back(table_pose);
    table_object.operation = table_object.ADD;
    
    bool success = planning_scene_interface_->applyCollisionObjects({table_object});
    
    if (success) {
        RCLCPP_INFO(node_->get_logger(), "✅ Table collision object added with %.1fcm safety margins", SAFETY_MARGIN * 100);
        
        // Add other collision setup methods
        addTCPCollisionGeometry();
        
        // Wait for scene to update
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        printCollisionObjectStatus();
        debugCollisionConfiguration();
    } else {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to add table collision object");
    }
    
    return success;
}


bool TrajectoryPlanner::addWorkspaceConstraints()
{
    RCLCPP_INFO(node_->get_logger(), "📐 Applying workspace constraints to ALL robot links...");
    
    moveit_msgs::msg::Constraints path_constraints;
    const auto& link_names = move_group_->getRobotModel()->getJointModelGroup(move_group_->getName())->getLinkModelNames();

    for (const auto& link_name : link_names) {
        if (link_name == "world") {
            continue;
        }

        moveit_msgs::msg::PositionConstraint pcm;
        pcm.header.frame_id = move_group_->getPlanningFrame();
        pcm.link_name = link_name;
        
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {config_.workspace.width, config_.workspace.depth, config_.workspace.height};
        
        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = config_.workspace.x_position;
        box_pose.position.y = config_.workspace.y_position;
        box_pose.position.z = config_.workspace.z_position;
        box_pose.orientation.w = 1.0;

        pcm.constraint_region.primitives.push_back(box);
        pcm.constraint_region.primitive_poses.push_back(box_pose);
        pcm.weight = 1.0;

        path_constraints.position_constraints.push_back(pcm);
    }
    
    move_group_->setPathConstraints(path_constraints);
    
    RCLCPP_INFO(node_->get_logger(), "   Applied constraints to %ld links.", link_names.size());
    printWorkspaceLimits();
    return true;
}

bool TrajectoryPlanner::validatePoseInWorkspace(const geometry_msgs::msg::Pose& pose)
{
    auto bounds = getWorkspaceBounds();
    
    bool valid = (pose.position.x >= bounds[0] && pose.position.x <= bounds[1] &&
                  pose.position.y >= bounds[2] && pose.position.y <= bounds[3] &&
                  pose.position.z >= bounds[4] && pose.position.z <= bounds[5]);

    if (!valid) {
        RCLCPP_WARN(node_->get_logger(), "❌ Pose [%.3f, %.3f, %.3f] is outside the defined workspace.", 
                     pose.position.x, pose.position.y, pose.position.z);
    } else {
        RCLCPP_DEBUG(node_->get_logger(), "✅ Pose [%.3f, %.3f, %.3f] is within the workspace.", 
                    pose.position.x, pose.position.y, pose.position.z);
    }
    return valid;
}


void TrajectoryPlanner::enableWorkspaceVisualization(bool enable)
{
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    
    if (enable && !marker_publisher_) {
        try {
            marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("workspace_boundary", 10);
            
            // FIXED: Don't publish immediately - let the timer handle it to prevent hanging
            // The first publish will happen when the timer triggers (after 5 seconds)
            visualization_timer_ = node_->create_wall_timer(
                std::chrono::seconds(5), 
                [this](){ this->publishWorkspaceBoundary(); }
            );
            
            RCLCPP_INFO(node_->get_logger(), "🔳 Workspace visualization ENABLED.");
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(), "⚠️ Failed to enable workspace visualization: %s", e.what());
        }
    } else if (!enable && marker_publisher_) {
        if(visualization_timer_) {
            visualization_timer_->cancel();
            visualization_timer_.reset();
        }
        marker_publisher_.reset();
        RCLCPP_INFO(node_->get_logger(), "🔲 Workspace visualization DISABLED.");
    }
}

void TrajectoryPlanner::publishWorkspaceBoundary()
{
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    
    if (!marker_publisher_ || !initialized_ || !operational_.load()) {
        return;
    }

    try {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker box_marker;
        
        // FIXED: Add safety check for move_group_
        if (!move_group_) {
            RCLCPP_WARN(node_->get_logger(), "⚠️ Cannot publish workspace boundary - move_group not ready");
            return;
        }
        
        box_marker.header.frame_id = move_group_->getPlanningFrame();
        box_marker.header.stamp = node_->now();
        box_marker.ns = "workspace_boundary";
        box_marker.id = 0;
        box_marker.type = visualization_msgs::msg::Marker::CUBE;
        box_marker.action = visualization_msgs::msg::Marker::ADD;

        box_marker.lifetime = rclcpp::Duration(0, 0);

        box_marker.pose.position.x = config_.workspace.x_position;
        box_marker.pose.position.y = config_.workspace.y_position;
        box_marker.pose.position.z = config_.workspace.z_position;
        box_marker.pose.orientation.w = 1.0;

        box_marker.scale.x = config_.workspace.width;
        box_marker.scale.y = config_.workspace.depth;
        box_marker.scale.z = config_.workspace.height;

        box_marker.color.r = 1.0f;
        box_marker.color.g = 0.0f;
        box_marker.color.b = 0.0f;
        box_marker.color.a = 0.25f;

        marker_array.markers.push_back(box_marker);
        marker_publisher_->publish(marker_array);
        
        RCLCPP_DEBUG(node_->get_logger(), "🔳 Workspace boundary published");
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Exception publishing workspace boundary: %s", e.what());
    }
}

void TrajectoryPlanner::printWorkspaceLimits()
{
    double x_min = config_.workspace.x_position - config_.workspace.width / 2.0;
    double x_max = config_.workspace.x_position + config_.workspace.width / 2.0;
    double y_min = config_.workspace.y_position - config_.workspace.depth / 2.0;
    double y_max = config_.workspace.y_position + config_.workspace.depth / 2.0;
    double z_min = config_.workspace.z_position - config_.workspace.height / 2.0;
    double z_max = config_.workspace.z_position + config_.workspace.height / 2.0;
    
    RCLCPP_INFO(node_->get_logger(), "📐 Workspace Limits: X[%.3f, %.3f], Y[%.3f, %.3f], Z[%.3f, %.3f]",
                x_min, x_max, y_min, y_max, z_min, z_max);
}

// === ACTION SERVER IMPLEMENTATION ===
// Add these methods to trajectory_planner.cpp

void TrajectoryPlanner::startActionServer()
{
    std::lock_guard<std::mutex> lock(action_mutex_);
    
    if (action_server_) {
        RCLCPP_WARN(node_->get_logger(), "Action server already running");
        return;
    }
    
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Cannot start action server - TrajectoryPlanner not initialized");
        return;
    }
    
    try {
        RCLCPP_INFO(node_->get_logger(), "🔧 Creating action server with debug logging...");
        
        action_server_ = rclcpp_action::create_server<PickAndPlaceAction>(
            node_,
            "pick_and_place",  // Action name
            [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PickAndPlaceAction::Goal> goal) {
                RCLCPP_INFO(node_->get_logger(), "🐛 DEBUG: handleGoal callback triggered!");
                return this->handleGoal(uuid, goal);
            },
            [this](const std::shared_ptr<GoalHandlePickAndPlace> goal_handle) {
                RCLCPP_INFO(node_->get_logger(), "🐛 DEBUG: handleCancel callback triggered!");
                return this->handleCancel(goal_handle);
            },
            [this](const std::shared_ptr<GoalHandlePickAndPlace> goal_handle) {
                RCLCPP_INFO(node_->get_logger(), "🐛 DEBUG: handleAccepted callback triggered!");
                this->handleAccepted(goal_handle);
            }
        );
        
        if (action_server_) {
            RCLCPP_INFO(node_->get_logger(), "🚀 Pick and Place Action Server created successfully");
            // RCLCPP_INFO(node_->get_logger(), "🔧 Action server pointer: %p", action_server_.get());
            RCLCPP_INFO(node_->get_logger(), "🔧 Action server pointer: %p", static_cast<void*>(action_server_.get()));
        } else {
            RCLCPP_ERROR(node_->get_logger(), "❌ Action server creation returned null pointer!");
            return;
        }
        
        // Small delay to ensure topics are advertised
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        RCLCPP_INFO(node_->get_logger(), "🚀 Pick and Place Action Server started on 'pick_and_place'");
        RCLCPP_INFO(node_->get_logger(), "🔧 Node name: %s", node_->get_name());
        RCLCPP_INFO(node_->get_logger(), "🔧 Node namespace: %s", node_->get_namespace());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to start action server: %s", e.what());
        action_server_.reset();
    }
}

void TrajectoryPlanner::stopActionServer()
{
    std::lock_guard<std::mutex> lock(action_mutex_);
    
    if (action_server_) {
        // Cancel any ongoing action
        if (current_goal_handle_ && action_in_progress_.load()) {
            auto result = std::make_shared<PickAndPlaceAction::Result>();
            result->success = false;
            result->message = "Action server shutting down";
            result->failure_reason = "Server shutdown requested";
            
            current_goal_handle_->abort(result);
            current_goal_handle_.reset();
        }
        
        action_server_.reset();
        action_in_progress_.store(false);
        RCLCPP_INFO(node_->get_logger(), "🛑 Pick and Place Action Server stopped");
    }
}

rclcpp_action::GoalResponse TrajectoryPlanner::handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickAndPlaceAction::Goal> goal)
{
    (void)uuid;  // Suppress unused parameter warning
    
    auto goal_received_time = std::chrono::steady_clock::now();
    
    RCLCPP_INFO(node_->get_logger(), "📨 Received pick and place goal request (ID: %s)", goal->operation_id.c_str());
    RCLCPP_INFO(node_->get_logger(), "📍 Pick: [%.1f, %.1f, %.1f] mm", goal->pick_x_mm, goal->pick_y_mm, goal->pick_z_mm);
    RCLCPP_INFO(node_->get_logger(), "📍 Place: [%.1f, %.1f, %.1f] mm", goal->place_x_mm, goal->place_y_mm, goal->place_z_mm);
    
    // === FAST VALIDATION (minimize response time) ===
    
    // Check if system is operational (immediate check)
    if (!operational_.load()) {
        RCLCPP_WARN(node_->get_logger(), "❌ Rejecting goal - system not operational");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Check if another action is in progress (immediate check)
    if (action_in_progress_.load()) {
        RCLCPP_WARN(node_->get_logger(), "❌ Rejecting goal - another action in progress");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Quick validation (avoid expensive checks in handleGoal)
    if (goal->operation_id.empty()) {
        RCLCPP_WARN(node_->get_logger(), "❌ Rejecting goal - empty operation_id");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // Defer detailed validation to execution phase to speed up response
    auto response_time = std::chrono::steady_clock::now() - goal_received_time;
    RCLCPP_INFO(node_->get_logger(), "✅ Goal accepted (ID: %s) in %.3fs", 
                goal->operation_id.c_str(), std::chrono::duration<double>(response_time).count());
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryPlanner::handleCancel(
    const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
{
    (void)goal_handle;  // Suppress unused parameter warning
    
    RCLCPP_INFO(node_->get_logger(), "📨 Received cancel request for pick and place action");
    
    // Stop robot motion immediately
    if (operational_.load()) {
        stop();  // Emergency stop
    }
    
    action_in_progress_.store(false);
    RCLCPP_INFO(node_->get_logger(), "✅ Pick and place action cancelled successfully");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryPlanner::handleAccepted(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
{
    std::lock_guard<std::mutex> lock(action_mutex_);
    
    current_goal_handle_ = goal_handle;
    action_in_progress_.store(true);
    
    // Execute the action in a separate thread to avoid blocking
    std::thread execution_thread(&TrajectoryPlanner::executePickAndPlaceAction, this, goal_handle);
    execution_thread.detach();
}

void TrajectoryPlanner::executePickAndPlaceAction(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PickAndPlaceAction::Result>();
    
    // Initialize result with goal info
    result->operation_id = goal->operation_id;
    result->success = false;
    
    auto start_time = std::chrono::steady_clock::now();
    
    // IMPROVED: Timeout handling for the entire action
    const auto action_timeout = std::chrono::minutes(5); // Configurable timeout
    
    try {
        RCLCPP_INFO(node_->get_logger(), "🚀 Starting pick and place execution (ID: %s)", goal->operation_id.c_str());
        
        // Check timeout periodically during execution
        auto check_timeout = [&]() -> bool {
            if (std::chrono::steady_clock::now() - start_time > action_timeout) {
                RCLCPP_ERROR(node_->get_logger(), "⏰ Action timeout reached for ID: %s", goal->operation_id.c_str());
                result->message = "Action timeout reached";
                result->failure_reason = "Execution time exceeded maximum allowed duration";
                goal_handle->abort(result);
                return true;
            }
            return false;
        };
        
        // Publish initial feedback
        publishFeedback(goal_handle, "initializing", 0.0, "Starting pick and place sequence");
        if (check_timeout()) return;
        
        // Configure smooth motion if requested
        if (goal->smooth_motion && robot_operation_) {
            robot_operation_->setSmoothMotion(true);
            publishFeedback(goal_handle, "configuring", 5.0, "Smooth motion enabled");
        }
        if (check_timeout()) return;
        
        // Validate poses are in workspace
        publishFeedback(goal_handle, "validating", 10.0, "Validating target poses");
        
        auto pick_pose_for_validation = robot_operation_->createPoseFromMmAndDegrees(
            goal->pick_x_mm, goal->pick_y_mm, goal->pick_z_mm, 0, 0, 0);
        auto place_pose_for_validation = robot_operation_->createPoseFromMmAndDegrees(
            goal->place_x_mm, goal->place_y_mm, goal->place_z_mm, 0, 0, 0);
            
        if (!validatePoseInWorkspace(pick_pose_for_validation) || !validatePoseInWorkspace(place_pose_for_validation)) {
            result->message = "Target poses outside workspace";
            result->failure_reason = "Workspace validation failed";
            goal_handle->abort(result);
            return;
        }
        if (check_timeout()) return;
        
        // Check for cancellation before starting main operation
        if (goal_handle->is_canceling()) {
            result->message = "Action cancelled before execution";
            result->failure_reason = "User cancellation";
            goal_handle->canceled(result);
            return;
        }
        
        publishFeedback(goal_handle, "executing_pick_place", 20.0, "Executing pick and place sequence");
        
        // IMPROVED: Execute with periodic cancellation checks
        bool operation_success = false;
        std::atomic<bool> execution_complete{false};
        
        // Run the main operation in a separate thread with timeout monitoring
        std::thread execution_thread([&]() {
            try {
                operation_success = executeProfessionalPickAndPlace(
                    goal->pick_x_mm, goal->pick_y_mm, goal->pick_z_mm, 
                    goal->pick_roll_deg, goal->pick_pitch_deg, goal->pick_yaw_deg,
                    goal->place_x_mm, goal->place_y_mm, goal->place_z_mm, 
                    goal->place_roll_deg, goal->place_pitch_deg, goal->place_yaw_deg,
                    goal->clearance_height_mm
                );
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "❌ Exception in main operation: %s", e.what());
                operation_success = false;
            }
            execution_complete.store(true);
        });
        
        // Monitor execution with periodic checks
        while (!execution_complete.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            if (check_timeout() || goal_handle->is_canceling()) {
                // Signal cancellation to robot operation
                if (robot_operation_) {
                    robot_operation_->emergencyStop();
                }
                execution_thread.join();
                
                if (goal_handle->is_canceling()) {
                    result->message = "Action cancelled during execution";
                    result->failure_reason = "User cancellation during operation";
                    goal_handle->canceled(result);
                } else {
                    goal_handle->abort(result);
                }
                return;
            }
        }
        
        execution_thread.join();
        
        // Calculate final timing
        auto end_time = std::chrono::steady_clock::now();
        auto total_time = std::chrono::duration<double>(end_time - start_time).count();
        
        // Populate result with enhanced metrics
        if (robot_operation_) {
            auto metrics = robot_operation_->getLastOperationMetrics();
            result->planning_time_seconds = metrics.planning_time.count();
            result->execution_time_seconds = metrics.execution_time.count();
            result->recovery_attempts = static_cast<uint32_t>(metrics.recovery_attempts);
        }
        
        // Set final result
        if (operation_success) {
            publishFeedback(goal_handle, "completed", 100.0, "Pick and place completed successfully");
            result->success = true;
            result->message = "Pick and place operation completed successfully";
            RCLCPP_INFO(node_->get_logger(), "✅ Pick and place action completed successfully (ID: %s, time: %.2fs)", 
                       goal->operation_id.c_str(), total_time);
            goal_handle->succeed(result);
        } else {
            result->message = "Pick and place operation failed";
            result->failure_reason = robot_operation_ ? robot_operation_->getLastOperationMetrics().failure_reason : "Unknown failure";
            RCLCPP_ERROR(node_->get_logger(), "❌ Pick and place action failed (ID: %s): %s", 
                        goal->operation_id.c_str(), result->failure_reason.c_str());
            goal_handle->abort(result);
        }
        
    } catch (const std::exception& e) {
        result->message = "Exception during pick and place execution";
        result->failure_reason = std::string("Exception: ") + e.what();
        RCLCPP_ERROR(node_->get_logger(), "❌ Exception in pick and place action: %s", e.what());
        goal_handle->abort(result);
    }
    
    // IMPROVED: Thread-safe cleanup with error handling
    try {
        std::lock_guard<std::mutex> lock(action_mutex_);
        action_in_progress_.store(false);
        current_goal_handle_.reset();
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Exception during action cleanup: %s", e.what());
    }
}

bool TrajectoryPlanner::validateConfiguration() const
{
    RCLCPP_INFO(node_->get_logger(), "🔍 Validating configuration...");
    
    // Validate workspace doesn't collide with table
    double table_top = config_.table.z_position + config_.table.height;
    double workspace_bottom = config_.workspace.z_position - config_.workspace.height / 2.0;
    
    if (workspace_bottom < table_top + 0.01) { // 1cm clearance
        RCLCPP_ERROR(node_->get_logger(), 
                    "❌ Workspace bottom (%.3f) too close to table top (%.3f)", 
                    workspace_bottom, table_top);
        return false;
    }
    
    // Validate workspace and table overlap in X-Y plane
    double table_x_min = config_.table.x_offset - config_.table.length / 2.0;
    double table_x_max = config_.table.x_offset + config_.table.length / 2.0;
    double table_y_min = config_.table.y_offset - config_.table.width / 2.0;
    double table_y_max = config_.table.y_offset + config_.table.width / 2.0;
    
    double workspace_x_min = config_.workspace.x_position - config_.workspace.width / 2.0;
    double workspace_x_max = config_.workspace.x_position + config_.workspace.width / 2.0;
    double workspace_y_min = config_.workspace.y_position - config_.workspace.depth / 2.0;
    double workspace_y_max = config_.workspace.y_position + config_.workspace.depth / 2.0;
    
    // Check if workspace completely contains table in X-Y
    if (table_x_min < workspace_x_min || table_x_max > workspace_x_max ||
        table_y_min < workspace_y_min || table_y_max > workspace_y_max) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Table extends outside workspace in X-Y plane - this may cause issues");
    }
    
    // Validate workspace dimensions are reasonable
    if (config_.workspace.width > 3.0 || config_.workspace.depth > 3.0 || config_.workspace.height > 3.0) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Large workspace dimensions may affect performance");
    }
    
    // Validate robot scaling factors
    if (config_.robot.velocity_scaling < 0.01 || config_.robot.velocity_scaling > 1.0) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Invalid velocity scaling: %.3f", config_.robot.velocity_scaling);
        return false;
    }
    
    if (config_.robot.acceleration_scaling < 0.01 || config_.robot.acceleration_scaling > 1.0) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Invalid acceleration scaling: %.3f", config_.robot.acceleration_scaling);
        return false;
    }
    
    // Validate planning parameters
    if (config_.robot.planning_time <= 0.1 || config_.robot.planning_time > 30.0) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Invalid planning time: %.3f (must be 0.1-30s)", config_.robot.planning_time);
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "✅ Configuration validation passed");
    return true;
}

void TrajectoryPlanner::publishFeedback(
    const std::shared_ptr<GoalHandlePickAndPlace> goal_handle,
    const std::string& phase, 
    double completion_percentage,
    const std::string& status_message)
{
    if (!goal_handle || goal_handle->is_canceling()) {
        return;
    }
    
    auto feedback = std::make_shared<PickAndPlaceAction::Feedback>();
    feedback->current_phase = phase;
    feedback->completion_percentage = completion_percentage;
    feedback->status_message = status_message.empty() ? phase : status_message;
    feedback->elapsed_time_seconds = 0.0;  // Could calculate if needed
    feedback->emergency_stop_available = operational_.load();
    
    goal_handle->publish_feedback(feedback);
    
    RCLCPP_DEBUG(node_->get_logger(), "📊 Feedback: %s (%.1f%%) - %s", 
                phase.c_str(), completion_percentage, feedback->status_message.c_str());
}

bool TrajectoryPlanner::validateActionGoal(const std::shared_ptr<const PickAndPlaceAction::Goal> goal)
{
    // Validate operation ID
    if (goal->operation_id.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Goal validation failed: empty operation_id");
        return false;
    }
    
    // Validate clearance height (in mm now)
    if (goal->clearance_height_mm <= 0.0 || goal->clearance_height_mm > 500.0) {  // 500mm max clearance
        RCLCPP_ERROR(node_->get_logger(), "❌ Goal validation failed: invalid clearance_height_mm %.1f", 
                    goal->clearance_height_mm);
        return false;
    }
    
    // Validate positions are within reasonable bounds (mm)
    if (std::abs(goal->pick_x_mm) > 2000.0 || std::abs(goal->pick_y_mm) > 2000.0 || 
        goal->pick_z_mm < -500.0 || goal->pick_z_mm > 2000.0) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Goal validation failed: pick position out of bounds [%.1f, %.1f, %.1f] mm", 
                    goal->pick_x_mm, goal->pick_y_mm, goal->pick_z_mm);
        return false;
    }
    
    if (std::abs(goal->place_x_mm) > 2000.0 || std::abs(goal->place_y_mm) > 2000.0 || 
        goal->place_z_mm < -500.0 || goal->place_z_mm > 2000.0) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Goal validation failed: place position out of bounds [%.1f, %.1f, %.1f] mm", 
                    goal->place_x_mm, goal->place_y_mm, goal->place_z_mm);
        return false;
    }
    
    // Validate orientations (degrees)
    if (std::abs(goal->pick_roll_deg) > 180.0 || std::abs(goal->pick_pitch_deg) > 180.0 || std::abs(goal->pick_yaw_deg) > 180.0 ||
        std::abs(goal->place_roll_deg) > 180.0 || std::abs(goal->place_pitch_deg) > 180.0 || std::abs(goal->place_yaw_deg) > 180.0) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Goal validation failed: orientation values must be within ±180 degrees");
        return false;
    }
    
    RCLCPP_DEBUG(node_->get_logger(), "✅ Goal validation passed for operation_id: %s", goal->operation_id.c_str());
    return true;
}

geometry_msgs::msg::Pose TrajectoryPlanner::convertToMeterPose(const geometry_msgs::msg::Pose& pose_in_mm)
{
    geometry_msgs::msg::Pose pose_in_m = pose_in_mm;
    pose_in_m.position.x /= 1000.0;
    pose_in_m.position.y /= 1000.0;
    pose_in_m.position.z /= 1000.0;
    // Orientation remains the same (quaternion)
    return pose_in_m;
}

//=================================== End Action Server ==============================

bool TrajectoryPlanner::addTCPCollisionGeometry()
{
    RCLCPP_INFO(node_->get_logger(), "🔧 Adding TCP collision geometry...");
    
    try {
        // Get the end-effector link name
        std::string ee_link = move_group_->getEndEffectorLink();
        if (ee_link.empty()) {
            RCLCPP_WARN(node_->get_logger(), "⚠️ No end-effector link found, using default TCP collision");
            return true; // MoveIt will use default
        }
        
        RCLCPP_INFO(node_->get_logger(), "🔧 End-effector link: %s", ee_link.c_str());
        
        // MoveIt automatically handles TCP collision - no additional setup needed
        // Custom TCP collision geometry can be added here if needed
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "❌ Failed to add TCP collision geometry: %s", e.what());
        return false;
    }
}

bool TrajectoryPlanner::validatePlanningSceneState()
{
    RCLCPP_INFO(node_->get_logger(), "🔍 Validating planning scene state...");
    
    try {
        // Simplified validation - just check if collision objects exist
        auto collision_objects = planning_scene_interface_->getKnownObjectNames();
        RCLCPP_INFO(node_->get_logger(), "📋 Found %zu collision objects in planning scene", collision_objects.size());
        
        for (const auto& obj_name : collision_objects) {
            RCLCPP_INFO(node_->get_logger(), "   - %s", obj_name.c_str());
        }
        
        return !collision_objects.empty();
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Planning scene validation failed: %s", e.what());
        return false;
    }
}

void TrajectoryPlanner::printCollisionObjectStatus()
{
    try {
        auto known_objects = planning_scene_interface_->getKnownObjectNames();
        RCLCPP_INFO(node_->get_logger(), "🛡️ Collision Objects Status:");
        RCLCPP_INFO(node_->get_logger(), "   Total objects: %zu", known_objects.size());
        
        for (const auto& name : known_objects) {
            RCLCPP_INFO(node_->get_logger(), "   ✅ %s", name.c_str());
        }
        
        if (known_objects.empty()) {
            RCLCPP_WARN(node_->get_logger(), "⚠️ No collision objects found! This may cause collision issues.");
        }
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Failed to get collision object status: %s", e.what());
    }
}

void TrajectoryPlanner::debugCollisionConfiguration()
{
    try {
        RCLCPP_INFO(node_->get_logger(), "🔍 Collision Configuration Debug:");
        RCLCPP_INFO(node_->get_logger(), "   Planning group: %s", move_group_->getName().c_str());
        RCLCPP_INFO(node_->get_logger(), "   End-effector link: %s", move_group_->getEndEffectorLink().c_str());
        RCLCPP_INFO(node_->get_logger(), "   Planning frame: %s", move_group_->getPlanningFrame().c_str());
        
        auto joint_names = move_group_->getJointNames();
        RCLCPP_INFO(node_->get_logger(), "   Controlled joints (%zu):", joint_names.size());
        for (size_t i = 0; i < std::min(joint_names.size(), size_t(6)); ++i) {
            RCLCPP_INFO(node_->get_logger(), "     - %s", joint_names[i].c_str());
        }
        
        auto link_names = move_group_->getLinkNames();
        RCLCPP_INFO(node_->get_logger(), "   Robot links (%zu total, showing first 6):", link_names.size());
        for (size_t i = 0; i < std::min(link_names.size(), size_t(6)); ++i) {
            RCLCPP_INFO(node_->get_logger(), "     - %s", link_names[i].c_str());
        }
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "⚠️ Debug collision configuration failed: %s", e.what());
    }
}


} // namespace trajectory_plan