#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <iomanip>
#include <sstream>
#include <cmath>

class RobustRobotStateMonitor : public rclcpp::Node
{
public:
    RobustRobotStateMonitor() : Node("robust_robot_state_monitor")
    {
        RCLCPP_INFO(this->get_logger(), "🤖 Robust Robot State Monitor Node starting...");
        
        // Declare parameters
        this->declare_parameter("update_rate", 2.0);
        this->declare_parameter("planning_group", "mainpulation");
        this->declare_parameter("display_precision", 2);
        
        update_rate_ = this->get_parameter("update_rate").as_double();
        planning_group_ = this->get_parameter("planning_group").as_string();
        precision_ = this->get_parameter("display_precision").as_int();
        
        RCLCPP_INFO(this->get_logger(), "📊 Configuration:");
        RCLCPP_INFO(this->get_logger(), "   Update rate: %.1f Hz", update_rate_);
        RCLCPP_INFO(this->get_logger(), "   Planning group: %s", planning_group_.c_str());
        RCLCPP_INFO(this->get_logger(), "   Display precision: %d decimal places", precision_);
        
        // Publishers
        state_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "robot_current_state", 10);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "robot_current_pose", 10);
        
        // Joint state subscriber - DIRECTLY subscribe to /joint_states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RobustRobotStateMonitor::jointStateCallback, this, std::placeholders::_1));
        
        // Initialize robot model
        init_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&RobustRobotStateMonitor::initializeRobotModel, this));
        
        RCLCPP_INFO(this->get_logger(), "✅ Robust Robot State Monitor initialized!");
        RCLCPP_INFO(this->get_logger(), "📍 Waiting for robot model and joint states...");
    }

private:
    void initializeRobotModel()
    {
        try {
            RCLCPP_INFO(this->get_logger(), "🔄 Loading robot model...");
            
            // Cancel the timer since we only need it once
            if (init_timer_) {
                init_timer_->cancel();
                init_timer_.reset();
            }
            
            robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
                shared_from_this(), "robot_description");
            
            kinematic_model_ = robot_model_loader_->getModel();
            if (!kinematic_model_) {
                RCLCPP_ERROR(this->get_logger(), "❌ Could not load robot model");
                // Retry in 5 seconds
                init_timer_ = this->create_wall_timer(
                    std::chrono::seconds(5),
                    std::bind(&RobustRobotStateMonitor::initializeRobotModel, this));
                return;
            }
            
            kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
            kinematic_state_->setToDefaultValues();
            
            joint_model_group_ = kinematic_model_->getJointModelGroup(planning_group_);
            if (!joint_model_group_) {
                RCLCPP_ERROR(this->get_logger(), "❌ Could not find joint group: %s", planning_group_.c_str());
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "📋 Robot Model Details:");
            RCLCPP_INFO(this->get_logger(), "   Model frame: %s", kinematic_model_->getModelFrame().c_str());
            RCLCPP_INFO(this->get_logger(), "   Planning group: %s", planning_group_.c_str());
            
            auto joint_names = joint_model_group_->getVariableNames();
            RCLCPP_INFO(this->get_logger(), "   Active joints (%zu):", joint_names.size());
            for (const auto& joint : joint_names) {
                RCLCPP_INFO(this->get_logger(), "     - %s", joint.c_str());
            }
            
            // Get end effector link
            const auto& link_names = joint_model_group_->getLinkModelNames();
            if (!link_names.empty()) {
                end_effector_link_ = link_names.back();
                RCLCPP_INFO(this->get_logger(), "   End effector: %s", end_effector_link_.c_str());
            }
            
            is_initialized_ = true;
            
            // Now create the periodic timer for state updates
            auto period = std::chrono::duration<double>(1.0 / update_rate_);
            timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(period),
                std::bind(&RobustRobotStateMonitor::publishCurrentState, this));
            
            RCLCPP_INFO(this->get_logger(), "✅ Robot model loaded successfully!");
            RCLCPP_INFO(this->get_logger(), "📍 Started monitoring joint states directly...");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to load robot model: %s", e.what());
            // Retry in 5 seconds
            init_timer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                std::bind(&RobustRobotStateMonitor::initializeRobotModel, this));
            is_initialized_ = false;
        }
    }
    
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!is_initialized_) return;
        
        // Update the robot state with current joint positions
        try {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (i < msg->position.size()) {
                    kinematic_state_->setJointPositions(msg->name[i], &msg->position[i]);
                }
            }
            
            // Store latest joint state
            latest_joint_state_ = *msg;
            has_joint_data_ = true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "❌ Error updating robot state: %s", e.what());
        }
    }
    
    void publishCurrentState()
    {
        if (!is_initialized_ || !has_joint_data_) {
            static int warning_count = 0;
            if (warning_count++ % 10 == 0) {
                RCLCPP_WARN(this->get_logger(), "⚠️  Waiting for robot model and joint state data...");
            }
            return;
        }
        
        try {
            kinematic_state_->enforceBounds();
            
            // Get end effector pose
            const Eigen::Isometry3d& end_effector_state = 
                kinematic_state_->getGlobalLinkTransform(end_effector_link_);
            
            // Convert to position (mm) and orientation (degrees)
            double x_mm = end_effector_state.translation().x() * 1000.0;
            double y_mm = end_effector_state.translation().y() * 1000.0;
            double z_mm = end_effector_state.translation().z() * 1000.0;
            
            // Convert rotation matrix to RPY
            Eigen::Matrix3d rotation_matrix = end_effector_state.rotation();
            Eigen::Vector3d euler = rotation_matrix.eulerAngles(0, 1, 2);
            
            double roll_deg = euler[0] * 180.0 / M_PI;
            double pitch_deg = euler[1] * 180.0 / M_PI;
            double yaw_deg = euler[2] * 180.0 / M_PI;
            
            // Create formatted string
            std::stringstream ss;
            ss << std::fixed << std::setprecision(precision_);
            ss << "Position (mm): X=" << x_mm << ", Y=" << y_mm << ", Z=" << z_mm;
            ss << " | Orientation (deg): Roll=" << roll_deg << ", Pitch=" << pitch_deg << ", Yaw=" << yaw_deg;
            
            // Publish string message
            auto state_msg = std_msgs::msg::String();
            state_msg.data = ss.str();
            state_publisher_->publish(state_msg);
            
            // Create and publish pose message
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = kinematic_model_->getModelFrame();
            pose_msg.pose.position.x = end_effector_state.translation().x();
            pose_msg.pose.position.y = end_effector_state.translation().y();
            pose_msg.pose.position.z = end_effector_state.translation().z();
            
            Eigen::Quaterniond quat(end_effector_state.rotation());
            pose_msg.pose.orientation.x = quat.x();
            pose_msg.pose.orientation.y = quat.y();
            pose_msg.pose.orientation.z = quat.z();
            pose_msg.pose.orientation.w = quat.w();
            
            pose_publisher_->publish(pose_msg);
            
            // Console output
            static int counter = 0;
            counter++;
            
            if (counter % 5 == 1) {
                RCLCPP_INFO(this->get_logger(), " ");
                RCLCPP_INFO(this->get_logger(), "🤖 ===== ROBUST ROBOT STATE MONITOR =====");
                RCLCPP_INFO(this->get_logger(), "    Frame: %s -> %s", 
                           kinematic_model_->getModelFrame().c_str(), end_effector_link_.c_str());
            }
            
            RCLCPP_INFO(this->get_logger(), 
                       "📍 Position: X=%*.2f, Y=%*.2f, Z=%*.2f mm", 
                       6, x_mm, 6, y_mm, 6, z_mm);
            RCLCPP_INFO(this->get_logger(), 
                       "🔄 Rotation: R=%*.2f°, P=%*.2f°, Y=%*.2f°", 
                       6, roll_deg, 6, pitch_deg, 6, yaw_deg);
            
            // Display joint positions
            std::stringstream joint_ss;
            joint_ss << "🔧 Joints: ";
            for (size_t i = 0; i < latest_joint_state_.name.size(); ++i) {
                if (i > 0) joint_ss << ", ";
                if (i < latest_joint_state_.position.size()) {
                    joint_ss << latest_joint_state_.name[i] << "=" 
                            << std::fixed << std::setprecision(2) 
                            << (latest_joint_state_.position[i] * 180.0 / M_PI) << "°";
                }
            }
            RCLCPP_INFO(this->get_logger(), "%s", joint_ss.str().c_str());
            
            if (counter % 5 == 0) {
                RCLCPP_INFO(this->get_logger(), "===============================");
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "❌ Error computing robot state: %s", e.what());
        }
    }
    
    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr kinematic_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    
    sensor_msgs::msg::JointState latest_joint_state_;
    std::string end_effector_link_{"tcp"};
    std::string planning_group_;
    double update_rate_;
    int precision_;
    bool is_initialized_{false};
    bool has_joint_data_{false};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RobustRobotStateMonitor>();
    
    RCLCPP_INFO(node->get_logger(), "🚀 Starting Robust Robot State Monitor...");
    RCLCPP_INFO(node->get_logger(), "💡 Use Ctrl+C to stop monitoring");
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "❌ Node crashed: %s", e.what());
    }
    
    RCLCPP_INFO(node->get_logger(), "🛑 Robust Robot State Monitor shutting down");
    rclcpp::shutdown();
    return 0;
}