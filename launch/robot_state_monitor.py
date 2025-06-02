#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import moveit_commander
import tf_transformations
import math
import time
from threading import Lock

class RobotStateMonitor(Node):
    def __init__(self):
        super().__init__('robot_state_monitor')
        
        self.get_logger().info('🤖 Robot State Monitor Node starting...')
        
        # Declare parameters
        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('planning_group', 'mainpulation')
        self.declare_parameter('display_precision', 2)
        
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.planning_group = self.get_parameter('planning_group').get_parameter_value().string_value
        self.precision = self.get_parameter('display_precision').get_parameter_value().integer_value
        
        self.get_logger().info(f'📊 Configuration:')
        self.get_logger().info(f'   Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'   Planning group: {self.planning_group}')
        self.get_logger().info(f'   Display precision: {self.precision} decimal places')
        
        # Publishers
        self.state_publisher = self.create_publisher(String, 'robot_current_state', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'robot_current_pose', 10)
        
        # Initialize MoveIt
        self.is_initialized = False
        self.move_group = None
        self.counter = 0
        self.init_lock = Lock()
        
        # Wait for MoveIt to be ready
        self.create_timer(2.0, self.initialize_moveit)  # One-time initialization
        
        # Create timer for periodic updates
        self.create_timer(1.0 / self.update_rate, self.publish_current_state)
        
        self.get_logger().info('🔄 Waiting for MoveIt to initialize...')

    def initialize_moveit(self):
        """Initialize MoveIt interface (called once)"""
        with self.init_lock:
            if self.is_initialized:
                return
                
            try:
                self.get_logger().info('🔄 Initializing MoveIt interface...')
                
                # Initialize moveit_commander
                moveit_commander.roscpp_initialize([])
                
                # Create move group interface
                self.move_group = moveit_commander.MoveGroupCommander(self.planning_group)
                
                self.get_logger().info('📋 MoveIt Interface Details:')
                self.get_logger().info(f'   Planning frame: {self.move_group.get_planning_frame()}')
                self.get_logger().info(f'   End effector frame: {self.move_group.get_end_effector_link()}')
                
                # Print available joints
                joint_names = self.move_group.get_active_joints()
                self.get_logger().info(f'   Active joints ({len(joint_names)}):')
                for joint in joint_names:
                    self.get_logger().info(f'     - {joint}')
                
                self.is_initialized = True
                self.get_logger().info('✅ MoveIt interface initialized successfully')
                self.get_logger().info('📍 Starting to monitor robot state...')
                
            except Exception as e:
                self.get_logger().error(f'❌ Failed to initialize MoveIt interface: {str(e)}')
                self.is_initialized = False

    def publish_current_state(self):
        """Publish current robot state"""
        if not self.is_initialized:
            if self.counter % 10 == 0:  # Throttled warning
                self.get_logger().warn('⚠️  MoveIt interface not initialized yet...')
            self.counter += 1
            return
        
        try:
            # Get current pose
            current_pose = self.move_group.get_current_pose()
            
            # Convert position to mm
            x_mm = current_pose.pose.position.x * 1000.0
            y_mm = current_pose.pose.position.y * 1000.0
            z_mm = current_pose.pose.position.z * 1000.0
            
            # Convert quaternion to Euler angles (degrees)
            orientation_q = current_pose.pose.orientation
            quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            euler = tf_transformations.euler_from_quaternion(quaternion)
            
            # Convert radians to degrees
            roll_deg = math.degrees(euler[0])
            pitch_deg = math.degrees(euler[1])
            yaw_deg = math.degrees(euler[2])
            
            # Create formatted string
            state_str = (f"Position (mm): X={x_mm:.{self.precision}f}, "
                        f"Y={y_mm:.{self.precision}f}, Z={z_mm:.{self.precision}f} | "
                        f"Orientation (deg): Roll={roll_deg:.{self.precision}f}, "
                        f"Pitch={pitch_deg:.{self.precision}f}, Yaw={yaw_deg:.{self.precision}f}")
            
            # Publish messages
            state_msg = String()
            state_msg.data = state_str
            self.state_publisher.publish(state_msg)
            self.pose_publisher.publish(current_pose)
            
            # Log to console with nice formatting
            self.counter += 1
            
            if self.counter % 5 == 1:  # Print header every 5 updates
                self.get_logger().info('')
                self.get_logger().info('🤖 ===== ROBOT STATE MONITOR =====')
                self.get_logger().info(f'    Frame: {self.move_group.get_planning_frame()} -> {self.move_group.get_end_effector_link()}')
            
            self.get_logger().info(f'📍 Position: X={x_mm:6.2f}, Y={y_mm:6.2f}, Z={z_mm:6.2f} mm')
            self.get_logger().info(f'🔄 Rotation: R={roll_deg:6.2f}°, P={pitch_deg:6.2f}°, Y={yaw_deg:6.2f}°')
            
            # Get joint positions
            joint_positions = self.move_group.get_current_joint_values()
            joint_names = self.move_group.get_active_joints()
            
            joint_str = '🔧 Joints: '
            for i, (name, pos) in enumerate(zip(joint_names, joint_positions)):
                if i > 0:
                    joint_str += ', '
                joint_str += f'{name}={math.degrees(pos):.2f}°'
            
            self.get_logger().info(joint_str)
            
            if self.counter % 5 == 0:
                self.get_logger().info('===============================')
                
        except Exception as e:
            if self.counter % 10 == 0:  # Throttled error
                self.get_logger().error(f'❌ Error getting robot state: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    node = RobotStateMonitor()
    
    node.get_logger().info('🚀 Starting Robot State Monitor...')
    node.get_logger().info('💡 Use Ctrl+C to stop monitoring')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Received shutdown signal')
    except Exception as e:
        node.get_logger().error(f'❌ Node crashed: {str(e)}')
    finally:
        node.get_logger().info('🛑 Robot State Monitor shutting down')
        if node.move_group:
            moveit_commander.roscpp_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()