#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # ============= LAUNCH ARGUMENTS =============
        DeclareLaunchArgument(
            'demo_mode',
            default_value='action_server',
            description='Demo mode: action_server or direct_api'
        ),
        
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.0.10',
            description='CB07 robot IP address for gripper'
        ),
        
        DeclareLaunchArgument(
            'gripper_simulation',
            default_value='false',
            description='Gripper simulation mode (true=simulation, false=real hardware)'
        ),
        
        DeclareLaunchArgument(
            'smooth_motion',
            default_value='true',
            description='Enable smooth motion mode'
        ),
        
        # ============= GRIPPER SERVICE NODE =============
        Node(
            package='rbpodo_gripper',
            executable='gripper_service_node',
            name='gripper_service_node',
            output='screen',
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'cb_simulation': LaunchConfiguration('gripper_simulation')
            }],
            prefix='',  # Can add 'xterm -e' for separate terminal if needed
        ),
        
        # ============= TRAJECTORY PLANNER NODE =============
        Node(
            package='trajectory_plan',
            executable='pick_and_place_node',
            name='pick_and_place_node',
            output='screen',
            parameters=[{
                'demo_mode': LaunchConfiguration('demo_mode'),
                'robot.smooth_motion': LaunchConfiguration('smooth_motion'),
                # Add any other trajectory planner parameters here
            }],
            # Wait a bit for gripper service to start first
            # This ensures gripper service is available when trajectory planner starts
        ),
        
    ])
