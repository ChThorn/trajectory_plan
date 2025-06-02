# #!/usr/bin/env python3

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare

# def generate_launch_description():
#     # Launch arguments
#     model_id = LaunchConfiguration('model_id')
#     robot_ip = LaunchConfiguration('robot_ip')
#     use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    
#     # Declare arguments
#     model_id_arg = DeclareLaunchArgument(
#         'model_id',
#         default_value='rb3_730es_u',
#         description='Robot model ID'
#     )
    
#     robot_ip_arg = DeclareLaunchArgument(
#         'robot_ip',
#         default_value='10.0.2.7',
#         description='Robot IP address'
#     )
    
#     use_fake_hardware_arg = DeclareLaunchArgument(
#         'use_fake_hardware',
#         default_value='false',
#         description='Use fake hardware'
#     )
    
#     # Step 1: Launch rbpodo_bringup (includes all controllers)
#     rbpodo_bringup = IncludeLaunchDescription(
#         PathJoinSubstitution([
#             FindPackageShare('rbpodo_bringup'),
#             'launch',
#             'rbpodo.launch.py'
#         ]),
#         launch_arguments={
#             'model_id': model_id,
#             'robot_ip': robot_ip,
#             'use_fake_hardware': use_fake_hardware,
#             'use_rviz': 'false',  # We'll start RViz with MoveIt
#         }.items()
#     )
    
#     # Step 2: Activate joint_trajectory_controller (delayed)
#     activate_trajectory_controller = TimerAction(
#         period=5.0,  # Wait 5 seconds for rbpodo_bringup to be ready
#         actions=[
#             Node(
#                 package='controller_manager',
#                 executable='spawner',
#                 arguments=['joint_trajectory_controller'],
#                 output='screen',
#             )
#         ]
#     )
    
#     # Step 3: Launch MoveIt (without ros2_control_node, delayed)
#     moveit_launch = TimerAction(
#         period=8.0,  # Wait 8 seconds for controllers to be ready
#         actions=[
#             IncludeLaunchDescription(
#                 PathJoinSubstitution([
#                     FindPackageShare('rb3_730es_u_moveit_config'),
#                     'launch',
#                     'moveit.launch.py'
#                 ]),
#                 launch_arguments={
#                     'rviz_config': 'moveit.rviz',
#                 }.items()
#             )
#         ]
#     )
    
#     # Step 4: Launch pick and place node (delayed)
#     pick_and_place_node = TimerAction(
#         period=15.0,  # Wait 15 seconds for everything to be ready
#         actions=[
#             Node(
#                 package='trajectory_plan',
#                 executable='pick_and_place_node',
#                 name='pick_and_place_node',
#                 output='screen',
#                 parameters=[
#                     {'use_sim_time': False}
#                 ]
#             )
#         ]
#     )
    
#     return LaunchDescription([
#         model_id_arg,
#         robot_ip_arg,
#         use_fake_hardware_arg,
#         rbpodo_bringup,
#         activate_trajectory_controller,
#         moveit_launch,
#         pick_and_place_node
#     ])


#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    model_id = LaunchConfiguration('model_id')
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    
    # Declare arguments
    model_id_arg = DeclareLaunchArgument(
        'model_id',
        default_value='rb3_730es_u',
        description='Robot model ID'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='10.0.2.7',
        description='Robot IP address'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use fake hardware'
    )
    
    # Step 1: Launch rbpodo_bringup (hardware + basic visualization)
    rbpodo_bringup = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('rbpodo_bringup'),
            'launch',
            'rbpodo.launch.py'
        ]),
        launch_arguments={
            'model_id': model_id,
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'use_rviz': 'false',  # We'll use MoveIt's RViz instead
        }.items()
    )
    
    # Step 2: Launch MoveIt (planning server + MoveIt RViz) - delayed
    moveit_launch = TimerAction(
        period=5.0,  # Wait for rbpodo_bringup to be ready
        actions=[
            IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare('rb3_730es_u_moveit_config'),
                    'launch',
                    'moveit.launch.py'
                ]),
                launch_arguments={
                    'rviz_config': 'moveit.rviz',
                }.items()
            )
        ]
    )
    
    # Step 3: Launch pick and place node - delayed
    pick_and_place_node = TimerAction(
        period=15.0,  # Wait for everything to be ready
        actions=[
            Node(
                package='trajectory_plan',
                executable='pick_and_place_node',
                name='pick_and_place_node',
                output='screen',
                parameters=[
                    {'use_sim_time': False}
                ]
            )
        ]
    )
    
    return LaunchDescription([
        model_id_arg,
        robot_ip_arg,
        use_fake_hardware_arg,
        rbpodo_bringup,
        moveit_launch,
        pick_and_place_node
    ])