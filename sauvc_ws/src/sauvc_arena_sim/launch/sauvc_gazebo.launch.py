import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sauvc_arena_sim')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg_share, 'worlds', 'sauvc_arena.world')
    auv_urdf_file = os.path.join(pkg_share, 'urdf', 'sauvc_auv.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'sauvc_arena.rviz')

    with open(auv_urdf_file, 'r') as f:
        auv_desc = f.read()

    return LaunchDescription([
        # 1. Start Gazebo Server & Client with the SAUVC Arena World
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path, 'verbose': 'true'}.items()
        ),

        # 2. Publish Robot Description for ROS 2 & RViz
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='auv_state_publisher',
            output='screen',
            parameters=[{'robot_description': auv_desc}]
        ),

        # 3. Spawn the SAUVC AUV directly into the Gazebo pool at the Starting Zone
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='auv_spawner',
            output='screen',
            arguments=[
                '-entity', 'sauvc_auv',
                '-file', auv_urdf_file,
                '-x', '0.7',
                '-y', '2.0',
                '-z', '1.8',
                '-Y', '0.0'
            ]
        ),

        # 4. SAUVC Autonomous Mission Controller Node
        Node(
            package='sauvc_arena_sim',
            executable='sauvc_mission_controller_node',
            name='sauvc_mission_controller_node',
            output='screen'
        ),

        # 5. Interactive Simulation Control Dashboard (Start, Stop, Reset & Manual Steering)
        Node(
            package='sauvc_arena_sim',
            executable='sim_control_panel.py',
            name='sim_control_panel',
            output='screen'
        )
    ])
