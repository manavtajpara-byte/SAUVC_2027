import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sauvc_arena_sim')
    arena_urdf_file = os.path.join(pkg_share, 'urdf', 'sauvc_arena.urdf')
    auv_urdf_file = os.path.join(pkg_share, 'urdf', 'sauvc_auv.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'sauvc_arena.rviz')

    with open(arena_urdf_file, 'r') as f:
        arena_desc = f.read()

    with open(auv_urdf_file, 'r') as f:
        auv_desc = f.read()

    return LaunchDescription([
        # 1. Publishes static Arena and Props on /arena_description (Fixed to world)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='arena_state_publisher',
            output='screen',
            parameters=[{'robot_description': arena_desc}],
            remappings=[('/robot_description', '/arena_description')]
        ),

        # 2. Publishes AUV Robot Model on /robot_description (Rooted at base_link)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='auv_state_publisher',
            output='screen',
            parameters=[{'robot_description': auv_desc}]
        ),

        # 3. 6-DoF Kinematics & Autonomous Mission Execution Controller
        Node(
            package='sauvc_arena_sim',
            executable='sauvc_mission_controller_node',
            name='sauvc_mission_controller_node',
            output='screen'
        ),

        # 4. RViz2 3D Visualizer
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),

        # 5. Interactive Simulation Control Dashboard (Start, Stop, Reset & Manual Steering)
        Node(
            package='sauvc_arena_sim',
            executable='sim_control_panel.py',
            name='sim_control_panel',
            output='screen'
        )
    ])
