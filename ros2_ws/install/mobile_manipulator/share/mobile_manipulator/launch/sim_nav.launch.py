import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'mobile_manipulator'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'mobile_world.world')

    return LaunchDescription([
        # 1. Start Gazebo with your world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
            output='screen'
        ),

        # 2. Publish Robot State (TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]
        ),

        # 3. Spawn Robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity', 'my_bot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.2']
        )
    ])