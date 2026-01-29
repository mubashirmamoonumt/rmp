from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             '-s', 'libgazebo_ros_factory.so',
             'worlds/mobile_world.world'],
        output='screen'
    )

    return LaunchDescription([gazebo])
