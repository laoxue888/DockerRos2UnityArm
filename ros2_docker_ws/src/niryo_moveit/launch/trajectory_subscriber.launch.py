from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='niryo_moveit',
             executable='trajectory_subscriber.py',
             output='screen'),
    ])