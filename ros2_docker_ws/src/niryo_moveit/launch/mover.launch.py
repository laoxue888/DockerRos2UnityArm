from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'tcp_ip',
            default_value='0.0.0.0',
            description='TCP IP address for the server endpoint'
        ),
        DeclareLaunchArgument(
            'tcp_port',
            default_value='10000',
            description='TCP port for the server endpoint'
        ),
        
        # Server Endpoint Node
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            emulate_tty=True,
            parameters=[
                {'ROS_IP': '0.0.0.0'},
                {'ROS_TCP_PORT': 10000},
            ],
        ),
        
        # Mover Node
        Node(
            package='niryo_moveit',
            executable='mover.py',
            name='mover',
            output='screen',
            arguments=['--wait']
        ),
        
        # # Include demo.launch.py
        # IncludeLaunchDescription(
        #     os.path.join(
        #         get_package_share_directory('niryo_moveit'),
        #         'launch/demo.launch.py'
        #     )
        # )
    ])