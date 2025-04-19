# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("niryo_one", package_name="niryo_one_moveit_config").to_moveit_configs()
#     return generate_demo_launch(moveit_config)

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("niryo_one", package_name="niryo_one_moveit_config").to_moveit_configs()
    
    # 获取原有的launch描述
    demo_launch = generate_demo_launch(moveit_config)
    
    # 创建您想要添加的节点
    example_node = Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            emulate_tty=True,
            parameters=[
                {'ROS_IP': '0.0.0.0'},
                {'ROS_TCP_PORT': 10000},
            ],
        )
    
    # 将原有launch描述和新节点组合
    launch_description = LaunchDescription()
    launch_description.add_action(demo_launch)
    launch_description.add_action(example_node)
    
    return launch_description