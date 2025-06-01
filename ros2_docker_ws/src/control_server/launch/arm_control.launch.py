#!/usr/bin/env python3
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    """
    说明：由于gazebo无法启动调试，因此将部分节点放到这里来启动，以实现arm_control_from_UI.py文件的调试
    """
    arm_robot_sim_path = os.path.join(get_package_share_directory('panda_moveit_config'))
    
    moveit_config = (
        MoveItConfigsBuilder(robot_name="panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(arm_robot_sim_path + "/config/controller_setting.yaml") # controller_setting 手动创建的
        .to_moveit_configs()
    )

    launch_description = LaunchDescription(
        [
            Node(
                name="moveit_py",
                package='control_server',
                executable='unity_panda_arm_mover_server',
                output="both",
                parameters=[moveit_config.to_dict(),
                            {"use_sim_time": True},
                            {"limited":False},
                        ],
            ),
            Node(
                name="get_end_position",
                package='control_server',
                executable='get_end_position',
                output="both",
            ),

            Node(
                name="panda_joint_monitor",
                package='control_server',
                executable='panda_joint_monitor',
                output="both",
            ),

            # Node(
            # package='ros2_tcp_endpoint',
            # executable='default_server_endpoint',
            # emulate_tty=True,
            # parameters=[
            #     {'ROS_IP': '0.0.0.0'},
            #     {'ROS_TCP_PORT': 10000},
            # ],
            # ),
        ] 
    )

    # 获取原有的launch描述
    # rviz_moveit_config = MoveItConfigsBuilder("panda", package_name="panda_moveit_config").to_moveit_configs()
    # rviz_demo_launch = generate_demo_launch(moveit_config)
    # launch_description.add_action(rviz_demo_launch)

    return launch_description