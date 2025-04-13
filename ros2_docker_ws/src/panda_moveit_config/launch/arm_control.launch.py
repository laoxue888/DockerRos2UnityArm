#!/usr/bin/env python3

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def generate_launch_description():
    """
    说明：由于gazebo无法启动调试，因此将部分节点放到这里来启动，以实现arm_control_from_UI.py文件的调试
    """
    arm_robot_sim_path = os.path.join(get_package_share_directory('panda_moveit_config'))
    
    moveit_config = (
        MoveItConfigsBuilder(robot_name="panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        # .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        # .moveit_cpp(arm_robot_sim_path + "/config/controller_setting.yaml")
        .to_moveit_configs()
    )
    #.joint_limits(file_path="config/joint_limits.yaml")
    #.robot_description_kinematics(file_path="config/kinematics.yaml")

    moveit_py_node = Node(
        name="moveit_py",
        package="unity_control_example",
        executable="control_test_node",
        output="both",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": True},
                    ],
    )

    return LaunchDescription(
        [
            moveit_py_node,
        ]
    )