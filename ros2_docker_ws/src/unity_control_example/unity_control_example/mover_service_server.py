#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from niryo_moveit.srv import MoverService, MoverService_Request, MoverService_Response
from pymoveit2 import MoveIt2, MoveIt2State
from rclpy.callback_groups import ReentrantCallbackGroup
from typing import List
from niryo_moveit.msg import RobotTrajectory

def joint_names() -> List[str]:
    return [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    ]


def base_link_name() -> str:
    return "base_link"


def end_effector_name() -> str:
    return "hand_link"


class MoverServiceServer(Node):
    def __init__(self):
        super().__init__('mover_service_server')
        # 创建服务，指定服务类型和回调函数
        self.srv = self.create_service(
            MoverService, 
            'niryo_moveit', 
            self.handle_mover_request)
        
        self.declare_parameter("synchronous", True)
        # If non-positive, don't cancel. Only used if synchronous is False
        self.declare_parameter("cancel_after_secs", 0.0)
        # Planner ID
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")

        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names(),
            base_link_name=base_link_name(),
            end_effector_name=end_effector_name(),
            group_name="niryo_one_arm",
            callback_group=callback_group,
        )

        self.moveit2.planner_id = (
            self.get_parameter("planner_id").get_parameter_value().string_value
        )

        # Scale down velocity and acceleration of joints (percentage of maximum)
        # moveit2.max_velocity = 0.5
        # moveit2.max_acceleration = 0.5

        synchronous = self.get_parameter("synchronous").get_parameter_value().bool_value
        cancel_after_secs = (
            self.get_parameter("cancel_after_secs").get_parameter_value().double_value
        )
        
        self.get_logger().info("MoverService Server 已启动，等待请求...")
    
    def plan_trajectory(self, moveit2, destination_pose, start_joint_angles):
        """"""

    def handle_mover_request(self, request: MoverService_Request, response: MoverService_Response):
        """
        经过测试的服务请求处理函数，可以！！！！
        处理服务请求的回调函数"""
        # 处理请求 NiryoMoveitJoints joints_input，geometry_msgs/Pose pick_pose，geometry_msgs/Pose place_pose
        response = MoverService_Response()
        self.get_logger().info(f"收到请求: \n{request}")
        joints_input = request.joints_input
        pick_pose = request.pick_pose
        place_pose = request.place_pose

        # 填充响应, 规划完后返回 RobotTrajectory[] trajectories
        # ------------------------------------------------
        # Pre grasp - position gripper directly above target object
        # geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.3251306384334037, y=-0.00011786139226348373, z=0.2591435232868775), orientation=geometry_msgs.msg.Quaternion(x=0.6168519094330289, y=0.6164980955928598, z=0.3459624507522454, w=0.34602572538570503))
        # pick_pose.position.x = 0.2251306384334037
        # pick_pose.position.y = 0.117186139226348373
        pick_pose.position.z = 0.3591435232868775
        # pick_pose.orientation.x = 0.6168519094330289
        # pick_pose.orientation.y = 0.6164980955928598
        # pick_pose.orientation.z = 0.3459624507522454
        # pick_pose.orientation.w = 0.34602572538570503

        # rviz2执行动作
        self.moveit2.move_to_pose(position=pick_pose.position, quat_xyzw=pick_pose.orientation, target_link='hand_link')

        pre_grasp_pose_joint_trajectory = self.moveit2.plan(position=pick_pose.position, quat_xyzw=pick_pose.orientation, target_link='hand_link')
        pre_grasp_pose = RobotTrajectory()
        pre_grasp_pose.joint_trajectory = pre_grasp_pose_joint_trajectory
        # If the trajectory has no points, planning has failed and we return an empty response
        if not pre_grasp_pose.joint_trajectory.points:
            return response
        response.trajectories.append(pre_grasp_pose)

        self.get_logger().info(f"发送响应: \n{response}")
        return response

def main(args=None):
    rclpy.init(args=args)
    
    mover_server = MoverServiceServer()
    
    try:
        rclpy.spin(mover_server)
    except KeyboardInterrupt:
        mover_server.get_logger().info("服务端被用户中断")
    finally:
        # 清理资源
        mover_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()