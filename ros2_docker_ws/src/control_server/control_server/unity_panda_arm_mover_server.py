#!/usr/bin/env python3
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
# set pose goal with PoseStamped message
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
# moveit python library
from moveit.core.robot_state import RobotState
from moveit_msgs.msg import RobotTrajectory
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit.core.kinematic_constraints import construct_joint_constraint
from panda_arm_msg.srv import ControlRvizArm, ControlRvizArm_Response, ControlUnityArm, ControlUnityArm_Request, ControlUnityArm_Response
import numpy as np
import math

class Controller(Node):
    def __init__(self):
        super().__init__('commander')
        self.srv_control_rviz_arm = self.create_service(
            ControlRvizArm, 
            'control_rviz_arm', 
            self.handle_request_control_rviz_arm)
        
        self.before_position = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.before_gripper_state = 'close'
        
        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "panda_link0"
        # instantiate MoveItPy instance and get planning component
        self.panda = MoveItPy(node_name="moveit_py")
        self.panda_arm = self.panda.get_planning_component("panda_arm")
        self.panda_hand = self.panda.get_planning_component("hand")
        self.logger = get_logger("moveit_py.pose_goal")

        robot_model = self.panda.get_robot_model()
        self.robot_state = RobotState(robot_model)

    def plan_and_execute(self,
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
    ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
            logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory # 获取规划结果的轨迹
            robot_trajectory_msg = robot_trajectory.get_robot_trajectory_msg()
            joint_trajectory = robot_trajectory_msg.joint_trajectory # 获取关节轨迹
            # logger.info("joint_trajectory: {}".format(joint_trajectory))

            robot.execute(robot_trajectory, controllers=[]) # 控制rviz2的机械臂运动

            time.sleep(sleep_time)
            return True, joint_trajectory # 返回规划结果的轨迹
        else:
            logger.error("Planning failed")
            time.sleep(sleep_time)
            return False, None
    # function to move a gripper
    def move_to(self, x, y, z, xo, yo, zo, wo):
        # 经过标定
        self.pose_goal.pose.position.x = y
        self.pose_goal.pose.position.y = -x
        self.pose_goal.pose.position.z = z
        self.pose_goal.pose.orientation.x = xo
        self.pose_goal.pose.orientation.y = yo
        self.pose_goal.pose.orientation.z = zo
        self.pose_goal.pose.orientation.w = wo
        self.panda_arm.set_goal_state(pose_stamped_msg = self.pose_goal, pose_link="panda_link8")
        return self.plan_and_execute(self.panda, self.panda_arm, self.logger, sleep_time=1.0)

    # function for a gripper action
    def gripper_action(self, action):

        self.panda_hand.set_start_state_to_current_state()

        if action == 'open':
            joint_values = {"panda_finger_joint1": 0.03}
        elif action == 'close':
            joint_values = {"panda_finger_joint1": 0.001}
        else:
            self.get_logger().info("no such action")

        self.robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state = self.robot_state,
            joint_model_group = self.panda.get_robot_model().get_joint_model_group("hand"),
        )        
        self.panda_hand.set_goal_state(motion_plan_constraints=[joint_constraint])
        return self.plan_and_execute(self.panda, self.panda_hand, self.logger, sleep_time=3.0)

    def handle_request_control_rviz_arm(self, request, response):
        """"""
        position = request.position
        # gripper_state = request.open_or_close
        response = ControlRvizArm_Response()
        # 使用服务通信
        # 这里加判断 ，检测是否规划成功
        plan_flag, joint_trajectory = self.move_to(position[0], position[1], position[2], position[3], position[4], position[5], position[6])
        if not plan_flag:
            response.success = False
            return response

        # if gripper_state != self.before_gripper_state:
        #     if gripper_state == 'open':
        #         if not self.gripper_action("open"):
        #             response.success = False
        #             return response
        #     elif gripper_state == 'close':
        #         if not self.gripper_action("close"):
        #             response.success = False
        #             return response
        #     else:
        #         self.get_logger().info("no such action")

        # self.before_position = position
        # self.before_gripper_state = gripper_state
        response.joint_trajectory = joint_trajectory
        response.success = True
        return response

def main():
    """"""
    rclpy.init(args=None)

    controller = Controller()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = controller.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

    # rclpy.init(args=None)
    # node = Controller()
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

if __name__ == '__main__':
    """"""
    main()