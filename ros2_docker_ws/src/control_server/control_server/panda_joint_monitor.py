import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatusArray, GoalStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory
import threading
from PySide6.QtCore import Signal, QObject, Slot
from multiprocessing import Queue
from panda_arm_msg.srv import ControlUnityArm
from panda_arm_msg.srv import ControlUnityArm_Request, ControlUnityArm_Response
from moveit_msgs.msg import RobotTrajectory

class GetTrajectoryThread(threading.Thread):

    def __init__(self,):
        super().__init__()
        self.daemon = True  # 设置为守护线程


    def send_trajectory(self, trajectory: JointTrajectory):
        """发送轨迹"""

class FollowJointTrajectoryMonitor(Node):
    def __init__(self):
        super().__init__('panda_joint_monitor')

        # group_name = 'niryo_one_arm'
        group_name = 'panda_arm'
        
        # 设置QoS配置以匹配动作服务器
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # 订阅动作状态主题
        self._status_sub = self.create_subscription(
            GoalStatusArray,
            '/{}_controller/follow_joint_trajectory/_action/status'.format(group_name),
            self._status_callback,
            qos_profile
        )
        
        # 订阅反馈主题
        self._feedback_sub = self.create_subscription(
            FollowJointTrajectory.Impl.FeedbackMessage,
            '/{}_controller/follow_joint_trajectory/_action/feedback'.format(group_name),
            self._feedback_callback,
            qos_profile
        )

        self.get_logger().info('开始监听/{}_controller/follow_joint_trajectory动作信息...'.format(group_name))

        # 发布话题
        self.publisher_ = self.create_publisher(JointTrajectory, '/panda_joint_trajectory', 10)
        self.panda_arm_joint_trajectory = JointTrajectory()

        # self.send_trajectory_thread = GetTrajectoryThread()
        # self.send_trajectory_thread.start()
        # self.queue_trajectory = Queue()  # 使用双端队列来存储轨迹点

        self.srv_get_trajectory = self.create_service(
            ControlUnityArm, 
            'get_trajectory_panda_joint_monitor', 
            self.handle_request_get_trajectory)
        self.panda_trajectory = None

    def handle_request_get_trajectory(self, request: ControlUnityArm_Request, response: ControlUnityArm_Response):
        """"Handle the request to send trajectory to panda_joint_monitor."""
        self.panda_trajectory = request.joint_trajectory
        response = ControlUnityArm_Response()
        response.success = True
        return response

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('收到反馈:')
        self.get_logger().info(f'Goal ID: {feedback_msg.goal_id.uuid}')
        self.get_logger().info(f'Header: {feedback.header}')
        self.get_logger().info(f'Joint names: {feedback.joint_names}')
        if feedback.desired.positions:
            self.get_logger().info(f'Desired positions: {feedback.desired.positions}')
        if feedback.actual.positions:
            self.get_logger().info(f'Actual positions: {feedback.actual.positions}')
        if feedback.error.positions:
            self.get_logger().info(f'Error positions: {feedback.error.positions}')
        self.get_logger().info('---')

        # 这里应该创建队列，进队出队，然后使用动作通信，从而实现虚拟机械臂的控制
        # self.panda_arm_joint_trajectory.header = feedback.header
        # self.panda_arm_joint_trajectory.joint_names = feedback.joint_names
        # self.panda_arm_joint_trajectory.points.append(feedback.desired)

    def _status_callback(self, status_msg):
        for status_info in status_msg.status_list:
            self.get_logger().info(f'收到状态更新 - Goal ID: {status_info.goal_info.goal_id.uuid}')
            self.get_logger().info(f'状态: {self._status_to_str(status_info.status)}')
            self.get_logger().info('---')
        
    def _status_to_str(self, status):
        if status == GoalStatus.STATUS_UNKNOWN:
            return 'UNKNOWN'
        elif status == GoalStatus.STATUS_ACCEPTED:
            return 'ACCEPTED'
        elif status == GoalStatus.STATUS_EXECUTING:
            return 'EXECUTING'
        elif status == GoalStatus.STATUS_CANCELING:
            return 'CANCELING'
        elif status == GoalStatus.STATUS_SUCCEEDED:
            return 'SUCCEEDED'
        elif status == GoalStatus.STATUS_CANCELED:
            return 'CANCELED'
        elif status == GoalStatus.STATUS_ABORTED:
            return 'ABORTED'
        else:
            return f'UNKNOWN_STATUS_CODE_{status}'

def main(args=None):
    # rclpy.init(args=args)
    # monitor = FollowJointTrajectoryMonitor()
    # rclpy.spin(monitor)
    # monitor.destroy_node()
    # rclpy.shutdown()

    rclpy.init(args=None)

    monitor = FollowJointTrajectoryMonitor()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(monitor)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = monitor.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()