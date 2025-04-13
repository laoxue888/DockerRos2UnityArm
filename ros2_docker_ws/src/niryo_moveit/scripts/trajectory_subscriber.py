#!/usr/bin/env python3

import rclpy
from niryo_moveit.msg import NiryoMoveitJoints
from rclpy.node import Node
import threading

class NovelSubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.novel_subscriber_ = self.create_subscription(NiryoMoveitJoints, "/niryo_joints", self.novel_callback, 10)

    def novel_callback(self, msg):
        self.get_logger().info("I heard:\n%s", msg)
        print("I heard:\n%s", msg)

# 要加这个if语句，方便调试节点
if __name__=='__main__':
    rclpy.init(args=None)
    node_subscriber = NovelSubNode("Trajectory_Subscriber")

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    rate = node_subscriber.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
