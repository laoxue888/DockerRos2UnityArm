#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from NodeGraphQt import BaseNode
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from panda_arm_msg.srv import ControlRvizArm, ControlRvizArm_Request
import rclpy

__all__ = ['PandaArmControlNode']

class PandaArmControlNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = 'nodes.arm.control'
    NODE_NAME = 'panda arm control'

    def __init__(self):
        super(PandaArmControlNode, self).__init__()
        # self.add_input('text_in')
        self.add_output('next_step')
        
        self.srv_node = Node(self.NODE_NAME.replace(' ', '_'))
        self.cli = self.srv_node.create_client(ControlRvizArm, 'control_rviz_arm')

        self.add_text_input('position.x', 'position.x', text='0.3')
        self.add_text_input('position.y', 'position.y', text='0.3')
        self.add_text_input('position.z', 'position.z', text='0.3')
        self.add_text_input('orientation.x', 'orientation.x', text='0.0')
        self.add_text_input('orientation.y', 'orientation.y', text='0.0')
        self.add_text_input('orientation.z', 'orientation.z', text='0.0')
        self.add_text_input('orientation.w', 'orientation.w', text='0.0')

        self.add_combo_menu('gripper_state', 'gripper_state', items=['open', 'close'])

    def execute(self):
        """"""
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # 通过服务发送位置信号
        request = ControlRvizArm_Request()
        request.position = [float(self.get_property('position.x')), 
                            float(self.get_property('position.y')), 
                            float(self.get_property('position.z')),
                            float(self.get_property('orientation.x')),
                            float(self.get_property('orientation.y')),
                            float(self.get_property('orientation.z')),
                            float(self.get_property('orientation.w'))]
        request.open_or_close = self.get_property('gripper_state')
 
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self.srv_node, future)

        response = future.result()

        if response:
            self.messageSignal.emit(f'{request.position}执行成功')
        else:
            self.messageSignal.emit(f'{request.position}执行失败')

        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

