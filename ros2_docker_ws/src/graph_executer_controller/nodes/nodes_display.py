#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from NodeGraphQt import BaseNode, NodeBaseWidget
from rclpy.node import Node
import rclpy
import json
from rclpy.node import Node
import re
from Qt import QtCore, QtWidgets
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PySide6.QtCore import *
from PySide6.QtWidgets import *
from PySide6.QtGui import *
from PySide6.Qt import *
import time
from utils.general import get_execution_order
from panda_arm_msg.srv import YoloImage, YoloImage_Request
import pyqtgraph as pg
import numpy as np
from ultralytics import YOLO
import os
import logging


BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

__all__ = ['SpinRos2Node','ImageDisplayByTopic','ImageDisplayByService']

class SpinRos2Node(BaseNode, QObject):
    __identifier__ = 'nodes.display'
    NODE_NAME = 'Spin ros2 node'

    def __init__(self):
        super(SpinRos2Node, self).__init__()
        self.add_input('ros2_node')

        self.add_checkbox('isDisplayImageLoop', text='开启图像显示循环')

        chk_widget = self.get_widget("isDisplayImageLoop")
        chk_widget.value_changed.connect(self.chk_value_changed)

        self.is_loop = False
    
    def chk_value_changed(self):
        """复选框值改变时的回调函数"""
        if self.get_property("isDisplayImageLoop"):
            self.is_loop = True
        else:
            self.is_loop = False

    def execute(self):
        """"""
        if not self.is_loop:
            ros2_node = self.input(0).connected_ports()[0].node().ros2_node
            rclpy.spin_once(ros2_node)
            # self.messageSignal.emit(f'{self.NODE_NAME} executed.')
            # print(f'{self.NODE_NAME} executed.')
        else:
            execution_order = get_execution_order(self)[:-1]
            while self.is_loop:
                # while rclpy.ok():
                    # print(f'{self.NODE_NAME} executing
                for node in execution_order:
                    if hasattr(node, 'execute'):
                        node.execute() # 运行节点
                ros2_node = self.input(0).connected_ports()[0].node().ros2_node
                rclpy.spin_once(ros2_node)
                # print(f'{self.NODE_NAME} executed.')
        
        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

class ImageData:
    def __init__(self):
        self.rgb_image = None

class ImageDisplayWidget(QtWidgets.QWidget):
    """
    Custom widget to be embedded inside a node.
    """
    img_display = Signal(ImageData)

    def __init__(self, parent=None):
        super(ImageDisplayWidget, self).__init__(parent)
        from ui.nodes.nodes_display import ui_image_display
        
        self.ui = ui_image_display.Ui_ImageDisplayForm()
        self.ui.setupUi(self)
        # logging.getLogger().setLevel(logging.WARNING)

        # self.imv = pg.ImageView()
        # self.ui.verticalLayout.addWidget(self.imv)
        self.label_img = QLabel(self)
        self.label_img.setMouseTracking(True)
        self.ui.verticalLayout.addWidget(self.label_img)
        self.label_fps = QLabel(self)
        self.label_fps.setMaximumHeight(30)
        self.label_fps.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.ui.verticalLayout.addWidget(self.label_fps)
        
        self.rgb_image = None
        self.img_display.connect(self.set_scene_data)

        self.num_frame = 0
        self.num_time = 0.0
        self.before_time = None

        model_path = os.path.join(BASE_DIR, 'res', 'models', 'YOLO', 'best.pt')
        self.yolo_model = YOLO(model_path)

    def set_scene_data(self, image_data: ImageData):
        """显示图像的函数，要使用信号来调用，不可外部调用"""
        rgb_image = np.rot90(image_data.rgb_image, 2)
        rgb_image = np.ascontiguousarray(rgb_image)
        # conf = 0.8表示置信度大于0.8的目标才会被检测出来
        results = self.yolo_model.predict(rgb_image, verbose=False, conf=0.9)

        current_annotated_image = results[0].plot()
        annotated_image = cv2.cvtColor(current_annotated_image, cv2.COLOR_BGR2RGB)

        height, width, channel = annotated_image.shape
        bytesPerLine = 3 * width
        qimage = QImage(annotated_image.data, width, height, bytesPerLine, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)
        self.label_img.setPixmap(pixmap.scaled(self.label_img.size(), Qt.KeepAspectRatio))

        # self.imv.setImage(rgb_image)
        
        if self.before_time:
            self.num_frame += 1
            self.num_time += time.time() - self.before_time
            if self.num_frame % 10 == 0:
                fps = self.num_frame / self.num_time
                # print(f"fps: {fps:.2f}")
                self.num_time = 0.0
                self.num_frame = 0
                self.label_fps.setText(f"fps: {fps:.2f}")
        
        self.before_time = time.time()

    def set_node_obj(self, obj):
        """"""
        self.node_obj = obj

    def closeEvent(self, event):
        self.node_obj.set_property("open_window", False)
        return super().closeEvent(event)

class NodeWidgetButton(NodeBaseWidget):
    """
    Wrapper that allows the widget to be added in a node object.
    """
    def __init__(self, parent=None):
        super(NodeWidgetButton, self).__init__(parent)
        # set the name for node property.
        self.set_name('button_widget')
        # set the custom widget.
        class MyCustomWidget(QtWidgets.QWidget):
            """
            Custom widget to be embedded inside a node.
            """
            def __init__(self, parent=None):
                super(MyCustomWidget, self).__init__(parent)
                self.btn = QtWidgets.QPushButton('get image topic')
                layout = QtWidgets.QHBoxLayout(self)
                layout.setContentsMargins(0, 0, 0, 0)
                layout.addWidget(self.btn)
                self.btn.clicked.connect(self.get_image_topic)
            def get_image_topic(self):
                """"""
                if not self.node_obj.is_created_ros2_node:
                    self.node_obj.create_ros2_node()

                # rclpy.spin_once(self.srv_node)
                topic_names_and_types = self.node_obj.srv_node.get_topic_names_and_types()
                image_topic = []
                # print("topic_names_and_types: ", topic_names_and_types)
                for topic_name, topic_types in topic_names_and_types:
                    # print(f"Topic: {topic_name}, Types: {topic_types}")
                    if topic_types[0] == "sensor_msgs/msg/Image":
                        print(f"Topic: {topic_name} is Image type")
                        image_topic.append(topic_name)

                print("image_topic: ", image_topic)
                self.node_obj.set_property("image_topic", image_topic)

                # self.node_obj.delete_ros2_node()

            def set_node_obj(self, obj):
                """"""
                self.node_obj = obj

        self.btn_widget = MyCustomWidget()
        self.set_custom_widget(self.btn_widget)
        # connect up the signals & slots.
        self.wire_signals()
    def wire_signals(self):
        """"""
    def get_value(self):
        """"""
    def set_value(self, value):
        """"""

class ImageDisplayByService(BaseNode, QObject):
    __identifier__ = 'nodes.display'
    NODE_NAME = 'Image display by service'

    def __init__(self):
        super(ImageDisplayByService, self).__init__()
        # self.add_input('start')
        self.add_output('ros2_node')
        self.add_output('rgb_image')

        self.is_created_ros2_node = False

        self.add_checkbox("open_window", text='show window')
        self.myui=ImageDisplayWidget()
        self.myui.set_node_obj(self)
        self.chk_value_changed()

        window_widget = self.get_widget("open_window")
        window_widget.value_changed.connect(self.chk_value_changed)

        self.cv_bridge = CvBridge()
        self.stop_loop = False
        self.ros2_node = None
        self.rgb_image = None

    def execute(self):
        """"""
        if not self.is_created_ros2_node:
            self.create_ros2_node()
        self.ros2_node = self.srv_node
        
        # 这里发送一个图片获取请求
        while not self.client.wait_for_service(timeout_sec=1.0):
            print('service not available, waiting again...')
        # 通过服务发送位置信号
        request = YoloImage_Request()
        request.num_image = 1 # 请求获取一张图片

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.srv_node, future)

        response = future.result()

        if response.success:
            images = response.images
            img = self.cv_bridge.imgmsg_to_cv2(response.images[0], "bgr8") #BGR
            # rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            image_data = ImageData()
            image_data.rgb_image = img
            # self.myui.rgb_image = rgb_image
            self.myui.img_display.emit(image_data)
            print(f"Received {len(images)} images from service")
        else:
            print("Failed to get image from service")
    
    def stop_execute(self):
        """"""
        self.stop_loop=True

    def chk_value_changed(self):
        # print("chk_value_changed")
        if self.get_property("open_window"):
            self.myui.show()
        else:
            self.myui.close()

    def create_ros2_node(self,):
        self.srv_node = Node(self.NODE_NAME.replace(' ', '_'))
        # self.sub_img = self.srv_node.create_subscription(Image, self.get_property("image_topic").strip(), self.camera_callback, 10)
        self.client = self.srv_node.create_client(YoloImage, 'yolo_image')
        self.is_created_ros2_node=True

    def delete_ros2_node(self,):
        """"""
        if self.is_created_ros2_node:
            self.srv_node.destroy_node()
            self.is_created_ros2_node = False

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

    def close_node(self,):
        """整个软件窗体关闭时调用"""
        self.myui.close()

    def _del_node(self):
        """删除节点前调用"""
        self.stop_loop=True
        self.myui.close()
        self.delete_ros2_node()

class ImageDisplayByTopic(BaseNode, QObject):
    __identifier__ = 'nodes.display'
    NODE_NAME = 'Image display by topic'

    def __init__(self):
        super(ImageDisplayByTopic, self).__init__()
        # self.add_input('start')
        self.add_output('ros2_node')

        self.is_created_ros2_node = False

        # self.add_text_input("image_topic", "image_topic")
        btn_widget = NodeWidgetButton(self.view)
        btn_widget.btn_widget.set_node_obj(self)
        self.add_custom_widget(btn_widget, tab='Custom')

        self.add_combo_menu("image_topic", "image_topic")
        self.add_checkbox("open_window", text='show window')
        self.myui=ImageDisplayWidget()
        self.myui.set_node_obj(self)
        self.chk_value_changed()

        window_widget = self.get_widget("open_window")
        window_widget.value_changed.connect(self.chk_value_changed)

        combo_menu_widget = self.get_widget("image_topic")
        combo_menu_widget.value_changed.connect(self.topic_changed)
        self.cv_bridge = CvBridge()
        self.stop_loop = False
        self.ros2_node = None

    def topic_changed(self):
        """"""
        if not self.is_created_ros2_node:
            self.create_ros2_node()

        combo_menu=self.get_widget("image_topic")
        combo_menu_text=combo_menu.get_value()
        if combo_menu_text == "":
            return
        self.sub_img = self.srv_node.create_subscription(Image, combo_menu_text.strip(), self.camera_callback, 10)
        print(f"topic_changed: {combo_menu_text.strip()}")
    
    def execute(self):
        """"""
        if not self.is_created_ros2_node:
            self.create_ros2_node()
        self.ros2_node = self.srv_node

    def stop_execute(self):
        """"""
        self.stop_loop=True

    def chk_value_changed(self):
        # print("chk_value_changed")
        if self.get_property("open_window"):
            self.myui.show() 
        else:
            self.myui.close()

    def create_ros2_node(self,):
        self.srv_node = Node(self.NODE_NAME.replace(' ', '_'))
        self.client = self.srv_node.create_client(YoloImage, 'yolo_image')
        self.is_created_ros2_node=True

    def camera_callback(self,data):
        """"""
        img = self.cv_bridge.imgmsg_to_cv2(data, "bgr8") #BGR
        # rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        image_data = ImageData()
        image_data.rgb_image = img
        # self.myui.rgb_image = rgb_image
        self.myui.img_display.emit(image_data)
        
    def delete_ros2_node(self,):
        """"""
        if self.is_created_ros2_node:
            self.srv_node.destroy_node()
            self.is_created_ros2_node = False

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

    def close_node(self,):
        """整个软件窗体关闭时调用"""
        self.myui.close()

    def _del_node(self):
        """删除节点前调用"""
        self.stop_loop=True
        self.myui.close()
        self.delete_ros2_node()

    



