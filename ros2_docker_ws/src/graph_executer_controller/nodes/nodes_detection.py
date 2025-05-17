#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from NodeGraphQt import BaseNode

class YoloDetectionNode(BaseNode):
    """
    YoloDetectionNode is a node that performs object detection using YOLOv8.
    It takes an image as input and outputs the detected objects.
    """

    def __init__(self):
        super(YoloDetectionNode, self).__init__()
        self.add_input("image", "Image")
        self.add_output("detected_objects", "Detected Objects")
        self.add_output("detection_time", "Detection Time")
        self.add_output("detection_result", "Detection Result")