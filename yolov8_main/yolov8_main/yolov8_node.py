import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from cv_bridge import CvBridge

from ultralytics import YOLO

from sensor_msgs.msg import Image
from yolov8_msg.msg import Yolov8InferenceMsg
from yolov8_msg.msg import Yolov8Inference
from yolov8_msg.msg import BoundingBox
from yolov8_msg.msg import Mask
from yolov8_msg.msg import Point2D

import cv2
import random
import numpy as np

class Yolov8Node(Node) :
    
    def __init__(self) :
        super().__init__('yolov8_node')

        self._class_to_color = {}

        # parameter default setting
        self.declare_parameter('weight', 'yolov8n.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('conf_threshold', '0.5')
        self.declare_parameter('iou_threshold', '0.7')
        self.declare_parameter('image_reliability', QoSReliabilityPolicy.BEST_EFFORT)

        # get parameter value
        weight = self.get_parameter('weight').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.iou_threshold = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.image_qos_profile = QoSProfile(
            reliability=self.get_parameter('image_reliability').get_parameter_value().integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # parameter for image processing
        self.cv_bridge = CvBridge()
        self.model = YOLO(weight)
        # self.model.fuse()

        # create a publisher for yolov8 inference results
        self.pub = self.create_publisher(Yolov8InferenceMsg, '/yolov8_inference', 1)
        self.img_pub = self.create_publisher(Image, '/yolov8_result', 1)
        # self.img_pub = self.create_publisher(Image, 'inference_result', 1)
        # create a subscription for receiving raw image
        self.sub = self.create_subscription(Image, '/image_raw', self.img_callback, self.image_qos_profile)
        
    def img_callback(self, msg) : 
        # convert ROS messgae to CV2 image
        img = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

        # yolov8 results
        results = self.model.predict(
            source = img,
            conf = self.conf_threshold,
            iou = self.iou_threshold,
            device = self.device)
        results = results[0].cpu()

        inference_msg = Yolov8InferenceMsg()

        if not results.boxes :
            self.get_logger().info('No objects detected')
            return
        
        for i in range(len(results)) :
            inference_result =  Yolov8Inference()

            if results.boxes :
                bbox_info = results.boxes[i]
                inference_result.class_id = int(bbox_info.cls)
                inference_result.class_name = self.model.names[int(bbox_info.cls)]
                inference_result.score = float(bbox_info.conf)

                bbox_msg = BoundingBox()

                bbox = bbox_info.xywh[0]
                bbox_msg.center.x = float(bbox[0])
                bbox_msg.center.y = float(bbox[1])
                bbox_msg.size.x = float(bbox[2])
                bbox_msg.size.y = float(bbox[3])
                
                inference_result.bbox = bbox_msg

                label = inference_result.class_name

                if label not in self._class_to_color :
                    r = random.randint(0, 255)
                    g = random.randint(0, 255)
                    b = random.randint(0, 255)
                    self._class_to_color[label] = (r, g, b)
                
                color = self._class_to_color[label]

                pt1 = (round(bbox_msg.center.x - bbox_msg.size.x / 2.0),
                        round(bbox_msg.center.y - bbox_msg.size.y / 2.0))
                pt2 = (round(bbox_msg.center.x + bbox_msg.size.x / 2.0),
                        round(bbox_msg.center.y + bbox_msg.size.y / 2.0))
                cv2.rectangle(img, pt1, pt2, color, 2)
                cv2.putText(img, str(inference_result.class_name), ((pt1[0]+pt2[0])//2-5, pt1[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, thickness=2)

            if results.masks :
                mask_info = results.masks[i]
                mask_msg = Mask()
                mask_msg.data = [Point2D(x=float(point[0]), y=float(point[1])) for point in mask_info.xy[0].tolist()]
                mask_msg.height = results.orig_img.shape[0]
                mask_msg.width = results.orig_img.shape[1]

                inference_result.mask = mask_msg

                mask_array = np.array([[int(point.x), int(point.y)] for point in mask_msg.data])

                temp = img.copy()
                temp = cv2.fillPoly(temp, [mask_array], color)
                cv2.addWeighted(img, 0.4, temp, 0.6, 0, img)
                img = cv2.polylines(img, [mask_array], True, color, 2)

            inference_msg.yolov8_inference.append(inference_result)

        inference_msg.header = msg.header
        self.img_pub.publish(self.cv_bridge.cv2_to_imgmsg(img))
        self.pub.publish(inference_msg)

def main() :
    rclpy.init(args=None)
    node = Yolov8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()