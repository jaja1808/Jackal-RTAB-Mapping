import os
import rclpy
from rclpy.node import Node
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics import YOLO

class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')
        self.yolo_model = self.declare_parameter("yolo_model", "yolov8n.pt").value
        self.input_topic = self.declare_parameter("input_topic", "image_raw").value
        self.result_topic = self.declare_parameter("result_topic", "yolo_result").value
        self.result_image_topic = self.declare_parameter("result_image_topic", "yolo_image").value
        self.conf_thres = self.declare_parameter("conf_thres", 0.25).value
        self.iou_thres = self.declare_parameter("iou_thres", 0.45).value
        self.max_det = self.declare_parameter("max_det", 300).value
        self.classes = self.declare_parameter("classes", None).value
        self.tracker = self.declare_parameter("tracker", "bytetrack.yaml").value
        self.device = self.declare_parameter("device", None).value

        path = os.path.join(os.path.dirname(__file__), '../models')
        self.model = YOLO(f"{path}/{self.yolo_model}")
        self.model.fuse()

        self.bridge = cv_bridge.CvBridge()
        self.use_segmentation = self.yolo_model.endswith("-seg.pt")

        self.sub = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )
        self.results_pub = self.create_publisher(Detection2DArray, self.result_topic, 10)
        self.result_image_pub = self.create_publisher(Image, self.result_image_topic, 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model.track(
            source=cv_image,
            conf=self.conf_thres,
            iou=self.iou_thres,
            max_det=self.max_det,
            classes=self.classes,
            tracker=self.tracker,
            device=self.device,
            verbose=False,
            retina_masks=True
        )

        if results:
            yolo_result_msg = self.create_detections_array(results)
            yolo_result_image_msg = self.create_result_image(results)

            self.results_pub.publish(yolo_result_msg)
            self.result_image_pub.publish(yolo_result_image_msg)

    def create_detections_array(self, results):
        detections_msg = Detection2DArray()
        bounding_box = results[0].boxes.xywh
        classes = results[0].boxes.cls
        confidence_score = results[0].boxes.conf
        for bbox, cls, conf in zip(bounding_box, classes, confidence_score):
            detection = Detection2D()
            detection.bbox.center.x = float(bbox[0])
            detection.bbox.center.y = float(bbox[1])
            detection.bbox.size_x = float(bbox[2])
            detection.bbox.size_y = float(bbox[3])
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = int(cls)
            hypothesis.score = float(conf)
            detection.results.append(hypothesis)
            detections_msg.detections.append(detection)
        return detections_msg

    def create_result_image(self, results):
        plotted_image = results[0].plot()
        return self.bridge.cv2_to_imgmsg(plotted_image, encoding="bgr8")


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
