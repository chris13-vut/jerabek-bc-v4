#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from ultralytics import YOLO
import math
import os
import cv2

class YoloRosNode(Node):
    def __init__(self):
        super().__init__('yolo_tracker_node')
        self.publisher_ = self.create_publisher(Point, '/camera/target', 10)
        self.H = 1.7          
        self.PITCH_RAD = math.radians(15)   
        self.FOV_V_RAD = math.radians(55)   
        self.model_path = "/home/chris/Downloads/yolo11m-visdrone.pt"
        self.image_path = "/home/chris/estimate_yolo/test.JPG"
        self.output_path = "/home/chris/estimate_yolo/vzdalenost_test.JPG"
        if not os.path.exists(self.model_path): return
        self.model = YOLO(self.model_path)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def calculate_distance_and_angle(self, x1, y1, x2, y2, img_width, img_height):
        h_px = y2 - y1
        x_center, y_center = (x1 + x2) / 2, (y1 + y2) / 2
        f_px = img_height / (2 * math.tan(self.FOV_V_RAD / 2))
        local_pitch = self.PITCH_RAD + math.atan((y_center - (img_height / 2)) / f_px)
        distance = (self.H * (math.cos(local_pitch)**2) * f_px) / h_px
        yaw_angle = math.atan((x_center - (img_width / 2)) / f_px)
        return distance, yaw_angle

    def timer_callback(self):
        if not os.path.exists(self.image_path):
            self.publish_target(0.0, 0.0, False)
            return
        results = self.model(self.image_path, device='cpu', verbose=False)
        r = results[0]
        img_height, img_width = r.orig_shape
        best_det, best_score = None, 0.0
        for det in r.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = det
            if int(class_id) in [0, 1] and score > best_score:
                best_score, best_det = score, det
        if best_det:
            x1, y1, x2, y2, score, class_id = best_det
            dist, ang = self.calculate_distance_and_angle(x1, y1, x2, y2, img_width, img_height)
            self.publish_target(ang, dist, True)
        else:
            self.publish_target(0.0, 0.0, False)

    def publish_target(self, angle, distance, detected):
        msg = Point()
        msg.x, msg.y, msg.z = float(angle), float(distance), 1.0 if detected else 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloRosNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
