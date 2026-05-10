#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import math
import os
import cv2

class YoloRosNode(Node):
    def __init__(self):
        super().__init__('yolo_tracker_node')
        
        # Publisher pro data pro dron (C++ uzel)
        self.publisher_ = self.create_publisher(Point, '/camera/target', 10)
        
        # Publisher pro vizuální kontrolu (pokreslený obrázek)
        self.img_pub_ = self.create_publisher(Image, '/camera/yolo_annotated', 10)
        
        # Subscriber přijímající surová data z kamery
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        self.bridge = CvBridge()
        
        self.H = 1.7          
        self.PITCH_RAD = math.radians(15)   
        self.FOV_V_RAD = math.radians(55)   
        self.model_path = "/home/chris/Downloads/yolo11m-visdrone.pt"
        
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"MODEL NENALEZEN: {self.model_path}")
            return
            
        self.get_logger().info(f"Nacitam model: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.get_logger().info("Uzel spusten. Cekam na obrazky v topicu /camera/image_raw ...")

    def calculate_distance_and_angle(self, x1, y1, x2, y2, img_width, img_height):
        h_px = y2 - y1
        x_center, y_center = (x1 + x2) / 2, (y1 + y2) / 2
        f_px = img_height / (2 * math.tan(self.FOV_V_RAD / 2))
        local_pitch = self.PITCH_RAD + math.atan((y_center - (img_height / 2)) / f_px)
        distance = (self.H * (math.cos(local_pitch)**2) * f_px) / h_px
        yaw_angle = math.atan((x_center - (img_width / 2)) / f_px)
        return distance, yaw_angle

    def image_callback(self, msg):
        # 1. Převod ROS Image zprávy na OpenCV formát (numpy array)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Chyba prevodu obrazku: {e}")
            return

        img_height, img_width = cv_image.shape[:2]
        
        # 2. Spuštění detekce YOLO přímo na datech v RAM
        results = self.model(cv_image, device='cpu', verbose=False)
        r = results[0]
        
        best_det = None
        best_score = 0.0
        
        for det in r.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = det
            if int(class_id) in [0, 1] and score > best_score:
                best_score = score
                best_det = det
        
        if best_det:
            x1, y1, x2, y2, score, class_id = best_det
            dist, ang = self.calculate_distance_and_angle(x1, y1, x2, y2, img_width, img_height)
            self.publish_target(ang, dist, True)
            
            # Vykreslení do obrázku
            cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            label = f"{dist:.1f}m | {math.degrees(ang):.1f}st"
            cv2.putText(cv_image, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            self.publish_target(0.0, 0.0, False)

        # 3. Publikace výsledného obrázku zpět do ROS (pro RVIZ nebo rqt_image_view)
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.img_pub_.publish(annotated_msg)
        except Exception as e:
            pass

    def publish_target(self, angle, distance, detected):
        msg = Point()
        msg.x, msg.y, msg.z = float(angle), float(distance), 1.0 if detected else 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
