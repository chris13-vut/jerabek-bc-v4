#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        
        # Publisher do tématu, které poslouchá YOLO node
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Časovač nastavený na 2 sekundy
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.bridge = CvBridge()
        
        # Cesta k obrázku, který se bude publikovat
        self.image_path = "/home/chris/estimate_yolo/test.JPG"

        if not os.path.exists(self.image_path):
            self.get_logger().error(f"UPOZORNENI: Obrazek nenalezen na ceste: {self.image_path}")
        else:
            self.get_logger().info(f"Kamera simulator spusten. Publikuji {self.image_path} kazde 2 sekundy.")

    def timer_callback(self):
        if not os.path.exists(self.image_path):
            self.get_logger().warning("Cekam na vlozeni obrazku test.JPG...", throttle_duration_sec=2.0)
            return

        # Načtení obrázku pomocí OpenCV
        cv_image = cv2.imread(self.image_path)
        
        if cv_image is None:
            self.get_logger().error("Obrazek existuje, ale nejde nacist (mozna je poskozeny).")
            return

        try:
            # Převod z OpenCV formátu (numpy array) do ROS 2 Image zprávy
            msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            
            # Odeslání do ROSu
            self.publisher_.publish(msg)
            self.get_logger().info("Snímek odeslán do /camera/image_raw")
        except Exception as e:
            self.get_logger().error(f"Chyba pri prevodu/publikaci: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
