#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import numpy as np

class MouseCameraNode(Node):
    def __init__(self):
        super().__init__('mouse_camera_node')
        self.pub = self.create_publisher(Point, '/camera/target', 10)
        
        # Nastavení okna
        self.width = 640
        self.height = 480
        self.mouse_x = self.width // 2
        self.mouse_y = self.height // 2
        self.detected = False

        cv2.namedWindow("Kamera Simulator (Ovladani mysi)")
        cv2.setMouseCallback("Kamera Simulator (Ovladani mysi)", self.mouse_event)
        
        self.timer = self.create_timer(0.1, self.publish_position)
        self.get_logger().info("Simulator spusten. Hybej mysi v okne pro simulaci cloveka.")

    def mouse_event(self, event, x, y, flags, param):
        self.mouse_x = x
        self.mouse_y = y
        # Simulujeme detekci jen když je myš v okně (nebo můžeme přidat kliknutí)
        self.detected = True

    def publish_position(self):
        # Přepočet X souřadnice na úhel (např. -30 až +30 stupňů)
        # Střed (320) = 0 stupňů
        relative_x = (self.mouse_x - (self.width / 2)) / (self.width / 2)
        angle = relative_x * 40.0 # Rozsah +- 40 stupňů
        
        # Přepočet Y souřadnice na vzdálenost (např. 2 až 15 metrů)
        # Spodek okna = blízko, Vršek = daleko
        distance = 15.0 - (self.mouse_y / self.height) * 12.0

        msg = Point()
        msg.x = float(angle)
        msg.y = float(distance)
        msg.z = 1.0 if self.detected else 0.0
        
        self.pub.publish(msg)

        # Vykreslení "obrazu z kamery"
        img = np.zeros((self.height, self.width, 3), np.uint8)
        # Nakreslíme "člověka" (kolečko) na pozici myši
        cv2.circle(img, (self.mouse_x, self.mouse_y), 10, (0, 0, 255), -1)
        cv2.putText(img, f"Uhel: {angle:.1f} deg", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(img, f"Vzdalenost: {distance:.1f} m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("Kamera Simulator (Ovladani mysi)", img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = MouseCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
