import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray

import cv2
from cv_bridge import CvBridge

class ApriltagDetection(Node):
    def __init__(self):
        super().__init__('apriltag_det')
        self.camera_subscriber_ = self.create_subscription(
            Image,
            '/zed/zed_node/left_raw/image_raw_color',
            self.camera_callback,
            10   
        )
        self.detection_publisher_ = self.create_publisher(
            Image,
            '/zed/zed_node/left_raw/detection',
            10
        )
        self.apriltag_subscriber_ = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.apriltag_callback,
            10
        )

        self.bridge = CvBridge()
        self.target_detected = False
        self.target_centre = None

    def camera_callback(self, img : Image):
        cv_img = self.bridge.imgmsg_to_cv2(img)
        display = cv_img
        if self.target_detected:
            display = cv2.circle(
                display, 
                (int(self.target_centre.x), int(self.target_centre.y)),
                5, (0,0,255), -1
            )

        display_msg = self.bridge.cv2_to_imgmsg(display)
        self.detection_publisher_.publish(display_msg)

    def apriltag_callback(self, msg : AprilTagDetectionArray):
        if len(msg.detections) > 0:
            self.target_detected = True
            self.target_centre = msg.detections[0].centre
        else:
            self.target_detected = False

def main(args=None):
    rclpy.init(args=args)

    apriltag_detection_node = ApriltagDetection()
    rclpy.spin(apriltag_detection_node)
    apriltag_detection_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()