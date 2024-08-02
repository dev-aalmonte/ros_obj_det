import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster

import cv2
from cv_bridge import CvBridge

from .point_cloud_conversion import *

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
        self.point_cloud_subscriber = self.create_subscription(
            PointCloud2,
            "/zed/zed_node/point_cloud/cloud_registered",
            self.point_cloud_callback,
            10
        )
        self.depth_subscriber = self.create_subscription(
            Image,
            "/zed/zed_node/depth/depth_registered",
            self.depth_callback,
            10
        )

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster = TransformBroadcaster(self)
        
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.target_detected = False
        self.target_centre = None
        self.xyz_point: tuple = None
        self.static_t: TransformStamped = None
        self.tf_transfer = False

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

    def depth_callback(self, msg: Image):
        if self.target_detected:
            pass
            # z = get_depth_from_xy(msg, int(self.target_centre.x), int(self.target_centre.y))
            # self.get_logger().info(str(z))

    def point_cloud_callback(self, msg: PointCloud2):
        if self.target_detected:
            self.xyz_point = get_xyz_from_point_xy(msg, int(self.target_centre.x), int(self.target_centre.y))


    def apriltag_callback(self, msg: AprilTagDetectionArray):
        if len(msg.detections) > 0:
            self.target_detected = True
            self.target_centre = msg.detections[0].centre
            
            # h = Header()
            # h.stamp = self.get_clock().now().to_msg()
            # h.frame_id = "map"

            # # Publish a TF
            # if self.static_t == None:
            #     # if self.xyz_point != None and not self.xyz_point[0] != self.xyz_point[0]:
            #     self.static_t = get_tf_from_xyz((0.0,0.0,0.0), h, "zed_camera_link")
            #     # self.static_t.child_frame_id = "zed_camera_link"
            #     # self.static_t.transform.translation.x = 0
            #     # self.static_t.transform.translation.y = 0
            #     # self.static_t.transform.translation.z = 0

            #     self.static_broadcaster.sendTransform(self.static_t)
            
            # if self.static_t != None and not self.tf_transfer and self.xyz_point != None:
            #     # Set the 3D Position
            #     temp_header = Header()
            #     temp_header.stamp = self.get_clock().now().to_msg()
            #     temp_header.frame_id = "tag_temp"
            #     temp_tf = get_tf_from_xyz(self.xyz_point, temp_header, "zed_left_camera_frame")
            #     self.broadcaster.sendTransform(temp_tf)
                
            #     stamp = self.get_clock().now().to_msg()

            #     try:
            #         transformPos: TransformStamped = self.buffer.lookup_transform('map', 'tag_temp', stamp)
            #         self.static_broadcaster.sendTransform(transformPos)
            #         self.tf_transfer = transformPos
            #     except TransformException as ex:
            #         pass

        else:
            self.target_detected = False

def main(args=None):
    rclpy.init(args=args)
    try:
        apriltag_detection_node = ApriltagDetection()
        rclpy.spin(apriltag_detection_node)
        apriltag_detection_node.destroy_node()
    except KeyboardInterrupt as ex:
        rclpy.shutdown()
        pass


if __name__ == '__main__':
    main()