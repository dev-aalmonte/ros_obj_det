import rclpy
from rclpy.node import Node

from .point_cloud_conversion import get_xyz_from_point_xy

import cv2
from cv_bridge import CvBridge

import torch

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped

from tf2_ros.transform_broadcaster import TransformBroadcaster

class YoloObjDet(Node):

    def __init__(self):
        super().__init__('yolo_obj_det')

        self.raw_image_subscriber = self.create_subscription(
            Image,
            '/zed/zed_node/left_raw/image_raw_color',
            self.rec_image_callback,
            10)
        self.point_cloud_subscriber = self.create_subscription(
            PointCloud2,
            "/zed/zed_node/point_cloud/cloud_registered",
            self.point_cloud_callback,
            10)
        self.publisher_ = self.create_publisher(
            Image,
            '/zed/zed_node/left_raw/image_highlight',
            10)
        
        self.broadcaster = TransformBroadcaster(self)
        
        self.declare_parameter('confidence_value', rclpy.Parameter.Type.DOUBLE)
        package_share_param = self.declare_parameter("package_share_path", rclpy.Parameter.Type.STRING)
        package_share_path = package_share_param.get_parameter_value().string_value

        self.model = torch.hub.load("ultralytics/yolov5", "custom", f"{package_share_path}/models/best.onnx", verbose=False)
        self.bridge = CvBridge()
        self.colors = [
            (255,0,0),
            (0,255,0),
            (0,0,255),
            (255,255,0),
            (255,0,255),
            (0,255,255)
        ]

        self.objects = []
        self.objloc = {}
    
    def point_cloud_callback(self, msg : PointCloud2):
        if len(self.objects) > 0:
            for obj in self.objects:
                w, h = obj['center_point']
                x_float, y_float, z_float = get_xyz_from_point_xy(msg, w, h)

                if not x_float != x_float:
                    self.objloc[obj['name']] = (
                        round(x_float, 2),
                        round(y_float, 2),
                        round(z_float, 2)
                    )

                    # Publish a TF
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = "zed_left_camera_frame"
                    t.child_frame_id = f"{obj['name']}_frame"

                    t.transform.translation.x = x_float
                    t.transform.translation.y = y_float
                    t.transform.translation.z = z_float

                    self.broadcaster.sendTransform(t)

    def rec_image_callback(self, msg : Image):
        self.objects = []
        # Convert from message to MAT
        cv_img : cv2.Mat = self.bridge.imgmsg_to_cv2(msg)
        # Resize the image to the requiered for the model 640x640
        resized = cv2.resize(cv_img, (640, 640), interpolation=cv2.INTER_LINEAR)
        # Run the model
        result = self.model(resized)
        pandas = result.pandas().xyxy[0]
        # Draw some squares on the original image
        display = cv_img
        for index, row in pandas.iterrows():
            if row.confidence < self.get_parameter('confidence_value').get_parameter_value().double_value:
                continue

            percentages_coord = [
                (row.xmin / 640, row.ymin / 640), 
                (row.xmax / 640, row.ymax / 640)
            ]
            
            min_orig = (
                int(msg.width * percentages_coord[0][0]),
                int(msg.height * percentages_coord[0][1])
            )

            max_orig = (
                int(msg.width * percentages_coord[1][0]),
                int(msg.height * percentages_coord[1][1])
            )
            
            center = (
                int(max_orig[0] - ((max_orig[0] - min_orig[0]) / 2)),
                int(max_orig[1] - ((max_orig[1] - min_orig[1]) / 2))
            )

            self.objects.append({
                'objclass': row['class'],
                'name': row['name'],
                'confidence': round(row['confidence'], 2),
                'min_point': min_orig,
                'max_point': max_orig,
                'center_point': center
            })

            # Modifying image
            display = cv2.rectangle(
                display, 
                min_orig,
                max_orig,
                self.colors[index % len(self.colors)],
                2
            )

            label = f"{row['class']}:{row['name']} - {row.confidence:0,.2f}"
            (text_width, text_height), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_COMPLEX, 0.6, 1 
            )

            display = cv2.circle(
                display,
                center,
                3,
                self.colors[index % len(self.colors)],
                -1
            )

            if row['name'] in self.objloc:
                display = cv2.putText(
                    display, f"{self.objloc[row['name']]}",
                    (center[0] + 20, center[1]),
                    cv2.FONT_HERSHEY_COMPLEX, 0.6, self.colors[index % len(self.colors)], 1
                )

            display = cv2.rectangle(
                display,
                (min_orig[0], min_orig[1] - 20),
                (min_orig[0] + text_width, min_orig[1]),
                self.colors[index % len(self.colors)],
                -1
            )
            display = cv2.putText(
                display, label,
                (min_orig[0], min_orig[1]),
                cv2.FONT_HERSHEY_COMPLEX, 0.6, (255,255,255), 1
            )

        display_msg = self.bridge.cv2_to_imgmsg(display)
        self.publisher_.publish(display_msg)

        # Display Image (Testing purposes only)
        # cv2.imshow("Display Image", display)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    yolo_obj_det_node = YoloObjDet()
    rclpy.spin(yolo_obj_det_node)
    yolo_obj_det_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()