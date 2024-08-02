import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'ros_obj_det'
    april_ros_share_dir = get_package_share_directory('apriltag_ros')
    tags_36h11_yaml_file = os.path.join(april_ros_share_dir, 'cfg', 'tags_36h11.yaml')

    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            remappings=[
                ('image_rect', '/zed/zed_node/left_raw/image_raw_color'),
                ('camera_info', '/zed/zed_node/left_raw/camera_info')
            ],
            parameters=[
                {'params_file': tags_36h11_yaml_file}
            ]
        ),
        Node(
            package=package_name,
            executable='apriltag_det',
            output='screen'
        ),
        Node(
            package=package_name,
            executable='yolo_obj_det',
            parameters=[{
                "package_share_path": FindPackageShare(package_name),
                'confidence_value': .5
            }],
            output='screen'
        )
    ])