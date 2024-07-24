from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'ros_obj_det'

    return LaunchDescription([
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