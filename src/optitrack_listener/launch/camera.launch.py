"""Camera-only session: just the Orbbec Femto Mega node, no OptiTrack, no bag.

IMPORTANT: must be run as root so pyorbbecsdk can claim the USB device.
Use the pixi task:  sudo pixi run camera
(After running scripts/setup_usb_permissions.sh once, sudo needs no password.)

Published topics (see orbbec_camera_ros/camera_node.py):
  /camera/color/image_raw    — sensor_msgs/Image  (rgb8)
  /camera/depth/image_raw    — sensor_msgs/Image  (16UC1, millimetres)
  /camera/color/camera_info  — RGB intrinsics / calibration
  /camera/depth/camera_info  — depth intrinsics / calibration

View with:  pixi run rviz   (add an Image display on /camera/color/image_raw)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orbbec_camera_ros',
            executable='camera_node',
            name='orbbec_camera',
            output='screen',
        ),
    ])
