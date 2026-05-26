"""Brings up Jerome's pure-Python NatNet ROS 2 driver (optitrack_ros_py) and
our example listener in one shot.

Native macOS variant: reads OPTITRACK_PC_IP / MY_IP / OPTITRACK_FRAME_ID
from the environment (set by run.sh), with lab-default fallbacks.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    server = os.environ.get('OPTITRACK_PC_IP', '192.168.201.5')
    client = os.environ.get('MY_IP', '192.168.201.89')
    frame = os.environ.get('OPTITRACK_FRAME_ID', 'world')

    # `client_address` is the LOCAL socket bind address (natnet_py
    # internally does `sock.bind((client_address, 0))`). Running natively
    # on a Mac with multiple interfaces (Wi-Fi + lab Ethernet), binding to
    # 0.0.0.0 produced `[Errno 65] No route to host` because asyncio's UDP
    # connect() then couldn't unambiguously pick an outbound interface for
    # 192.168.201.5. Binding to the lab-LAN IP specifically disambiguates.

    return LaunchDescription([
        Node(
            package='optitrack_ros_py',
            executable='optitrack_ros',
            name='optitrack_ros',
            output='screen',
            parameters=[{
                # Network
                'server_address': server,        # Motive PC — direct connect
                'client_address': client,        # bind specifically to the lab-LAN NIC
                'broadcast_address': '',         # disable discovery; without this,
                                                 # optitrack_ros_py defaults broadcast to
                                                 # 255.255.255.255 and natnet_py prefers
                                                 # discovery over direct connect.
                'timeout': 5.0,

                # Behavior
                'sync': True,
                'restamp': False,
                'diagnostics': True,
                'frame_id': frame,

                # Template applied to every rigid body Motive streams:
                # publish geometry_msgs/PoseStamped on /optitrack/<name>
                # and broadcast TF (frame) -> <name>.
                # The literal string "NAME" is the placeholder optitrack_ros_py
                # replaces with each rigid body's actual name at runtime.
                'rigid_bodies.default.enabled': True,
                'rigid_bodies.default.tf': True,
                'rigid_bodies.default.frame_id': 'NAME',
                'rigid_bodies.default.root_frame_id': frame,
                'rigid_bodies.default.topic': '/optitrack/NAME',
                'rigid_bodies.default.project_to_2d': False,
            }],
        ),

        Node(
            package='optitrack_listener',
            executable='listener',
            name='optitrack_listener',
            output='screen',
        ),
    ])
