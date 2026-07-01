"""Dataset recording session: OptiTrack driver + Orbbec Femto Mega + rosbag.

IMPORTANT: must be run as root so pyorbbecsdk can claim the USB device.
Use the pixi task:  sudo pixi run record
(After running scripts/setup_usb_permissions.sh once, sudo needs no password.)

Mock mode (no hardware needed, for testing the pipeline):
  pixi run record-mock        — synthetic camera frames + synthetic pose,
                                no sudo, no OptiTrack server required.

Bag output: <repo_root>/datasets/optitrack_YYYYMMDD_HHMMSS/
Topics recorded:
  /optitrack/*               — PoseStamped of EVERY rigid body Motive streams
                               (the camera's marker body among them)
  /tf, /tf_static            — same poses as TF, for RViz playback
  /camera/color/image_raw    — Orbbec RGB frames   (stamped with capture time)
  /camera/depth/image_raw    — Orbbec depth frames (stamped with capture time)
  /camera/color/camera_info  — RGB intrinsics / calibration
  /camera/depth/camera_info  — depth intrinsics / calibration

Afterwards, export frames + per-frame interpolated camera pose with:
  pixi run export -- datasets/optitrack_YYYYMMDD_HHMMSS --rigid-body <name>
"""

import os
import re
from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

_CAMERA_NS = os.environ.get('CAMERA_NS', 'camera')
_MOCK = os.environ.get('MOCK_RECORD', '') not in ('', '0', 'false')

# Camera streams: full resolution — color 4K MJPEG + depth WFOV-unbinned
# 1024x1024 — both @5 fps (equal fps keeps hardware frame sync on).
#
# 5 fps keeps the total data rate (~13 MB/s: color ~3 + uncompressed 1024x1024
# depth ~10) under the ~16 MB/s the current USB 2.0 link sustains, so both
# streams hold a stable 5 fps at full resolution. Raise fps only on a real
# USB 3.0 / Gigabit-PoE link (see DATASET_RECORD.md); on USB-C bus power the
# camera also hard-limits depth to <=640x576 and color to <=1080p.
_COLOR_W = int(os.environ.get('CAMERA_COLOR_WIDTH', '3840'))
_COLOR_H = int(os.environ.get('CAMERA_COLOR_HEIGHT', '2160'))
_COLOR_FPS = int(os.environ.get('CAMERA_COLOR_FPS', '15'))
_DEPTH_W = int(os.environ.get('CAMERA_DEPTH_WIDTH', '1024'))
_DEPTH_H = int(os.environ.get('CAMERA_DEPTH_HEIGHT', '1024'))
_DEPTH_FPS = int(os.environ.get('CAMERA_DEPTH_FPS', '15'))

# Connection: 'usb' (default) or 'net' for Gigabit Ethernet / PoE. Net avoids
# the USB 2.0 bandwidth ceiling and needs no root. Set the camera's LAN IP:
#   CAMERA_CONNECTION=net CAMERA_NET_IP=192.168.201.XX pixi run record
_CONNECTION = os.environ.get('CAMERA_CONNECTION', 'usb')
_NET_IP = os.environ.get('CAMERA_NET_IP', '')
_NET_PORT = int(os.environ.get('CAMERA_NET_PORT', '8090'))

_RECORD_TOPICS = [
    '/tf',
    '/tf_static',
    f'/{_CAMERA_NS}/color/image_raw/compressed',
    f'/{_CAMERA_NS}/depth/image_raw',
    f'/{_CAMERA_NS}/ir/image_raw',
    f'/{_CAMERA_NS}/color/camera_info',
    f'/{_CAMERA_NS}/depth/camera_info',
    f'/{_CAMERA_NS}/ir/camera_info',
    f'/{_CAMERA_NS}/imu',
    # Sync evidence for later adjustment: /diagnostics carries the NatNet
    # clock-sync stats, /rosout preserves node warnings (e.g. the camera
    # falling back from capture-time to arrival-time stamps).
    '/diagnostics',
    '/rosout',
]


def _default_client_ip(server: str) -> str:
    """Local IP the OS would use to reach the Motive PC.

    A UDP connect() sends no packets but makes the kernel pick the outbound
    interface — so the NatNet socket binds to the right NIC without the user
    having to export MY_IP.
    """
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((server, 1510))
        return s.getsockname()[0]
    except OSError:
        return '192.168.201.89'
    finally:
        s.close()


def generate_launch_description():
    server = os.environ.get('OPTITRACK_PC_IP', '192.168.201.5')
    client = os.environ.get('MY_IP') or _default_client_ip(server)
    frame  = os.environ.get('OPTITRACK_FRAME_ID', 'world')

    bag_dir = os.path.join(
        os.getcwd(), 'datasets',
        f'optitrack_{datetime.now().strftime("%Y%m%d_%H%M%S")}',
    )

    if _MOCK:
        pose_source = Node(
            package='optitrack_listener',
            executable='mock_pose_publisher',
            name='mock_optitrack',
            output='screen',
        )
    else:
        pose_source = Node(
            package='optitrack_ros_py',
            executable='optitrack_ros',
            name='optitrack_ros',
            output='screen',
            parameters=[{
                'server_address':   server,
                'client_address':   client,
                'broadcast_address': '',
                'timeout': 5.0,
                # sync=True + restamp=False: poses are stamped with the
                # camera mid-exposure time converted through the NatNet
                # clock sync — i.e. capture time, network lag removed.
                'sync': True,
                'restamp': False,
                'diagnostics': True,
                'frame_id': frame,
                'rigid_bodies.default.enabled':      True,
                'rigid_bodies.default.tf':           True,
                'rigid_bodies.default.frame_id':     'NAME',
                'rigid_bodies.default.root_frame_id': frame,
                'rigid_bodies.default.topic':        '/optitrack/NAME',
                'rigid_bodies.default.project_to_2d': False,
            }],
        )

    return LaunchDescription([
        pose_source,

        # ── Orbbec Femto Mega camera (synthetic frames in mock mode) ────────
        Node(
            package='orbbec_camera_ros',
            executable='camera_node',
            name='orbbec_camera',
            output='screen',
            parameters=[{
                'mock': _MOCK,
                'connection': _CONNECTION,
                'net_ip': _NET_IP,
                'net_port': _NET_PORT,
                'color_width': _COLOR_W,
                'color_height': _COLOR_H,
                'color_fps': _COLOR_FPS,
                'depth_width': _DEPTH_W,
                'depth_height': _DEPTH_H,
                'depth_fps': _DEPTH_FPS,
            }],
        ),

        # ── rosbag recording ────────────────────────────────────────────────
        # Everything goes through ONE -e regex: /optitrack/.* catches every
        # rigid body regardless of Motive naming, the alternation covers the
        # fixed topics. Do NOT pass a topic list alongside -e — on Humble
        # that combination silently subscribes to nothing (empty bags).
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--output', bag_dir,
                 '-e', '^(' + '|'.join(['/optitrack/.*']
                                       + [re.escape(t) for t in _RECORD_TOPICS])
                       + ')$'],
            output='screen',
        ),
    ])
