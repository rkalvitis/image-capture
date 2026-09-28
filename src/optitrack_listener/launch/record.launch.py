"""iPhone photo-session pose recording: OptiTrack driver + rosbag.

Records ONLY the session's rigid bodies (RECORD_BODIES env var, default
phone/phone_mount/popillia_mount) for the duration of a stop-and-shoot photo
session. No camera node, no root — photos are taken on the iPhone and
matched to these poses afterwards by scripts/match_photos.py via EXIF
timestamps.

Use the pixi tasks:
  pixi run record-poses       — real OptiTrack (Motive must be streaming)
  pixi run record-mock        — synthetic pose publisher, no hardware

Bag output: <repo_root>/datasets/phone_YYYYMMDD_HHMMSS/  (or $RECORD_DIR)
Topics recorded (single -e regex — on Humble, mixing a topic list with -e
silently records nothing):
  /optitrack/phone, /optitrack/phone_mount, /optitrack/popillia_mount
  (override the body list per run:  RECORD_BODIES="a b c" pixi run record-poses)

Session routine (see docs/IPHONE_IMPLEMENTATION_PLAN.md Stage 1.2):
start this, photograph the ms clock, shoot stop-and-shoot (~3 s still per
shot), photograph the clock again, Ctrl-C, AirDrop the DNGs into
datasets/phone_<ts>/photos/ and the clock shots into .../clock/.
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

_MOCK = os.environ.get('MOCK_RECORD', '') not in ('', '0', 'false')


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
        return '192.168.201.245'
    finally:
        s.close()


def generate_launch_description():
    server = os.environ.get('OPTITRACK_PC_IP', '192.168.201.5')
    client = os.environ.get('MY_IP') or _default_client_ip(server)
    frame  = os.environ.get('OPTITRACK_FRAME_ID', 'world')

    # RECORD_DIR (2026-09-15): record straight into a dataset folder, e.g.
    # RECORD_DIR=datasets/sep15_10deg_9vertical_FS/mocap (the image-capture
    # coordinator sets it); default = datasets/phone_<timestamp>/ as before.
    bag_dir = os.environ.get('RECORD_DIR') or os.path.join(
        os.getcwd(), 'datasets',
        f'phone_{datetime.now().strftime("%Y%m%d_%H%M%S")}',
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
                # mocap mid-exposure time converted through the NatNet
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

        # ── rosbag recording ────────────────────────────────────────────────
        # ONE -e regex listing exactly the session bodies — nothing else
        # (no /tf, /rosout, ...). Do NOT pass a topic list alongside -e —
        # on Humble that combination silently subscribes to nothing (empty
        # bags). A topic only appears (and gets subscribed) once Motive
        # actually TRACKS that body — an absent topic means the body's
        # markers aren't visible/solving in Motive right now.
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--output', bag_dir,
                 '-e', '^(' + '|'.join(
                     '/optitrack/' + b for b in
                     os.environ.get(
                         'RECORD_BODIES',
                         'phone phone_mount popillia_mount').split()) + ')$'],
            output='screen',
        ),
    ])
