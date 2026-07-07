"""ROS 2 node that streams color and depth from an Orbbec Femto Mega using
pyorbbecsdk.  Publishes on the standard /camera/* topic tree so the topics
can be recorded together with the OptiTrack pose in a single rosbag.

Published topics:
  /camera/color/image_raw/compressed — sensor_msgs/CompressedImage (jpeg)
  /camera/depth/image_raw            — sensor_msgs/Image (16UC1, millimetres)
  /camera/ir/image_raw               — sensor_msgs/Image (mono16, ToF infrared)
  /camera/color/camera_info          — sensor_msgs/CameraInfo
  /camera/depth/camera_info          — sensor_msgs/CameraInfo
  /camera/ir/camera_info             — sensor_msgs/CameraInfo
  /camera/imu                        — sensor_msgs/Imu (6-DoF accel + gyro)

Color is published as the camera's native MJPEG bitstream, NOT decoded to
rgb8: decoding + publishing raw 1080p in Python managed only ~2 fps and
flooded "frameset queue fulled" warnings, while passing the JPEG through
sustains the full frame rate and keeps bags ~30x smaller. Decode happens
offline in scripts/export_dataset.py.

Timestamps: image headers carry the *capture* time, not the arrival time.
We stamp from the device hardware clock (frame.get_timestamp_us), which
timer_sync_with_host() expresses in host epoch microseconds at the capture
instant — so USB transfer latency (~90 ms, variable) is excluded. It is
rock-steady (33 ms, zero jitter). We deliberately do NOT use the SDK's global
timestamp: its continuous re-fit intermittently diverges under recording load
(observed duplicating ~80% of stamps and jumping epochs), which mis-assigns the
OptiTrack pose to each frame. Arrival time is the fallback but carries USB
transfer jitter; a residual constant offset can still be cancelled at export
with --time-offset (see scripts/export_dataset.py). See _frame_stamp.

Parameters:
  color_width   (int,   default 1920)
  color_height  (int,   default 1080)
  color_fps     (int,   default 30)
  depth_width   (int,   default 640)
  depth_height  (int,   default 576)
  depth_fps     (int,   default 30)
  frame_timeout (int,   default 100)   — ms to wait for a frameset
  mock          (bool,  default False) — publish synthetic frames, no hardware
  connection    (str,   default 'usb') — 'usb' or 'net' (Gigabit Ethernet/PoE)
  net_ip        (str,   default '')    — camera LAN IP (required for 'net')
  net_port      (int,   default 8090)  — Femto Mega network data port
  enable_ir     (bool,  default True)  — publish the ToF IR stream
  enable_imu    (bool,  default True)  — publish the 6-DoF IMU
"""

import io
import time
from array import array

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu

import numpy as np

try:
    from pyorbbecsdk import (
        Config,
        Context,
        OBFormat,
        OBFrameType,
        OBSensorType,
        Pipeline,
    )
    _SDK_OK = True
except ImportError:
    _SDK_OK = False


def _make_camera_info(intr, frame_id: str) -> CameraInfo:
    ci = CameraInfo()
    ci.header.frame_id = frame_id
    ci.width = intr.width
    ci.height = intr.height
    fx, fy = float(intr.fx), float(intr.fy)
    cx, cy = float(intr.cx), float(intr.cy)
    ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    ci.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    ci.distortion_model = 'plumb_bob'
    return ci


class OrbbecCameraNode(Node):
    def __init__(self):
        super().__init__('orbbec_camera')

        # Default to full 1080p color. Color travels compressed (MJPEG), so
        # 1920x1080@30 pairs with depth 640x576@30 (frame sync requires equal
        # fps) even over the USB 2.0 link.
        self.declare_parameter('color_width', 1920)
        self.declare_parameter('color_height', 1080)
        self.declare_parameter('color_fps', 30)
        self.declare_parameter('depth_width', 640)
        self.declare_parameter('depth_height', 576)
        self.declare_parameter('depth_fps', 30)
        self.declare_parameter('frame_timeout', 100)
        self.declare_parameter('mock', False)
        # Connection: 'usb' (default) or 'net' (Gigabit Ethernet / PoE). Net
        # avoids the USB 2.0 bandwidth ceiling and the macOS root requirement.
        self.declare_parameter('connection', 'usb')
        self.declare_parameter('net_ip', '')      # camera IP on the LAN
        self.declare_parameter('net_port', 8090)  # Femto Mega data port
        # Capture the camera's other sensors too. IR is the ToF sensor's raw
        # infrared (same resolution as depth) — it roughly doubles the ToF wire
        # bandwidth, so it can throttle fps on a saturated USB 2.0 link. IMU is
        # the 6-DoF accel+gyro (tiny bandwidth, high rate).
        self.declare_parameter('enable_ir', True)
        self.declare_parameter('enable_imu', True)
        # Hardware frame sync pairs color+depth into one frameset. It only emits
        # when the two align, which roughly halves the delivered rate — and the
        # export pipeline re-pairs by timestamp anyway, so it's off by default.
        self.declare_parameter('frame_sync', False)

        self._connection = self.get_parameter('connection').value
        self._net_ip = self.get_parameter('net_ip').value
        self._net_port = self.get_parameter('net_port').value
        self._enable_ir = self.get_parameter('enable_ir').value
        self._enable_imu = self.get_parameter('enable_imu').value
        self._frame_sync = self.get_parameter('frame_sync').value
        self._cw = self.get_parameter('color_width').value
        self._ch = self.get_parameter('color_height').value
        self._cf = self.get_parameter('color_fps').value
        self._dw = self.get_parameter('depth_width').value
        self._dh = self.get_parameter('depth_height').value
        self._df = self.get_parameter('depth_fps').value
        self._timeout = self.get_parameter('frame_timeout').value
        self._mock = self.get_parameter('mock').value

        # Shutdown coordination: _shutting_down makes the SDK frame callback a
        # no-op so pipeline.stop() can't deadlock against a mid-flight callback;
        # _stopped makes stop_camera() idempotent.
        self._shutting_down = False
        self._stopped = False

        self._color_pub = self.create_publisher(
            CompressedImage, '/camera/color/image_raw/compressed', 10)
        self._depth_pub = self.create_publisher(Image,      '/camera/depth/image_raw',   10)
        self._cinfo_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self._dinfo_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        self._ir_pub    = self.create_publisher(Image,      '/camera/ir/image_raw',      10)
        self._irinfo_pub = self.create_publisher(CameraInfo, '/camera/ir/camera_info',   10)
        self._imu_pub   = self.create_publisher(Imu,        '/camera/imu',               50)

        # Warn-once flag for the timestamp fallback (see _frame_stamp).
        self._warned_no_device_ts = False
        self._mock_t0 = None
        # Latest accel frame, paired with each gyro frame into one Imu message.
        self._last_accel = None
        self._ir_info = None

        if self._mock:
            self.get_logger().warn('MOCK MODE — publishing synthetic frames, no camera hardware used')
            self._color_info = self._mock_camera_info(self._cw, self._ch, 'camera_color_optical_frame')
            self._depth_info = self._mock_camera_info(self._dw, self._dh, 'camera_depth_optical_frame')
            self.create_timer(1.0 / self._cf, self._publish_mock_frames)
            return

        if not _SDK_OK:
            self.get_logger().fatal('pyorbbecsdk is not installed — cannot start camera node')
            raise RuntimeError('pyorbbecsdk not available')

        # USB (default) or network device. Net streams over Gigabit Ethernet,
        # so it needs no root and isn't capped by the USB 2.0 link.
        if self._connection == 'net':
            if not self._net_ip:
                self.get_logger().fatal(
                    "connection:=net requires net_ip:=<camera IP on the LAN>")
                raise RuntimeError('net_ip not set')
            self.get_logger().info(
                f'Connecting to Femto Mega over network at '
                f'{self._net_ip}:{self._net_port}')
            self._ctx = Context()
            device = self._ctx.create_net_device(self._net_ip, int(self._net_port))
            self._pipeline = Pipeline(device)
        else:
            self._pipeline = Pipeline()

        # Sync the device hardware clock with the host once at startup; the
        # SDK's GlobalTimestampFitter keeps refining the mapping afterwards.
        try:
            self._pipeline.get_device().timer_sync_with_host()
        except Exception as e:  # not fatal — global timestamps may still work
            self.get_logger().warn(f'timer_sync_with_host failed: {e}')

        config = Config()
        # MJPG: take the camera's compressed bitstream as-is (see module
        # docstring) instead of having the SDK decode to RGB on the host.
        color_prof = self._pick_profile(
            OBSensorType.COLOR_SENSOR, self._cw, self._ch, OBFormat.MJPG, self._cf, 'color')
        depth_prof = self._pick_profile(
            OBSensorType.DEPTH_SENSOR, self._dw, self._dh, OBFormat.Y16, self._df, 'depth')
        config.enable_stream(color_prof)
        config.enable_stream(depth_prof)

        # IR: the ToF sensor's raw infrared, at the depth resolution/fps.
        if self._enable_ir:
            try:
                ir_prof = self._pick_profile(
                    OBSensorType.IR_SENSOR, self._dw, self._dh, OBFormat.Y16,
                    self._df, 'ir')
                config.enable_stream(ir_prof)
            except Exception as e:
                self.get_logger().warn(f'IR stream unavailable, skipping: {e}')
                self._enable_ir = False

        # IMU: 6-DoF accel + gyro (separate high-rate streams, default profiles).
        if self._enable_imu:
            try:
                accel_pl = self._pipeline.get_stream_profile_list(
                    OBSensorType.ACCEL_SENSOR)
                gyro_pl = self._pipeline.get_stream_profile_list(
                    OBSensorType.GYRO_SENSOR)
                config.enable_stream(accel_pl.get_stream_profile_by_index(0))
                config.enable_stream(gyro_pl.get_stream_profile_by_index(0))
            except Exception as e:
                self.get_logger().warn(f'IMU streams unavailable, skipping: {e}')
                self._enable_imu = False

        # Hardware frame sync pairs color+depth into one frameset by capture
        # time, but it only emits aligned pairs — which roughly halves the
        # delivered rate — and needs EQUAL color/depth fps. Off by default:
        # export re-pairs each color frame to the nearest depth by timestamp, so
        # we stream them independently at full rate.
        if self._frame_sync and color_prof.get_fps() == depth_prof.get_fps():
            try:
                self._pipeline.enable_frame_sync()
            except Exception as e:
                self.get_logger().warn(f'enable_frame_sync failed: {e}')
        elif self._frame_sync:
            self.get_logger().warn(
                f'color {color_prof.get_fps()}fps != depth {depth_prof.get_fps()}fps'
                ' — frame sync off; color/depth paired by timestamp at export')

        # Push model: the SDK streams framesets continuously and delivers each
        # to _on_frames on its own thread. This sustains the full 30 fps the
        # device sends over USB 2.0 — pulling with wait_for_frames from a ROS
        # timer or Python loop bottlenecked at ~15 fps (the consumer couldn't
        # drain fast enough, so the SDK dropped every other frame).
        # camera_info depends on get_camera_param(), only valid after start(),
        # so the callback guards on _color_info until we set it just below.
        self._color_info = None
        self._depth_info = None
        self._pipeline.start(config, self._on_frames)
        self._cam_param = self._pipeline.get_camera_param()
        self._color_info = _make_camera_info(self._cam_param.rgb_intrinsic, 'camera_color_optical_frame')
        self._depth_info = _make_camera_info(self._cam_param.depth_intrinsic, 'camera_depth_optical_frame')
        # IR shares the ToF sensor with depth, so reuse the depth intrinsics.
        self._ir_info = _make_camera_info(self._cam_param.depth_intrinsic, 'camera_ir_optical_frame')
        self.get_logger().info(
            f'Orbbec camera started — '
            f'color {self._cw}x{self._ch}@{self._cf}fps  depth {self._dw}x{self._dh}@{self._df}fps'
            f'  ir={"on" if self._enable_ir else "off"}'
            f'  imu={"on" if self._enable_imu else "off"}'
        )

    def _pick_profile(self, sensor_type, w, h, fmt, fps, label):
        """Best matching video profile.

        Exact (w, h, fmt, fps) if available; otherwise the same resolution and
        format at the highest fps <= requested (the sensor may not offer the
        requested fps — e.g. depth WFOV unbinned 1024x1024 caps at 15). Falls
        back to the sensor default only if the resolution/format is unavailable.
        """
        profiles = self._pipeline.get_stream_profile_list(sensor_type)
        try:
            return profiles.get_video_stream_profile(w, h, fmt, fps)
        except Exception:
            pass
        same_res = []
        for i in range(profiles.get_count()):
            sp = profiles.get_stream_profile_by_index(i).as_video_stream_profile()
            if sp.get_width() == w and sp.get_height() == h and sp.get_format() == fmt:
                same_res.append(sp)
        if same_res:
            at_or_below = [c for c in same_res if c.get_fps() <= fps]
            best = (max(at_or_below, key=lambda c: c.get_fps()) if at_or_below
                    else min(same_res, key=lambda c: c.get_fps()))
            self.get_logger().warn(
                f'{label} {w}x{h}@{fps}fps unavailable; using '
                f'{w}x{h}@{best.get_fps()}fps')
            return best
        self.get_logger().warn(
            f'Requested {label} profile {w}x{h}@{fps} unavailable, using default')
        return profiles.get_default_video_stream_profile()

    # ── timestamps ──────────────────────────────────────────────────────────

    # A host-synced device clock reads epoch microseconds (~1.7e15 today).
    # Below this threshold the value is a relative/unsynced clock we must not
    # trust as a host-clock stamp.
    _EPOCH_US = 1_000_000_000_000_000

    def _frame_stamp(self, frame):
        """Capture time of a frame as a ROS stamp, in the host clock.

        Uses the device hardware timestamp (``get_timestamp_us``). After
        ``timer_sync_with_host()`` at startup the device clock is expressed in
        host epoch microseconds and marks the *capture* instant — so the USB
        transfer latency (measured here at ~90 ms, and variable) is already
        excluded, exactly what we want for lining frames up with OptiTrack.

        We deliberately avoid the SDK global timestamp: its continuous re-fit
        intermittently duplicates stamps / jumps epochs under recording load
        (observed mis-stamping ~80% of frames). We also avoid arrival time,
        which carries the transfer jitter. If the device clock isn't host-synced
        (value not in epoch range) we fall back to arrival, then the node clock;
        a residual constant offset can be cancelled at export with --time-offset.
        """
        try:
            dev_us = int(frame.get_timestamp_us())
        except Exception:
            dev_us = 0
        if dev_us > self._EPOCH_US:
            return Time(nanoseconds=dev_us * 1000).to_msg()

        if not self._warned_no_device_ts:
            self._warned_no_device_ts = True
            self.get_logger().warn(
                'Device capture timestamp not host-synced — falling back to '
                'arrival time. Stamps include USB transfer jitter; correct a '
                'residual offset at export with --time-offset.')
        try:
            arr_us = int(frame.get_system_timestamp_us())
        except Exception:
            arr_us = 0
        if arr_us > self._EPOCH_US:
            return Time(nanoseconds=arr_us * 1000).to_msg()
        return self.get_clock().now().to_msg()

    # ── real camera ─────────────────────────────────────────────────────────

    def _on_frames(self, frameset):
        """SDK push callback — the continuous frame stream (see __init__).

        Runs on an SDK-owned thread. An exception must never escape here: it
        would propagate into the C++ caller and abort the process (SIGABRT).
        In particular, on SIGINT rclpy invalidates the publisher context while
        frames may still be arriving, so publish() raises RCLError — catch it.
        """
        if (self._shutting_down or not rclpy.ok()
                or frameset is None or self._color_info is None):
            return  # tearing down, or not yet ready (camera_info set after start())
        try:
            self._publish_frameset(frameset)
        except Exception as e:  # noqa: BLE001 — must not let it reach C++
            if rclpy.ok():
                self.get_logger().warn(f'frame publish failed: {e}')
            # else: context torn down mid-callback during shutdown — expected

    def _publish_frameset(self, frameset):
        color_frame = frameset.get_color_frame()
        if color_frame is not None:
            stamp = self._frame_stamp(color_frame)
            # Native MJPEG bitstream — published as-is, decoded at export
            self._publish_color_jpeg(bytes(color_frame.get_data()), stamp)

        depth_frame = frameset.get_depth_frame()
        if depth_frame is not None:
            stamp = self._frame_stamp(depth_frame)
            w, h = depth_frame.get_width(), depth_frame.get_height()
            scale = depth_frame.get_depth_scale()
            data = np.frombuffer(bytes(depth_frame.get_data()), dtype=np.uint16)
            # Convert to millimetres (standard ROS depth encoding)
            if scale != 1.0:
                data = (data.astype(np.float32) * scale).astype(np.uint16)
            if data.size == w * h:
                self._publish_depth(data, w, h, stamp)

        if self._enable_ir:
            ir_frame = frameset.get_ir_frame()
            if ir_frame is not None:
                self._publish_ir(ir_frame)

        if self._enable_imu:
            # IMU frames arrive in their own framesets at the IMU rate. Pair the
            # latest accel with each gyro into one Imu message.
            accel = frameset.get_frame(OBFrameType.ACCEL_FRAME)
            if accel is not None:
                self._last_accel = accel.as_accel_frame()
            gyro = frameset.get_frame(OBFrameType.GYRO_FRAME)
            if gyro is not None:
                self._publish_imu(gyro.as_gyro_frame())

    # ── shared publishing ────────────────────────────────────────────────────

    def _publish_ir(self, frame):
        w, h = frame.get_width(), frame.get_height()
        data = np.frombuffer(bytes(frame.get_data()), dtype=np.uint16)
        if data.size != w * h:
            return
        stamp = self._frame_stamp(frame)
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = 'camera_ir_optical_frame'
        img.width, img.height = w, h
        img.encoding = 'mono16'
        img.step = w * 2
        img.data = array('B', data.tobytes())  # fast path, see _publish_color_jpeg
        self._ir_pub.publish(img)
        self._ir_info.header.stamp = stamp
        self._irinfo_pub.publish(self._ir_info)

    def _publish_imu(self, gyro):
        msg = Imu()
        msg.header.stamp = self._frame_stamp(gyro)
        msg.header.frame_id = 'camera_imu_frame'
        # No absolute orientation from the raw IMU: flag as unavailable.
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = float(gyro.get_x())
        msg.angular_velocity.y = float(gyro.get_y())
        msg.angular_velocity.z = float(gyro.get_z())
        if self._last_accel is not None:
            a = self._last_accel
            msg.linear_acceleration.x = float(a.get_x())
            msg.linear_acceleration.y = float(a.get_y())
            msg.linear_acceleration.z = float(a.get_z())
        self._imu_pub.publish(msg)

    def _publish_color_jpeg(self, jpeg: bytes, stamp):
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.format = 'jpeg'
        # array('B') hits the generated setter's fast path; assigning bytes
        # triggers a per-byte pure-Python validation loop (~60 ms on a 2 MiB
        # frame) that pegged a core and made the SDK drop framesets.
        msg.data = array('B', jpeg)
        self._color_pub.publish(msg)
        self._color_info.header.stamp = stamp
        self._cinfo_pub.publish(self._color_info)

    def _publish_depth(self, data: np.ndarray, w: int, h: int, stamp):
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = 'camera_depth_optical_frame'
        img.width, img.height = w, h
        img.encoding = '16UC1'
        img.step = w * 2
        img.data = array('B', data.tobytes())  # fast path, see _publish_color_jpeg
        self._depth_pub.publish(img)
        self._depth_info.header.stamp = stamp
        self._dinfo_pub.publish(self._depth_info)

    # ── mock mode ────────────────────────────────────────────────────────────

    @staticmethod
    def _mock_camera_info(w: int, h: int, frame_id: str) -> CameraInfo:
        ci = CameraInfo()
        ci.header.frame_id = frame_id
        ci.width, ci.height = w, h
        f = float(max(w, h))  # plausible focal length
        cx, cy = w / 2.0, h / 2.0
        ci.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        ci.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.distortion_model = 'plumb_bob'
        return ci

    def _publish_mock_frames(self):
        now = self.get_clock().now()
        if self._mock_t0 is None:
            self._mock_t0 = now
        t = (now - self._mock_t0).nanoseconds * 1e-9
        stamp = now.to_msg()

        # Color: horizontal gradient sweeping over time so frames differ
        x = np.linspace(0, 255, self._cw, dtype=np.float32)
        row = ((x + t * 60.0) % 256).astype(np.uint8)
        color = np.zeros((self._ch, self._cw, 3), dtype=np.uint8)
        color[:, :, 0] = row
        color[:, :, 1] = row[::-1]
        color[:, :, 2] = int(t * 30) % 256
        from PIL import Image as PILImage
        buf = io.BytesIO()
        PILImage.fromarray(color, mode='RGB').save(buf, format='JPEG')
        self._publish_color_jpeg(buf.getvalue(), stamp)

        # Depth: radial ramp in millimetres
        yy, xx = np.mgrid[0:self._dh, 0:self._dw]
        depth = (1000 + 2 * np.hypot(xx - self._dw / 2, yy - self._dh / 2)).astype(np.uint16)
        self._publish_depth(depth.reshape(-1), self._dw, self._dh, stamp)

    def stop_camera(self):
        """Cleanly stop the SDK pipeline exactly once.

        Ordering matters: flip _shutting_down and let any in-flight _on_frames
        callback return before calling pipeline.stop(), otherwise stop() can
        deadlock against the callback thread's lock (the 'recursive_mutex'
        abort). Letting stop() run to completion is what tells the camera
        firmware to halt its video streams and releases the USB device —
        skipping it (e.g. a hard os._exit) leaves the streams wedged, so the
        next run enumerates the device but receives no color/depth frames.
        """
        if self._mock or self._stopped:
            return
        self._stopped = True
        self._shutting_down = True
        time.sleep(0.15)  # let a mid-flight frame callback drain
        try:
            self._pipeline.stop()
        except Exception as e:
            self.get_logger().warn(f'pipeline.stop() failed: {e}')

    def destroy_node(self):
        self.stop_camera()
        super().destroy_node()


def main():
    import os
    rclpy.init()
    node = None
    try:
        node = OrbbecCameraNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        # Order is the whole game here:
        #   1. stop_camera() SYNCHRONOUSLY halts the video streams (sends the
        #      SDK stop command). Completing this is what keeps the next run
        #      from inheriting a device still streaming into a dead process.
        #   2. os._exit() only AFTER that completes. The SDK leaves non-daemon
        #      threads that otherwise stall interpreter exit ~5 s and then abort
        #      in a static destructor (recursive_mutex). The streams are already
        #      stopped, so a hard exit is safe — the OS reclaims the USB handle.
        # The original teardown ran stop() in an abandonable 3 s daemon thread
        # and let the publish-race abort kill the process first, so the stop
        # never finished — that is what wedged the camera.
        if node is not None:
            node.stop_camera()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os._exit(0)


if __name__ == '__main__':
    main()
