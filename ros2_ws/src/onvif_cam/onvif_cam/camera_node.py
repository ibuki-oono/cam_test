"""Low-latency ONVIF/RTSP camera node.

Publishes:
  ~/image_raw             (sensor_msgs/Image, bgr8)          - lazy (only when subscribed)
  ~/image_raw/compressed  (sensor_msgs/CompressedImage, jpeg) - lazy (only when subscribed)
Subscribes:
  ~/zoom                  (std_msgs/Float32)  ズーム速度 -1.0(広角)..1.0(望遠), 0 で停止。
                          最後の指令から zoom_timeout 秒経つと自動停止する。
"""
import http.client
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32

try:
    from onvif_cam._fast_rtsp import FastRtspCapture
except ImportError:
    FastRtspCapture = None

SOAP_ENV = (
    '<?xml version="1.0"?><s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope" '
    'xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl" xmlns:tt="http://www.onvif.org/ver10/schema">'
    '<s:Body>{}</s:Body></s:Envelope>'
)


class OnvifCamNode(Node):
    def __init__(self):
        super().__init__('onvif_cam')
        self.host = self.declare_parameter('host', '192.168.1.123').value
        # stream0 = 2592x1944 (main), stream1 = 704x576 (sub)
        stream = self.declare_parameter('stream', 'stream1').value
        pw_md5 = self.declare_parameter(
            'password_md5', 'E10ADC3949BA59ABBE56E057F20F883E').value  # md5("123456")
        self.rtsp_url = self.declare_parameter(
            'rtsp_url',
            f'rtsp://{self.host}:554/{stream}?username=admin&password={pw_md5}').value
        self.rtsp_transport = self.declare_parameter('rtsp_transport', 'tcp').value
        self.profile = self.declare_parameter('profile', 'MainStream').value
        self.frame_id = self.declare_parameter('frame_id', 'camera').value
        self.zoom_timeout = float(self.declare_parameter('zoom_timeout', 0.5).value)
        # 配信レート [Hz]。<= 0 でカメラの全フレーム (約25fps) を低遅延配信。
        rate = float(self.declare_parameter('publish_rate', 0.0).value)
        self.publish_period = 1.0 / rate if rate > 0 else 0.0
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 80).value)
        self.last_pub = 0.0

        # RELIABLE publisher は BEST_EFFORT / RELIABLE どちらの subscriber とも接続できる。
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_raw = self.create_publisher(Image, '~/image_raw', qos)
        self.pub_compressed = self.create_publisher(
            CompressedImage, '~/image_raw/compressed', qos)
        self.create_subscription(Float32, '~/zoom', self.on_zoom, 10)

        self.zoom_speed = 0.0
        self.last_zoom_cmd = 0.0
        self.create_timer(0.1, self.zoom_watchdog)

        self.running = True

        # Capture -> Publish latest-frame-only handoff (prevents RTSP queue buildup)
        self._frame_lock = threading.Lock()
        self._frame_event = threading.Event()
        self._pending_frame = None
        self._active_cap = None
        self._cap_lock = threading.Lock()

        # Async PTZ worker (keeps ROS callback non-blocking + reuses HTTP connection)
        self._ptz_lock = threading.Lock()
        self._ptz_event = threading.Event()
        self._pending_ptz_speed = None
        self._http_conn = None

        self.pub_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.ptz_thread = threading.Thread(target=self.ptz_loop, daemon=True)
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.pub_thread.start()
        self.ptz_thread.start()
        self.capture_thread.start()

    # ---------------- video ----------------
    def _has_subscribers(self) -> bool:
        return (
            self.pub_raw.get_subscription_count() > 0
            or self.pub_compressed.get_subscription_count() > 0
        )

    def _offer_frame(self, stamp, buf, width: int, height: int):
        with self._frame_lock:
            self._pending_frame = (stamp, buf, width, height)
        self._frame_event.set()

    def capture_loop(self):
        if FastRtspCapture is not None:
            self.get_logger().info(
                f'Using low-latency FFmpeg decoder (transport={self.rtsp_transport})')
            self._capture_loop_fast()
        else:
            self.get_logger().warn(
                '_fast_rtsp extension unavailable; falling back to OpenCV CAP_FFMPEG')
            self._capture_loop_opencv()

    def _capture_loop_fast(self):
        while self.running and rclpy.ok():
            try:
                cap = FastRtspCapture(
                    self.rtsp_url, transport=self.rtsp_transport, timeout_ms=3000)
            except Exception as e:  # noqa: BLE001
                if not self.running:
                    break
                self.get_logger().warn(f'RTSP open failed: {e}, retrying')
                time.sleep(1.5)
                continue

            with self._cap_lock:
                self._active_cap = cap
            self.get_logger().info('RTSP stream opened')

            try:
                while self.running and rclpy.ok():
                    now = time.monotonic()
                    need_frame = (
                        self._has_subscribers()
                        and (self.publish_period <= 0.0 or (now - self.last_pub) >= self.publish_period)
                    )
                    ok, buf, width, height = cap.read(convert=need_frame)
                    if not ok:
                        if self.running:
                            self.get_logger().warn('frame read failed, reconnecting')
                        break
                    if need_frame and buf is not None:
                        self.last_pub = time.monotonic()
                        stamp = self.get_clock().now().to_msg()
                        self._offer_frame(stamp, buf, width, height)
            finally:
                with self._cap_lock:
                    self._active_cap = None
                cap.close()

    def _capture_loop_opencv(self):
        while self.running and rclpy.ok():
            cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.isOpened():
                self.get_logger().warn(f'RTSP open failed: {self.rtsp_url}, retrying')
                time.sleep(2.0)
                continue
            self.get_logger().info('RTSP stream opened (OpenCV fallback)')
            try:
                while self.running and rclpy.ok():
                    now = time.monotonic()
                    need_frame = (
                        self._has_subscribers()
                        and (self.publish_period <= 0.0 or (now - self.last_pub) >= self.publish_period)
                    )
                    if not need_frame:
                        if not cap.grab():
                            self.get_logger().warn('frame grab failed, reconnecting')
                            break
                        continue
                    ok, frame = cap.read()
                    if not ok:
                        self.get_logger().warn('frame read failed, reconnecting')
                        break
                    self.last_pub = time.monotonic()
                    stamp = self.get_clock().now().to_msg()
                    h, w = frame.shape[:2]
                    self._offer_frame(stamp, frame, w, h)
            finally:
                cap.release()

    def publish_loop(self):
        jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        while self.running and rclpy.ok():
            if not self._frame_event.wait(timeout=0.2):
                continue
            self._frame_event.clear()
            with self._frame_lock:
                item = self._pending_frame
                self._pending_frame = None
            if item is None:
                continue

            stamp, buf, width, height = item
            want_comp = self.pub_compressed.get_subscription_count() > 0
            want_raw = self.pub_raw.get_subscription_count() > 0
            if not want_comp and not want_raw:
                continue

            try:
                if want_comp:
                    if isinstance(buf, np.ndarray):
                        frame = buf
                    else:
                        frame = np.frombuffer(buf, dtype=np.uint8).reshape((height, width, 3))
                    ok, enc = cv2.imencode('.jpg', frame, jpeg_params)
                    if ok:
                        cmsg = CompressedImage()
                        cmsg.header.stamp = stamp
                        cmsg.header.frame_id = self.frame_id
                        cmsg.format = 'jpeg'
                        cmsg.data = enc.tobytes()
                        self.pub_compressed.publish(cmsg)

                if want_raw:
                    msg = Image()
                    msg.header.stamp = stamp
                    msg.header.frame_id = self.frame_id
                    msg.height = height
                    msg.width = width
                    msg.encoding = 'bgr8'
                    msg.step = width * 3
                    msg.data = buf.tobytes() if isinstance(buf, np.ndarray) else buf
                    self.pub_raw.publish(msg)
            except Exception:  # noqa: BLE001  (node is shutting down)
                if not self.running:
                    break

    # ---------------- zoom (ONVIF PTZ) ----------------
    def _close_http(self):
        if self._http_conn is not None:
            try:
                self._http_conn.close()
            except Exception:  # noqa: BLE001
                pass
            self._http_conn = None

    def soap(self, body: str):
        payload = SOAP_ENV.format(body).encode()
        headers = {
            'Content-Type': 'application/soap+xml',
            'Connection': 'keep-alive',
        }
        for attempt in range(2):
            try:
                if self._http_conn is None:
                    self._http_conn = http.client.HTTPConnection(self.host, 80, timeout=2.0)
                self._http_conn.request('POST', '/onvif/ptz', body=payload, headers=headers)
                resp = self._http_conn.getresponse()
                resp.read()
                return
            except Exception as e:  # noqa: BLE001
                self._close_http()
                if attempt == 1:
                    self.get_logger().warn(f'PTZ request failed: {e}')

    def send_zoom(self, speed: float):
        if speed == 0.0:
            self.soap(
                f'<tptz:Stop><tptz:ProfileToken>{self.profile}</tptz:ProfileToken>'
                '<tptz:PanTilt>true</tptz:PanTilt><tptz:Zoom>true</tptz:Zoom></tptz:Stop>'
            )
        else:
            self.soap(
                f'<tptz:ContinuousMove><tptz:ProfileToken>{self.profile}</tptz:ProfileToken>'
                f'<tptz:Velocity><tt:Zoom x="{speed:.3f}"/></tptz:Velocity></tptz:ContinuousMove>'
            )

    def _enqueue_ptz(self, speed: float):
        with self._ptz_lock:
            self._pending_ptz_speed = speed
        self._ptz_event.set()

    def ptz_loop(self):
        while self.running:
            if not self._ptz_event.wait(timeout=0.2):
                continue
            self._ptz_event.clear()
            with self._ptz_lock:
                speed = self._pending_ptz_speed
                self._pending_ptz_speed = None
            if speed is not None:
                self.send_zoom(speed)
        self._close_http()

    def on_zoom(self, msg: Float32):
        speed = max(-1.0, min(1.0, float(msg.data)))
        if abs(speed) < 0.05:
            speed = 0.0
        self.last_zoom_cmd = time.monotonic()
        if speed != self.zoom_speed:
            self.zoom_speed = speed
            self._enqueue_ptz(speed)

    def zoom_watchdog(self):
        if self.zoom_speed != 0.0 and time.monotonic() - self.last_zoom_cmd > self.zoom_timeout:
            self.zoom_speed = 0.0
            self._enqueue_ptz(0.0)

    def destroy_node(self):
        self.running = False
        with self._cap_lock:
            if self._active_cap is not None:
                self._active_cap.abort()
        self._frame_event.set()
        self._ptz_event.set()
        self.capture_thread.join(timeout=2.0)
        self.pub_thread.join(timeout=1.0)
        self.ptz_thread.join(timeout=1.0)
        if self.zoom_speed != 0.0:
            self.send_zoom(0.0)
            self._close_http()
        super().destroy_node()


def main():
    rclpy.init()
    node = OnvifCamNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
