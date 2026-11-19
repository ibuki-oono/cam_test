"""ONVIF/RTSP camera node.

Publishes:
  ~/image_raw  (sensor_msgs/Image, bgr8)
Subscribes:
  ~/zoom       (std_msgs/Float32)  ズーム速度 -1.0(広角)..1.0(望遠), 0 で停止。
               最後の指令から zoom_timeout 秒経つと自動停止する (ジョイスティック等で送り続ける想定)。
"""
import threading
import time
import urllib.request

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

SOAP_ENV = (
    '<?xml version="1.0"?><s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope" '
    'xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl" xmlns:tt="http://www.onvif.org/ver10/schema">'
    '<s:Body>{}</s:Body></s:Envelope>'
)


class OnvifCamNode(Node):
    def __init__(self):
        super().__init__('onvif_cam')
        host = self.declare_parameter('host', '192.168.1.123').value
        # stream0 = 2592x1944 (main), stream1 = 704x576 (sub)
        stream = self.declare_parameter('stream', 'stream1').value
        pw_md5 = self.declare_parameter(
            'password_md5', 'E10ADC3949BA59ABBE56E057F20F883E').value  # md5("123456")
        self.rtsp_url = self.declare_parameter(
            'rtsp_url',
            f'rtsp://{host}:554/{stream}?username=admin&password={pw_md5}').value
        self.ptz_url = f'http://{host}/onvif/ptz'
        self.profile = self.declare_parameter('profile', 'MainStream').value
        self.frame_id = self.declare_parameter('frame_id', 'camera').value
        self.zoom_timeout = self.declare_parameter('zoom_timeout', 0.5).value
        # 配信レート [Hz]。<= 0 でカメラの全フレーム (約25fps) を配信。
        # カメラからの読み込みは常に全フレーム行い (遅延が溜まらないように)、配信だけ間引く。
        rate = float(self.declare_parameter('publish_rate', 2.0).value)
        self.publish_period = 1.0 / rate if rate > 0 else 0.0
        self.last_pub = 0.0

        # RELIABLE: 1.2MB級の非圧縮画像は BEST_EFFORT だとフラグメント欠落で大半が落ちる。
        # RELIABLE publisher は BEST_EFFORT / RELIABLE どちらの subscriber とも接続できる。
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Image, '~/image_raw', qos)
        self.create_subscription(Float32, '~/zoom', self.on_zoom, 10)

        self.zoom_speed = 0.0
        self.last_zoom_cmd = 0.0
        self.create_timer(0.1, self.zoom_watchdog)

        self.running = True
        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()

    # ---------------- video ----------------
    def capture_loop(self):
        while self.running and rclpy.ok():
            cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.isOpened():
                self.get_logger().warn(f'RTSP open failed: {self.rtsp_url}, retrying')
                time.sleep(2.0)
                continue
            self.get_logger().info('RTSP stream opened')
            while self.running:
                ok, frame = cap.read()
                if not ok:
                    self.get_logger().warn('frame read failed, reconnecting')
                    break
                now = time.monotonic()
                if now - self.last_pub < self.publish_period:
                    continue
                self.last_pub = now
                try:
                    self.publish(frame)
                except Exception:  # noqa: BLE001  (node is shutting down)
                    break
            cap.release()

    def publish(self, frame):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)

    # ---------------- zoom (ONVIF PTZ) ----------------
    def soap(self, body):
        req = urllib.request.Request(
            self.ptz_url, SOAP_ENV.format(body).encode(),
            {'Content-Type': 'application/soap+xml'})
        try:
            with urllib.request.urlopen(req, timeout=2) as r:
                r.read()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'PTZ request failed: {e}')

    def send_zoom(self, speed):
        if speed == 0.0:
            self.soap(f'<tptz:Stop><tptz:ProfileToken>{self.profile}</tptz:ProfileToken>'
                      '<tptz:PanTilt>true</tptz:PanTilt><tptz:Zoom>true</tptz:Zoom></tptz:Stop>')
        else:
            self.soap(f'<tptz:ContinuousMove><tptz:ProfileToken>{self.profile}</tptz:ProfileToken>'
                      f'<tptz:Velocity><tt:Zoom x="{speed:.3f}"/></tptz:Velocity></tptz:ContinuousMove>')

    def on_zoom(self, msg):
        speed = max(-1.0, min(1.0, float(msg.data)))
        if abs(speed) < 0.05:
            speed = 0.0
        self.last_zoom_cmd = time.monotonic()
        if speed != self.zoom_speed:
            self.zoom_speed = speed
            self.send_zoom(speed)

    def zoom_watchdog(self):
        if self.zoom_speed != 0.0 and time.monotonic() - self.last_zoom_cmd > self.zoom_timeout:
            self.zoom_speed = 0.0
            self.send_zoom(0.0)

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=2.0)
        if self.zoom_speed != 0.0:
            self.send_zoom(0.0)
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
