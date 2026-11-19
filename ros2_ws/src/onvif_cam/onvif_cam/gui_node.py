"""PyQt5 GUI for viewing /onvif_cam/image_raw and controlling /onvif_cam/zoom."""
import signal
import sys
import threading
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


def ros_image_to_qimage(msg: Image) -> QtGui.QImage:
    w, h = msg.width, msg.height
    enc = msg.encoding.lower()
    raw = bytes(msg.data)

    if enc == 'bgr8':
        if hasattr(QtGui.QImage, 'Format_BGR888'):
            return QtGui.QImage(raw, w, h, msg.step, QtGui.QImage.Format_BGR888).copy()
        arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, msg.step // 3, 3))[:, :w, ::-1]
        rgb = np.ascontiguousarray(arr)
        return QtGui.QImage(rgb.data, w, h, w * 3, QtGui.QImage.Format_RGB888).copy()
    if enc == 'rgb8':
        return QtGui.QImage(raw, w, h, msg.step, QtGui.QImage.Format_RGB888).copy()
    if enc == 'mono8':
        return QtGui.QImage(raw, w, h, msg.step, QtGui.QImage.Format_Grayscale8).copy()
    if enc == 'rgba8':
        return QtGui.QImage(raw, w, h, msg.step, QtGui.QImage.Format_RGBA8888).copy()
    if enc == 'bgra8':
        arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, msg.step // 4, 4))[:, :w, [2, 1, 0, 3]]
        rgba = np.ascontiguousarray(arr)
        return QtGui.QImage(rgba.data, w, h, w * 4, QtGui.QImage.Format_RGBA8888).copy()
    raise ValueError(f'Unsupported image encoding: {msg.encoding}')


class RosBridge(QtCore.QObject):
    frame_received = QtCore.pyqtSignal(QtGui.QImage, int, int, str)

    def __init__(self, node: Node, image_topic: str, zoom_topic: str):
        super().__init__()
        self.node = node
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = node.create_subscription(Image, image_topic, self._on_image, qos)
        self.pub = node.create_publisher(Float32, zoom_topic, 10)
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()

    def _spin_loop(self):
        while self._running and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            except Exception:  # noqa: BLE001
                break

    def _on_image(self, msg: Image):
        try:
            qimg = ros_image_to_qimage(msg)
        except Exception as e:  # noqa: BLE001
            self.node.get_logger().warn(f'Image decode failed: {e}')
            return
        self.frame_received.emit(qimg, msg.width, msg.height, msg.encoding)

    def publish_zoom(self, speed: float):
        if not self._running or not rclpy.ok():
            return
        msg = Float32()
        msg.data = float(speed)
        with self._lock:
            self.pub.publish(msg)

    def stop(self):
        self.publish_zoom(0.0)
        self._running = False
        self._thread.join(timeout=1.0)


class CamGuiWindow(QtWidgets.QMainWindow):
    def __init__(self, bridge: RosBridge, image_topic: str, zoom_topic: str):
        super().__init__()
        self.bridge = bridge
        self.image_topic = image_topic
        self.zoom_topic = zoom_topic

        self.setWindowTitle(f'ONVIF Camera Viewer ({image_topic})')
        self.resize(840, 680)

        self._pixmap = None
        self._last_frame_time = None
        self._fps = 0.0
        self._frame_count = 0
        self._img_info = 'No image yet'

        self._zoom_dir = 0  # -1: out, 0: stop, +1: in
        self._zoom_speed = 0.5

        self._build_ui()

        self.bridge.frame_received.connect(self._on_frame)

        # Keep publishing zoom while button/key is held (camera_node watchdog default is 0.5s)
        self._zoom_timer = QtCore.QTimer(self)
        self._zoom_timer.setInterval(150)
        self._zoom_timer.timeout.connect(self._send_active_zoom)

        # Update status indicator (e.g., stale frame detection)
        self._status_timer = QtCore.QTimer(self)
        self._status_timer.setInterval(500)
        self._status_timer.timeout.connect(self._update_status_label)
        self._status_timer.start()

    def _build_ui(self):
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        self.video_label = QtWidgets.QLabel(f'Waiting for {self.image_topic} ...')
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setStyleSheet(
            'background-color: #111; color: #bbb; font-size: 16px; border: 1px solid #333;'
        )
        self.video_label.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding
        )
        layout.addWidget(self.video_label, stretch=1)

        # Zoom controls
        ctrl_box = QtWidgets.QGroupBox(f'Zoom Control ({self.zoom_topic})')
        ctrl_layout = QtWidgets.QHBoxLayout(ctrl_box)

        self.btn_out = QtWidgets.QPushButton('Zoom Out (-)')
        self.btn_stop = QtWidgets.QPushButton('Stop (0)')
        self.btn_in = QtWidgets.QPushButton('Zoom In (+)')

        for btn in (self.btn_out, self.btn_stop, self.btn_in):
            btn.setMinimumHeight(44)
            btn.setFocusPolicy(QtCore.Qt.NoFocus)

        self.btn_out.pressed.connect(lambda: self._start_zoom(-1))
        self.btn_out.released.connect(self._stop_zoom)
        self.btn_in.pressed.connect(lambda: self._start_zoom(1))
        self.btn_in.released.connect(self._stop_zoom)
        self.btn_stop.clicked.connect(self._stop_zoom)

        ctrl_layout.addWidget(self.btn_out)
        ctrl_layout.addWidget(self.btn_stop)
        ctrl_layout.addWidget(self.btn_in)

        ctrl_layout.addSpacing(16)
        ctrl_layout.addWidget(QtWidgets.QLabel('Speed:'))

        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(10, 100)
        self.speed_slider.setValue(int(self._zoom_speed * 100))
        self.speed_slider.setFixedWidth(160)
        self.speed_slider.setFocusPolicy(QtCore.Qt.NoFocus)
        self.speed_slider.valueChanged.connect(self._on_speed_changed)
        ctrl_layout.addWidget(self.speed_slider)

        self.speed_label = QtWidgets.QLabel(f'{self._zoom_speed:.2f}')
        self.speed_label.setFixedWidth(40)
        ctrl_layout.addWidget(self.speed_label)

        layout.addWidget(ctrl_box)

        hint = QtWidgets.QLabel(
            'Hold button or keys: [Up / W / +] Zoom In, [Down / S / -] Zoom Out, [Space] Stop'
        )
        hint.setStyleSheet('color: #666; font-size: 12px;')
        layout.addWidget(hint)

        self.status_label = QtWidgets.QLabel()
        self.statusBar().addWidget(self.status_label, 1)
        self._update_status_label()

    def _on_frame(self, qimg: QtGui.QImage, width: int, height: int, encoding: str):
        now = time.monotonic()
        if self._last_frame_time is not None:
            dt = now - self._last_frame_time
            if dt > 0:
                inst_fps = 1.0 / dt
                self._fps = inst_fps if self._fps == 0.0 else (0.7 * self._fps + 0.3 * inst_fps)
        self._last_frame_time = now
        self._frame_count += 1
        self._img_info = f'{width}x{height} ({encoding})'
        self._pixmap = QtGui.QPixmap.fromImage(qimg)
        self._refresh_pixmap()
        self._update_status_label()

    def _refresh_pixmap(self):
        if self._pixmap is None:
            return
        scaled = self._pixmap.scaled(
            self.video_label.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        self.video_label.setPixmap(scaled)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh_pixmap()

    def _on_speed_changed(self, value: int):
        self._zoom_speed = value / 100.0
        self.speed_label.setText(f'{self._zoom_speed:.2f}')
        if self._zoom_dir != 0:
            self._send_active_zoom()
        self._update_status_label()

    def _start_zoom(self, direction: int):
        self._zoom_dir = direction
        self._send_active_zoom()
        if not self._zoom_timer.isActive():
            self._zoom_timer.start()
        self._update_status_label()

    def _send_active_zoom(self):
        if self._zoom_dir != 0:
            self.bridge.publish_zoom(self._zoom_dir * self._zoom_speed)

    def _stop_zoom(self):
        self._zoom_timer.stop()
        self._zoom_dir = 0
        self.bridge.publish_zoom(0.0)
        self._update_status_label()

    def _update_status_label(self):
        if self._last_frame_time is None:
            stream_state = 'Waiting for frames...'
        else:
            age = time.monotonic() - self._last_frame_time
            if age > 3.0:
                stream_state = f'Stale ({age:.1f}s ago) | {self._img_info}'
            else:
                stream_state = f'{self._img_info} | {self._fps:.1f} FPS | frames: {self._frame_count}'

        if self._zoom_dir > 0:
            zoom_state = f'Zoom IN (+{self._zoom_speed:.2f})'
        elif self._zoom_dir < 0:
            zoom_state = f'Zoom OUT (-{self._zoom_speed:.2f})'
        else:
            zoom_state = 'Zoom STOP (0.00)'

        self.status_label.setText(f'{stream_state}   |   {zoom_state}')

    def keyPressEvent(self, event: QtGui.QKeyEvent):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in (QtCore.Qt.Key_Up, QtCore.Qt.Key_W, QtCore.Qt.Key_Plus, QtCore.Qt.Key_Equal):
            self._start_zoom(1)
        elif key in (QtCore.Qt.Key_Down, QtCore.Qt.Key_S, QtCore.Qt.Key_Minus):
            self._start_zoom(-1)
        elif key == QtCore.Qt.Key_Space:
            self._stop_zoom()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QtGui.QKeyEvent):
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in (
            QtCore.Qt.Key_Up, QtCore.Qt.Key_W, QtCore.Qt.Key_Plus, QtCore.Qt.Key_Equal,
            QtCore.Qt.Key_Down, QtCore.Qt.Key_S, QtCore.Qt.Key_Minus,
        ):
            self._stop_zoom()
        else:
            super().keyReleaseEvent(event)

    def changeEvent(self, event):
        if event.type() == QtCore.QEvent.WindowDeactivate and self._zoom_dir != 0:
            self._stop_zoom()
        super().changeEvent(event)

    def closeEvent(self, event):
        self._stop_zoom()
        super().closeEvent(event)


def main():
    rclpy.init()
    node = Node('onvif_cam_gui')
    image_topic = node.declare_parameter('image_topic', '/onvif_cam/image_raw').value
    zoom_topic = node.declare_parameter('zoom_topic', '/onvif_cam/zoom').value

    app = QtWidgets.QApplication(sys.argv)
    signal.signal(signal.SIGINT, lambda *_: app.quit())

    # Allow Python to process Ctrl+C while Qt event loop is running
    sig_timer = QtCore.QTimer()
    sig_timer.start(200)
    sig_timer.timeout.connect(lambda: None)

    bridge = RosBridge(node, image_topic, zoom_topic)
    win = CamGuiWindow(bridge, image_topic, zoom_topic)
    win.show()

    try:
        app.exec_()
    finally:
        bridge.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
