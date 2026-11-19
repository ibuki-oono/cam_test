# cam_test

ONVIF/RTSP IPカメラ (192.168.1.123, MCL12D2_AF, 電動ズーム) の ROS 2 ドライバ。ROS 2 Humble / Jazzy 用。

## ビルドと起動

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run onvif_cam camera_node
```

## トピック

| トピック | 型 | 内容 |
|---|---|---|
| `/onvif_cam/image_raw` | `sensor_msgs/Image` (bgr8) | カメラ映像 (QoS: RELIABLE, depth 1) |
| `/onvif_cam/zoom` | `std_msgs/Float32` | ズーム速度 -1.0 (広角) 〜 1.0 (望遠)、0 で停止。最後の指令から `zoom_timeout` 秒で自動停止 |

## パラメータ

| 名前 | 初期値 | 内容 |
|---|---|---|
| `host` | `192.168.1.123` | カメラの IP |
| `stream` | `stream1` | `stream1` = 704x576, `stream0` = 2592x1944 |
| `publish_rate` | `2.0` | 配信レート [Hz]。0 以下でカメラの全フレーム (約25fps) |
| `zoom_timeout` | `0.5` | ズーム自動停止までの秒数 |
| `frame_id` | `camera` | 画像の frame_id |

例: `ros2 run onvif_cam camera_node --ros-args -p publish_rate:=5.0 -p stream:=stream0`

## カメラについてのメモ

- ONVIF は認証なしで応答する。RTSP の認証は初期値 admin / 123456 (MD5 で URL に入る)。
- ズームは ContinuousMove + Stop のみ対応。AbsoluteMove を送ると接続を切られる。
- 現在の倍率は ONVIF からは取れない (常に 0) が、映像右下に `KF 17X` のように表示される (1X〜)。
- `zoom.py` は ROS なしでズームを操作する単体スクリプト (`python zoom.py in 1.0 0.5`)。
