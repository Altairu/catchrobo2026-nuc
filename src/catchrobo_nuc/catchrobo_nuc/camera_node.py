"""
camera_node.py
USBカメラとRealSenseカメラにアクセスし、それぞれの映像をモニターに表示するROS2ノード

表示構成:
  - USBカメラ: RGBカラー映像を表示
  - RealSense: Depth(デプス/距離)画像のみを表示
"""

import sys
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

# pyrealsense2 のインポート試行
try:
    import pyrealsense2 as rs
    HAS_REALSENSE = True
except ImportError:
    HAS_REALSENSE = False


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # ─── ROS2 パラメータ宣言 ───────────────────────────
        self.declare_parameter('usb_device_index', -1)  # -1 の場合は自動検出
        self.declare_parameter('enable_usb_camera', True)
        self.declare_parameter('enable_realsense', True)
        self.declare_parameter('show_color', False)     # RealSenseのカラー画像表示 (デフォルト False)
        self.declare_parameter('show_depth', True)      # RealSenseのDepth画像表示 (デフォルト True)
        self.declare_parameter('fps', 15)                # 表示・ループ処理の更新FPS
        self.declare_parameter('width', 640)             # カメラ取得幅
        self.declare_parameter('height', 480)            # カメラ取得高さ
        self.declare_parameter('window_width', 640)      # 初期ウィンドウ幅
        self.declare_parameter('window_height', 480)     # 初期ウィンドウ高さ

        # パラメータ取得
        self.usb_device_index = self.get_parameter('usb_device_index').value
        self.enable_usb_camera = self.get_parameter('enable_usb_camera').value
        self.enable_realsense = self.get_parameter('enable_realsense').value
        self.show_color = self.get_parameter('show_color').value
        self.show_depth = self.get_parameter('show_depth').value
        self.target_fps = self.get_parameter('fps').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.window_width = self.get_parameter('window_width').value
        self.window_height = self.get_parameter('window_height').value

        # ─── ウィンドウ初期化 (サイズ変更可能 cv2.WINDOW_NORMAL) ────
        self._init_windows()

        # ─── カメラ初期化 ───────────────────────────
        self.cap_usb = None
        self.rs_pipeline = None
        self.rs_colorizer = None

        if self.enable_usb_camera:
            self._init_usb_camera()

        if self.enable_realsense:
            self._init_realsense()

        # ─── タイマー設定 ───────────────────────────
        timer_period = 1.0 / float(self.target_fps)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f'カメラ表示ノードを起動しました (RealSense Depth表示, 表示FPS: {self.target_fps})')

    def _init_windows(self):
        """表示用ウィンドウの初期化 (マウスドラッグで自由サイズ変更可能)"""
        try:
            if self.enable_usb_camera:
                cv2.namedWindow('USB Camera', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('USB Camera', self.window_width, self.window_height)

            if self.enable_realsense:
                if self.show_color:
                    cv2.namedWindow('RealSense Color', cv2.WINDOW_NORMAL)
                    cv2.resizeWindow('RealSense Color', self.window_width, self.window_height)
                if self.show_depth:
                    cv2.namedWindow('RealSense Depth', cv2.WINDOW_NORMAL)
                    cv2.resizeWindow('RealSense Depth', self.window_width, self.window_height)
        except Exception as e:
            self.get_logger().warn(f'ウィンドウの初期化エラー: {e}')

    def _find_usb_camera_index(self):
        """RealSense以外の利用可能なUSBカメラのデバイスインデックスを自動検索"""
        if self.usb_device_index >= 0:
            return self.usb_device_index

        for idx in [8, 6, 0, 1, 2, 4, 10]:
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                ret, _ = cap.read()
                cap.release()
                if ret:
                    self.get_logger().info(f'USBカメラを自動検出しました: デバイスインデックス {idx}')
                    return idx
        return 0

    def _init_usb_camera(self):
        """USBカメラ (OpenCV) の初期化"""
        idx = self._find_usb_camera_index()
        self.get_logger().info(f'USBカメラ (/dev/video{idx}) にアクセス中...')
        self.cap_usb = cv2.VideoCapture(idx)

        if self.cap_usb.isOpened():
            self.cap_usb.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap_usb.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap_usb.set(cv2.CAP_PROP_FPS, self.target_fps)
            self.get_logger().info(f'USBカメラ (Index: {idx}) の接続に成功しました。')
        else:
            self.get_logger().error(f'USBカメラ (Index: {idx}) のオープンに失敗しました。')
            self.cap_usb = None

    def _init_realsense(self):
        """RealSense (pyrealsense2) の初期化"""
        if not HAS_REALSENSE:
            self.get_logger().error('pyrealsense2 ライブラリが見つかりません。RealSenseが無効化されます。')
            return

        try:
            self.rs_pipeline = rs.pipeline()
            config = rs.config()

            # RealSenseパイプライン動作を安定させるためColorとDepthの両方を定義
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)
            if self.show_depth:
                config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

            # パイプライン開始
            self.rs_pipeline.start(config)
            if self.show_depth:
                self.rs_colorizer = rs.colorizer()
            self.get_logger().info('RealSenseカメラの起動に成功しました。')
        except Exception as e:
            self.get_logger().error(f'RealSenseカメラの初期化に失敗しました: {e}')
            self.rs_pipeline = None

    def timer_callback(self):
        """カメラフレーム取得＆モニター表示ループ"""

        # 1. USBカメラの処理
        if self.cap_usb and self.cap_usb.isOpened():
            ret, frame_usb = self.cap_usb.read()
            if ret:
                cv2.putText(frame_usb, 'USB Camera', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow('USB Camera', frame_usb)
            else:
                self.get_logger().warn('USBカメラからのフレーム読み込みに失敗しました。', throttle_duration_sec=5.0)

        # 2. RealSenseカメラの処理
        if self.rs_pipeline:
            try:
                frames = self.rs_pipeline.wait_for_frames(1000)

                # Color表示フラグがTrueの場合のみ描画
                if self.show_color:
                    color_frame = frames.get_color_frame()
                    if color_frame:
                        color_image = np.asanyarray(color_frame.get_data())
                        cv2.putText(color_image, 'RealSense Color', (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
                        cv2.imshow('RealSense Color', color_image)

                # Depth表示フラグがTrueの場合のみ取得・カラーマップ表示
                if self.show_depth and self.rs_colorizer:
                    depth_frame = frames.get_depth_frame()
                    if depth_frame:
                        depth_colormap = np.asanyarray(self.rs_colorizer.colorize(depth_frame).get_data())
                        cv2.putText(depth_colormap, 'RealSense Depth', (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        cv2.imshow('RealSense Depth', depth_colormap)

            except Exception as e:
                self.get_logger().warn(f'RealSenseフレーム取得エラー: {e}', throttle_duration_sec=5.0)

        # 3. OpenCVウィンドウイベント処理 ('q' キーで表示ウィンドウ終了)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("'q' キーが押されたためカメラノードを終了します。")
            rclpy.shutdown()

    def destroy_node(self):
        """リソースの解放"""
        self.get_logger().info('カメラリソースを解放します...')
        if self.cap_usb and self.cap_usb.isOpened():
            self.cap_usb.release()
        if self.rs_pipeline:
            try:
                self.rs_pipeline.stop()
            except Exception:
                pass
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
