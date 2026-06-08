"""
serial_motor_node.py
シリアル通信でロボマスモーターを制御するROS2ノード

サンプルコード (motor_control_gui.py) のプロトコルをROS2ノードに移植。
- 送信: 0xAA 0x55 + rm1 rm2 lm1 lm2 sm1 lm3(各int16 LE) + mode(int8) + CRC16 + 0x0A  計18バイト
- 受信: 0xBB 0x66 + angles×6(int16 LE) + rpm×2(int16 LE)  計18バイト
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

import serial
import serial.tools.list_ports
import struct
import threading
import time
import json


class SerialMotorNode(Node):
    def __init__(self):
        super().__init__('serial_motor_node')

        # シリアルポート管理
        self.ser = None
        self.is_connected = False
        self.serial_port_name = ''
        self.serial_lock = threading.Lock()

        # モーター状態 (RM1/RM2/LM1/LM2/SM1/LM3)
        self.motor_targets = [0, 0, 0, 0, 0, 0]   # ×10 に変換して送信
        self.control_mode = 0                      # 0=停止 1=PID 2=開ループ

        # フィードバック値
        self.fb_angles = [0.0] * 6
        self.fb_rpm = [0, 0]

        # 統計
        self.tx_count = 0
        self.rx_count = 0
        self.error_count = 0

        # ─── ROS2 インタフェース ──────────────────────
        self.pub_motor_fb = self.create_publisher(
            Float32MultiArray, '/catchrobo/motor_fb', 10)
        self.pub_serial_status = self.create_publisher(
            String, '/catchrobo/serial_status', 10)

        self.create_subscription(
            Float32MultiArray, '/catchrobo/motor_cmd', self._on_motor_cmd, 10)
        self.create_subscription(
            String, '/catchrobo/motor_mode', self._on_motor_mode, 10)
        self.create_subscription(
            String, '/catchrobo/set_ports', self._on_set_ports, 10)

        # 送信タイマー (50Hz)
        self.create_timer(0.020, self._timer_send)
        # ステータス送信タイマー (100Hz)
        self.create_timer(0.010, self._timer_status)

        # 受信スレッド
        self._recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._recv_thread.start()

        # 自動検出スレッド
        self._auto_scan_thread = threading.Thread(target=self._auto_scan_loop, daemon=True)
        self._auto_scan_thread.start()

        self.get_logger().info('シリアルモーターノード 起動完了 (自動スキャン開始)')

    # ─────────────────────────────────────────────────
    # ROS2 コールバック
    # ─────────────────────────────────────────────────

    def _on_set_ports(self, msg: String):
        """PCからのポート設定を受けてシリアル接続を切り替える"""
        try:
            data = json.loads(msg.data)
            port = data.get('serial_port', '')
            if not port:
                return
            if port == self.serial_port_name and self.is_connected:
                return
            self.get_logger().info(f'シリアルポート変更: {port}')
            self._disconnect()
            time.sleep(0.3)
            self._connect(port)
        except Exception as e:
            self.get_logger().error(f'set_ports 解析エラー: {e}')

    def _on_motor_cmd(self, msg: Float32MultiArray):
        """PCからのモーター目標値を受信する (degree値、×10してint16へ変換)"""
        vals = list(msg.data)
        for i in range(min(6, len(vals))):
            self.motor_targets[i] = int(vals[i] * 10)
        self.get_logger().info(f'motor_cmd 受信: targets={self.motor_targets}')

    def _on_motor_mode(self, msg: String):
        """PCからのモーター制御モードを受信する"""
        try:
            data = json.loads(msg.data)
            self.control_mode = int(data.get('mode', 0))
            self.get_logger().info(f'motor_mode 受信: control_mode={self.control_mode}')
        except Exception as e:
            self.get_logger().error(f'motor_mode 解析エラー: {e}')

    # ─────────────────────────────────────────────────
    # 送受信
    # ─────────────────────────────────────────────────

    def _timer_send(self):
        """50Hz: シリアルパケット送信"""
        if not self.is_connected or not self.ser:
            return
        try:
            rm1, rm2, lm1, lm2, sm1, lm3 = self.motor_targets
            payload = struct.pack('<BBhhhhhhb',
                                  0xAA, 0x55,
                                  rm1, rm2, lm1, lm2, sm1, lm3,
                                  self.control_mode)
            crc = self._calc_crc16(payload)
            packet = payload + struct.pack('<HB', crc, 0x0A)
            with self.serial_lock:
                self.ser.write(packet)
            self.tx_count += 1
        except Exception as e:
            self.error_count += 1
            self.is_connected = False
            self.get_logger().error(f'シリアル送信エラー: {e}')

    def _receive_loop(self):
        """シリアル受信ループ (別スレッド)"""
        rx_buf = bytearray()
        while rclpy.ok():
            ser = self.ser
            if not self.is_connected or not ser or not ser.is_open:
                time.sleep(0.05)
                continue
            try:
                # in_waitingを確認し、データがあればまとめて、なければ1バイトのブロッキングリードを行う
                waiting = ser.in_waiting
                chunk = ser.read(waiting if waiting > 0 else 1)
                
                if chunk:
                    rx_buf.extend(chunk)
                    # 18バイトパケットを探す
                    while len(rx_buf) >= 18:
                        idx = rx_buf.find(b'\xBB\x66')
                        if idx == -1:
                            rx_buf.clear()
                            break
                        if idx > 0:
                            del rx_buf[:idx]
                        if len(rx_buf) >= 18:
                            pkt = bytes(rx_buf[:18])
                            del rx_buf[:18]
                            self._parse_feedback(pkt)
            except Exception as e:
                if self.is_connected:
                    self.error_count += 1
                    self.is_connected = False
                    self.get_logger().error(f'シリアル受信エラー: {e}')

    def _parse_feedback(self, pkt: bytes):
        """受信パケットを解析してフィードバック値を更新する"""
        if len(pkt) < 18 or pkt[0] != 0xBB or pkt[1] != 0x66:
            return
        names = ['RM1', 'RM2', 'LM1', 'LM2', 'SM1', 'LM3']
        self.fb_angles = [
            struct.unpack_from('<h', pkt, 2 + i * 2)[0] / 10.0
            for i in range(6)
        ]
        self.fb_rpm = [
            struct.unpack_from('<h', pkt, 14)[0],
            struct.unpack_from('<h', pkt, 16)[0],
        ]
        self.rx_count += 1
        self._publish_motor_fb()

    def _publish_motor_fb(self):
        """モーターフィードバックをFloat32MultiArrayでパブリッシュする"""
        msg = Float32MultiArray()
        # data[0..4]: 角度, data[5..6]: RPM
        msg.data = [float(a) for a in self.fb_angles] + [float(r) for r in self.fb_rpm]
        self.pub_motor_fb.publish(msg)

    def _timer_status(self):
        """100Hz: シリアルステータス送信"""
        status = {
            'connected': self.is_connected,
            'port': self.serial_port_name,
            'tx_count': self.tx_count,
            'rx_count': self.rx_count,
            'error_count': self.error_count,
            'motor_angles': self.fb_angles,
            'motor_rpm': self.fb_rpm,
            'control_mode': self.control_mode,
            'motor_targets': [t / 10.0 for t in self.motor_targets],
            'timestamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_serial_status.publish(msg)

    # ─────────────────────────────────────────────────
    # CRC16 (Modbus)
    # ─────────────────────────────────────────────────

    def _calc_crc16(self, data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        return crc

    # ─────────────────────────────────────────────────
    # シリアルポート接続管理
    # ─────────────────────────────────────────────────

    def _auto_scan_loop(self):
        """未接続の間、1秒ごとにttyACM*をスキャンしてモーターマイコンを自動検出する"""
        import glob
        self.get_logger().info('モータースキャン開始...')
        while rclpy.ok():
            if self.is_connected:
                time.sleep(1.0)
                continue
            
            candidates = sorted(glob.glob('/dev/ttyACM*'))
            found_port = ''
            for port in candidates:
                try:
                    s = serial.Serial(port, 115200, timeout=0.2)
                    # ダミーコマンド（目標値0）を送信して応答を待つ
                    payload = struct.pack('<BBhhhhhhb', 0xAA, 0x55, 0, 0, 0, 0, 0, 0, 0)
                    crc = self._calc_crc16(payload)
                    packet = payload + struct.pack('<HB', crc, 0x0A)
                    s.write(packet)
                    
                    rx = s.read(64)
                    s.close()
                    
                    if b'\xBB\x66' in rx:
                        found_port = port
                        break
                except Exception as e:
                    pass
            
            if found_port:
                self.get_logger().info(f'モーターマイコン検出: {found_port}')
                self._connect(found_port)
            else:
                self.get_logger().warn('モーターデバイスが見つかりません。1秒後に再スキャンします...', throttle_duration_sec=5.0)
            
            time.sleep(1.0)

    def _connect(self, port: str):
        """シリアルポートに接続する"""
        try:
            s = serial.Serial(port, 115200, timeout=0.1)
            self.ser = s
            self.is_connected = True
            self.serial_port_name = port
            self.get_logger().info(f'シリアル 接続成功: {port}')
        except Exception as e:
            self.get_logger().error(f'シリアル 接続失敗 {port}: {e}')
            self.ser = None
            self.is_connected = False

    def _disconnect(self):
        """シリアル接続を切断する"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.is_connected = False
        self.serial_port_name = ''

    def destroy_node(self):
        self._disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
