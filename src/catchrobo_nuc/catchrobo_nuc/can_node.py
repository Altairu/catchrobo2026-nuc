"""
can_node.py
USB-CAN (slcanプロトコル) を使ってモジュール回路と通信するROS2ノード

対象モジュール:
  - MDD1  (Motor Driver Driver): BaseID = 0x200
  - SV_1  (Solenoid Valve):      BaseID = 0x300
  - SV_2  (Solenoid Valve):      BaseID = 0x301
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import serial.tools.list_ports
import threading
import time
import json


class CanNode(Node):
    def __init__(self):
        super().__init__('can_node')

        # シリアルポート管理
        self.ser = None
        self.is_connected = False
        self.can_port_name = ''
        self.serial_lock = threading.Lock()

        # 統計カウンタ
        self.tx_count = 0
        self.rx_count = 0
        self.error_count = 0

        # ─── モジュール状態 ───────────────────────────

        # MDD1: MotorDriverDriver (BaseID=0x200)
        self.mdd1 = {
            'base_id': 0x200,
            'name': 'MDD1',
            'tx_enabled': True,
            'motors': [
                {'target': 0, 'mode': 0, 'p': 10.0, 'i': 0.0, 'd': 0.0, 'wheel': 65, 'dir': 1},
                {'target': 0, 'mode': 0, 'p': 10.0, 'i': 0.0, 'd': 0.0, 'wheel': 65, 'dir': 1},
                {'target': 0, 'mode': 0, 'p': 10.0, 'i': 0.0, 'd': 0.0, 'wheel': 65, 'dir': 1},
                {'target': 0, 'mode': 0, 'p': 10.0, 'i': 0.0, 'd': 0.0, 'wheel': 65, 'dir': 1},
            ],
            'state': {
                'app_mode': 0,
                'param_send_requested': False,
                'param_setup_completed': False,
                'sw': [0, 0, 0, 0],
                'err': 0,
                'enc_deg': [0.0, 0.0, 0.0, 0.0],
                'enc_rps': [0.0, 0.0, 0.0, 0.0],
                'last_update': 0.0,
            },
        }

        # SV_1: Solenoid Valve (BaseID=0x300)
        self.sv1 = {
            'base_id': 0x300,
            'name': 'SV_1',
            'tx_enabled': True,
            'valves': 0,   # ビットフィールド (12ch分)
        }

        # SV_2: Solenoid Valve (BaseID=0x301)
        self.sv2 = {
            'base_id': 0x301,
            'name': 'SV_2',
            'tx_enabled': True,
            'valves': 0,
        }

        # ─── ROS2 インタフェース ──────────────────────

        # パブリッシャー
        self.pub_can_status = self.create_publisher(String, '/catchrobo/can_status', 10)
        self.pub_available_ports = self.create_publisher(String, '/catchrobo/available_ports', 10)

        # サブスクライバー
        self.create_subscription(String, '/catchrobo/module_cmd', self._on_module_cmd, 10)
        self.create_subscription(String, '/catchrobo/set_ports', self._on_set_ports, 10)
        self.create_subscription(String, '/catchrobo/serial_status', self._on_serial_status, 10)
        self.known_motor_port = ''

        # タイマー
        self.create_timer(0.010, self._timer_10ms)   # MDD送信 (100Hz)
        self.create_timer(0.100, self._timer_100ms)  # Solenoid送信 (10Hz)
        self.create_timer(1.000, self._timer_1s)     # ステータス送信 + ポートスキャン

        # CAN受信スレッド
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

        # 起動時に自動でCANデバイスを検出して接続する
        self._auto_scan_thread = threading.Thread(target=self._auto_scan_loop, daemon=True)
        self._auto_scan_thread.start()

        self.get_logger().info('CANノード 起動完了 (CAN自動検出を開始)')

    # ─────────────────────────────────────────────────
    # ROS2 コールバック
    # ─────────────────────────────────────────────────

    def _on_set_ports(self, msg: String):
        """PCからのポート設定コマンドを受信してCAN接続を切り替える"""
        try:
            data = json.loads(msg.data)
            can_port = data.get('can_port', '')
            if not can_port:
                return
            if can_port == self.can_port_name and self.is_connected:
                return  # 変更なし
            self.get_logger().info(f'CAN ポート変更: {can_port}')
            self._disconnect()
            time.sleep(0.3)
            self._connect(can_port)
        except Exception as e:
            self.get_logger().error(f'set_ports 解析エラー: {e}')

    def _on_serial_status(self, msg: String):
        """モーターノードのステータスを受信し、使用中のポートを把握する"""
        try:
            data = json.loads(msg.data)
            if data.get('connected'):
                self.known_motor_port = data.get('port', '')
            else:
                self.known_motor_port = ''
        except Exception:
            pass

    def _on_module_cmd(self, msg: String):
        """PCからのモジュール操作コマンドを受信して状態を更新する"""
        try:
            data = json.loads(msg.data)
            mod_name = data.get('name', '')
            action = data.get('action', '')

            if mod_name == 'MDD1':
                self._handle_mdd_cmd(self.mdd1, action, data)
            elif mod_name == 'SV_1':
                self._handle_solenoid_cmd(self.sv1, action, data)
            elif mod_name == 'SV_2':
                self._handle_solenoid_cmd(self.sv2, action, data)
        except Exception as e:
            self.get_logger().error(f'module_cmd 解析エラー: {e}')

    def _handle_mdd_cmd(self, m: dict, action: str, data: dict):
        """MDD向けコマンド処理"""
        if action == 'set_target':
            targets = data.get('targets', [])
            for i, t in enumerate(targets[:4]):
                m['motors'][i]['target'] = int(t)
        elif action == 'set_params':
            idx = data.get('motor_idx', 0)
            if 0 <= idx < 4:
                motor = m['motors'][idx]
                for key in ('p', 'i', 'd', 'wheel', 'mode', 'dir'):
                    if key in data:
                        motor[key] = data[key]
        elif action == 'send_params':
            m['state']['param_send_requested'] = True
            m['state']['param_setup_completed'] = False
            self.get_logger().info('MDD1 パラメータ送信要求')
        elif action == 'set_tx':
            m['tx_enabled'] = bool(data.get('enabled', True))

    def _handle_solenoid_cmd(self, m: dict, action: str, data: dict):
        """Solenoid向けコマンド処理"""
        if action == 'set_valves':
            m['valves'] = int(data.get('valves', 0)) & 0xFFF
        elif action == 'toggle_valve':
            idx = int(data.get('valve_idx', 0))
            m['valves'] ^= (1 << idx)
        elif action == 'set_tx':
            m['tx_enabled'] = bool(data.get('enabled', True))

    # ─────────────────────────────────────────────────
    # 送信タイマー
    # ─────────────────────────────────────────────────

    def _timer_10ms(self):
        """100Hz: MDD送信ループ"""
        if not self.is_connected:
            return
        m = self.mdd1
        if not m['tx_enabled']:
            return
        if m['state']['param_send_requested']:
            self._send_mdd_params(m)
            self._send_mdd_mode(m)
        elif m['state']['app_mode'] == 1 and m['state']['param_setup_completed']:
            self._send_mdd_target(m)

    def _timer_100ms(self):
        """10Hz: Solenoid送信ループ"""
        if not self.is_connected:
            return
        for sv in (self.sv1, self.sv2):
            if sv['tx_enabled']:
                self._send_solenoid(sv)

    def _timer_1s(self):
        """1Hz: ステータスとポートリスト公開"""
        self._publish_can_status()
        self._publish_available_ports()

    # ─────────────────────────────────────────────────
    # CAN フレーム送受信
    # ─────────────────────────────────────────────────

    def _send_can_frame(self, can_id: int, data: list):
        """slcan フォーマットでCANフレームを送信する"""
        if not self.is_connected or not self.ser:
            return
        try:
            dlc = min(len(data), 8)
            id_hex = f'{can_id:03X}'
            data_hex = ''.join(f'{b & 0xFF:02X}' for b in data[:dlc])
            frame = f't{id_hex}{dlc}{data_hex}\r'.encode()
            with self.serial_lock:
                self.ser.write(frame)
            self.tx_count += 1
        except Exception as e:
            self.error_count += 1
            self.is_connected = False
            self.get_logger().error(f'CAN送信エラー: {e}')

    def _encode_i16le(self, val: int) -> list:
        """符号付き16bit Little Endianにエンコードする"""
        val = max(-32768, min(32767, int(val))) & 0xFFFF
        return [val & 0xFF, (val >> 8) & 0xFF]

    def _send_mdd_params(self, m: dict):
        """MDD: PIDパラメータ送信 (BaseID+0x00 ~ +0x03)"""
        for i in range(4):
            motor = m['motors'][i]
            p = int(motor['p'] * 100)
            ig = int(motor['i'] * 100)
            d = int(motor['d'] * 100)
            wheel_dir = int(motor['wheel']) * (1 if motor['dir'] >= 0 else -1)
            payload = (self._encode_i16le(p) + self._encode_i16le(ig) +
                       self._encode_i16le(d) + self._encode_i16le(wheel_dir))
            self._send_can_frame(m['base_id'] + i, payload)

    def _send_mdd_mode(self, m: dict):
        """MDD: モード送信 (BaseID+0x10)"""
        payload = [m['motors'][i]['mode'] for i in range(4)]
        self._send_can_frame(m['base_id'] + 0x10, payload)

    def _send_mdd_target(self, m: dict):
        """MDD: 目標値送信 (BaseID+0x20)"""
        payload = []
        for i in range(4):
            t = max(-32768, min(32767, m['motors'][i]['target']))
            payload.extend(self._encode_i16le(t))
        self._send_can_frame(m['base_id'] + 0x20, payload)

    def _send_solenoid(self, m: dict):
        """Solenoid: バルブ状態送信"""
        v = m['valves']
        payload = [v & 0xFF, (v >> 8) & 0xFF]
        self._send_can_frame(m['base_id'], payload)

    def _read_loop(self):
        """CANフレーム受信ループ (別スレッド)"""
        buf = ''
        while rclpy.ok():
            if not self.is_connected or not self.ser:
                time.sleep(0.05)
                continue
            try:
                waiting = 0
                with self.serial_lock:
                    if self.ser.is_open:
                        waiting = self.ser.in_waiting
                
                if waiting > 0:
                    with self.serial_lock:
                        chunk = self.ser.read(waiting)
                    buf += chunk.decode('ascii', errors='ignore')
                    while '\r' in buf:
                        line, buf = buf.split('\r', 1)
                        self._parse_slcan_line(line.strip())
                else:
                    time.sleep(0.002)
            except Exception as e:
                self.error_count += 1
                self.is_connected = False
                self.get_logger().error(f'CAN受信エラー: {e}')

    def _parse_slcan_line(self, line: str):
        """slcan受信フレームを解析する"""
        if not line or line[0] not in ('t', 'T'):
            return
        try:
            ext = (line[0] == 'T')
            id_len = 8 if ext else 3
            can_id = int(line[1:1 + id_len], 16)
            dlc = int(line[1 + id_len], 10)
            hex_data = line[2 + id_len:2 + id_len + dlc * 2]
            data = [int(hex_data[i*2:i*2+2], 16) for i in range(dlc)]
            self.rx_count += 1
            self._handle_rx_frame(can_id, data)
        except Exception as e:
            self.error_count += 1

    def _handle_rx_frame(self, can_id: int, data: list):
        """受信CANフレームをモジュール状態に反映する"""
        m = self.mdd1
        base = m['base_id']

        if can_id == base + 0x30:
            # MDD ステータス受信
            if len(data) >= 6:
                m['state']['sw'] = list(data[:4])
                m['state']['err'] = data[4]
                m['state']['app_mode'] = data[5] & 0x01
                m['state']['last_update'] = time.time()
                if m['state']['param_send_requested'] and m['state']['app_mode'] == 1:
                    m['state']['param_send_requested'] = False
                    m['state']['param_setup_completed'] = True
                    self.get_logger().info('MDD1 パラメータ設定完了')

        elif can_id == base + 0x40 and len(data) >= 8:
            # エンコーダ角度受信
            for i in range(4):
                raw = data[i*2] | (data[i*2+1] << 8)
                if raw >= 0x8000:
                    raw -= 0x10000
                m['state']['enc_deg'][i] = raw / 10.0

        elif can_id == base + 0x50 and len(data) >= 8:
            # エンコーダ角速度受信
            for i in range(4):
                raw = data[i*2] | (data[i*2+1] << 8)
                if raw >= 0x8000:
                    raw -= 0x10000
                m['state']['enc_rps'][i] = raw / 100.0

    # ─────────────────────────────────────────────────
    # シリアルポート接続管理
    # ─────────────────────────────────────────────────

    def _auto_scan_loop(self):
        """未接続の間、1秒ごとにttyACM*をスキャンしてCANデバイスを自動検出するループ"""
        self.get_logger().info('CAN自動スキャン待機中... (モーター通信の確立待ち)')
        while rclpy.ok():
            if self.is_connected:
                time.sleep(1.0)
                continue
            
            # モーター側が接続されるまでCANのスキャンは保留する
            if not self.known_motor_port:
                time.sleep(1.0)
                continue

            port = self._detect_can_port()
            if port:
                self._connect(port)
            else:
                self.get_logger().warn(
                    'CANデバイスが見つかりません。1秒後に再スキャンします...', throttle_duration_sec=5.0)
            time.sleep(1.0)

    def _detect_can_port(self) -> str:
        """ttyACM*ポートをすべて試してslcanデバイスを自動判定する。

        判定方法:
          1. ポートを115200bpsで開く
          2. 'C\\r' (チャンネルクローズ) を送って既存状態をリセット
          3. 'S8\\r' (1Mbps設定) を送る
          4. 'O\\r' (チャンネルオープン) を送って応答を確認
          5. 応答バイトが存在すればslcanデバイスと判定
        """
        import glob
        candidates = sorted(glob.glob('/dev/ttyACM*'))
        
        # モーターノードが既に使用しているポートはスキャン対象から除外する
        if self.known_motor_port in candidates:
            candidates.remove(self.known_motor_port)
            
        if not candidates:
            return ''
        self.get_logger().info(f'スキャン対象ポート: {candidates}')
        for port in candidates:
            try:
                s = serial.Serial(port, 115200, timeout=0.3)
                s.reset_input_buffer()
                # リセット
                s.write(b'C\r')
                time.sleep(0.1)
                s.reset_input_buffer()
                # ボーレート設定
                s.write(b'S8\r')
                time.sleep(0.1)
                # オープンして応答を確認
                s.write(b'O\r')
                time.sleep(0.15)
                resp = s.read(s.in_waiting or 1)
                s.close()
                if resp:  # 何らかの応答があればslcanデバイスとみなす
                    self.get_logger().info(
                        f'slcanデバイス検出: {port} (応答={resp!r})')
                    return port
                else:
                    self.get_logger().info(
                        f'{port} は応答なし (マイコン側シリアルと判定)')
            except Exception as e:
                self.get_logger().warn(f'{port} スキャン失敗: {e}')
        return ''

    def _connect(self, port: str):
        """slcanデバイスに接続する"""
        try:
            s = serial.Serial(port, 115200, timeout=0.05)
            # slcan初期化シーケンス
            s.write(b'C\r')
            time.sleep(0.1)
            s.write(b'S8\r')   # 1Mbps
            time.sleep(0.1)
            s.write(b'O\r')    # チャンネルオープン
            time.sleep(0.1)
            s.reset_input_buffer()
            self.ser = s
            self.is_connected = True
            self.can_port_name = port
            self.get_logger().info(f'CAN 接続成功: {port}')
        except Exception as e:
            self.get_logger().error(f'CAN 接続失敗 {port}: {e}')
            self.ser = None
            self.is_connected = False

    def _disconnect(self):
        """CAN接続を切断する"""
        if self.ser and self.ser.is_open:
            try:
                with self.serial_lock:
                    self.ser.write(b'C\r')
                time.sleep(0.1)
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.is_connected = False
        self.can_port_name = ''

    # ─────────────────────────────────────────────────
    # パブリッシュ
    # ─────────────────────────────────────────────────

    def _publish_can_status(self):
        """CANノードの状態をJSONでパブリッシュする"""
        m = self.mdd1
        status = {
            'connected': self.is_connected,
            'port': self.can_port_name,
            'tx_count': self.tx_count,
            'rx_count': self.rx_count,
            'error_count': self.error_count,
            'modules': {
                'MDD1': {
                    'base_id': m['base_id'],
                    'tx_enabled': m['tx_enabled'],
                    'app_mode': m['state']['app_mode'],
                    'sw': m['state']['sw'],
                    'err': m['state']['err'],
                    'enc_deg': m['state']['enc_deg'],
                    'enc_rps': m['state']['enc_rps'],
                    'last_update': m['state']['last_update'],
                },
                'SV_1': {
                    'base_id': self.sv1['base_id'],
                    'tx_enabled': self.sv1['tx_enabled'],
                    'valves': self.sv1['valves'],
                },
                'SV_2': {
                    'base_id': self.sv2['base_id'],
                    'tx_enabled': self.sv2['tx_enabled'],
                    'valves': self.sv2['valves'],
                },
            },
            'timestamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_can_status.publish(msg)

    def _publish_available_ports(self):
        """利用可能なシリアルポートをJSONでパブリッシュする"""
        ports = [p.device for p in serial.tools.list_ports.comports()]
        msg = String()
        msg.data = json.dumps({'ports': ports, 'timestamp': time.time()})
        self.pub_available_ports.publish(msg)

    def destroy_node(self):
        self._disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
