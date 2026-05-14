"""
debug_node.py
cursesを使った全画面デバッグ表示ノード

表示内容:
  - 接続状態 (CAN / シリアル)
  - ロボマスモーター 角度・RPM
  - モジュール状態 (MDD1 / SV_1 / SV_2)
  - ROS2 トピック受信レート
  - 統計情報（TX/RX/ERR）
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

import curses
import threading
import time
import json


class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')

        # ─── 状態バッファ ─────────────────────────────
        self.can_status = {}
        self.serial_status = {}
        self.motor_fb = [0.0] * 7   # [ang×5, rpm×2]

        # トピック受信レート計測
        self._can_recv_times = []
        self._serial_recv_times = []
        self._motor_recv_times = []
        self._lock = threading.Lock()

        # ─── サブスクライバー ──────────────────────────
        self.create_subscription(String, '/catchrobo/can_status',    self._on_can_status,    10)
        self.create_subscription(String, '/catchrobo/serial_status', self._on_serial_status, 10)
        self.create_subscription(Float32MultiArray, '/catchrobo/motor_fb', self._on_motor_fb, 10)

        # ─── 描画スレッド起動 ──────────────────────────
        self._draw_thread = threading.Thread(target=self._run_curses, daemon=True)
        self._draw_thread.start()

        self.get_logger().info('デバッグノード 起動完了')

    # ─────────────────────────────────────────────────
    # ROS2 コールバック
    # ─────────────────────────────────────────────────

    def _on_can_status(self, msg: String):
        with self._lock:
            try:
                self.can_status = json.loads(msg.data)
                now = time.time()
                self._can_recv_times.append(now)
                self._can_recv_times = [t for t in self._can_recv_times if now - t < 2.0]
            except Exception:
                pass

    def _on_serial_status(self, msg: String):
        with self._lock:
            try:
                self.serial_status = json.loads(msg.data)
                now = time.time()
                self._serial_recv_times.append(now)
                self._serial_recv_times = [t for t in self._serial_recv_times if now - t < 2.0]
            except Exception:
                pass

    def _on_motor_fb(self, msg: Float32MultiArray):
        with self._lock:
            self.motor_fb = list(msg.data)
            now = time.time()
            self._motor_recv_times.append(now)
            self._motor_recv_times = [t for t in self._motor_recv_times if now - t < 2.0]

    # ─────────────────────────────────────────────────
    # レート計算ヘルパー
    # ─────────────────────────────────────────────────

    def _calc_rate(self, times: list) -> float:
        if len(times) < 2:
            return 0.0
        return (len(times) - 1) / (times[-1] - times[0] + 1e-9)

    # ─────────────────────────────────────────────────
    # curses 描画
    # ─────────────────────────────────────────────────

    def _run_curses(self):
        """curses フルスクリーン描画メインループ"""
        curses.wrapper(self._curses_main)

    def _curses_main(self, stdscr):
        curses.curs_set(0)   # カーソル非表示
        stdscr.nodelay(True) # 非ブロッキング入力
        curses.start_color()
        curses.use_default_colors()

        # カラーペア定義
        curses.init_pair(1, curses.COLOR_GREEN,   -1)   # オンライン
        curses.init_pair(2, curses.COLOR_RED,     -1)   # エラー/オフライン
        curses.init_pair(3, curses.COLOR_CYAN,    -1)   # 見出し
        curses.init_pair(4, curses.COLOR_YELLOW,  -1)   # 値
        curses.init_pair(5, curses.COLOR_WHITE,   -1)   # 通常テキスト
        curses.init_pair(6, curses.COLOR_MAGENTA, -1)   # ラベル

        COL_ONLINE  = curses.color_pair(1) | curses.A_BOLD
        COL_OFFLINE = curses.color_pair(2) | curses.A_BOLD
        COL_HEADER  = curses.color_pair(3) | curses.A_BOLD
        COL_VALUE   = curses.color_pair(4)
        COL_LABEL   = curses.color_pair(6)
        COL_NORM    = curses.color_pair(5)

        MOTOR_NAMES = ['RM1', 'RM2', 'LM1', 'LM2', 'SM1']

        while rclpy.ok():
            key = stdscr.getch()
            if key == ord('q'):
                break

            stdscr.erase()
            h, w = stdscr.getmaxyx()
            now_str = time.strftime('%Y-%m-%d %H:%M:%S')

            # ── タイトルバー ──────────────────────────
            title = ' CATCHROBO 2026 - NUC DEBUG MONITOR '
            stdscr.addstr(0, 0, '=' * w, COL_HEADER)
            stdscr.addstr(0, max(0, (w - len(title)) // 2), title, COL_HEADER)
            stdscr.addstr(0, max(0, w - len(now_str) - 1), now_str, COL_NORM)

            with self._lock:
                can  = dict(self.can_status)
                ser  = dict(self.serial_status)
                fb   = list(self.motor_fb)
                can_rate    = self._calc_rate(list(self._can_recv_times))
                serial_rate = self._calc_rate(list(self._serial_recv_times))
                motor_rate  = self._calc_rate(list(self._motor_recv_times))

            row = 2

            # ── 接続状態 ──────────────────────────────
            stdscr.addstr(row, 1, '[ CONNECTIONS ]', COL_HEADER)
            row += 1

            can_conn = can.get('connected', False)
            can_port = can.get('port', '---')
            stdscr.addstr(row, 3, 'CAN   : ', COL_LABEL)
            if can_conn:
                stdscr.addstr(row, 11, f'● ONLINE  ({can_port})', COL_ONLINE)
            else:
                stdscr.addstr(row, 11, '○ OFFLINE', COL_OFFLINE)
            row += 1

            ser_conn = ser.get('connected', False)
            ser_port = ser.get('port', '---')
            stdscr.addstr(row, 3, 'Serial: ', COL_LABEL)
            if ser_conn:
                stdscr.addstr(row, 11, f'● ONLINE  ({ser_port})', COL_ONLINE)
            else:
                stdscr.addstr(row, 11, '○ OFFLINE', COL_OFFLINE)
            row += 2

            # ── ロボマスモーター ──────────────────────
            stdscr.addstr(row, 1, '[ ROBOMASTER MOTORS ]', COL_HEADER)
            row += 1
            header = f'  {"Motor":<6} {"Angle[°]":>10} {"RPM":>8}  {"Mode":>6}'
            stdscr.addstr(row, 3, header, COL_LABEL)
            row += 1

            fb_angles = fb[:5] if len(fb) >= 5 else [0.0] * 5
            fb_rpms   = fb[5:7] if len(fb) >= 7 else [0, 0]
            ctrl_mode = ser.get('control_mode', 0)
            mode_str  = ['STOP', 'PID', 'OpenLoop']

            for i, name in enumerate(MOTOR_NAMES):
                ang = fb_angles[i] if i < len(fb_angles) else 0.0
                rpm = fb_rpms[i]   if i < len(fb_rpms)   else '-'
                rpm_str = f'{int(rpm):+d}' if isinstance(rpm, (int, float)) else '-'
                stdscr.addstr(row, 3, f'  {name:<6}', COL_LABEL)
                stdscr.addstr(row, 9, f'{ang:>+10.1f}°', COL_VALUE)
                stdscr.addstr(row, 21, f'{rpm_str:>8}', COL_VALUE)
                if i == 0:
                    stdscr.addstr(row, 31, f'  [{mode_str[min(ctrl_mode, 2)]}]', COL_NORM)
                row += 1
            row += 1

            # ── モジュール状態 ──────────────────────────
            stdscr.addstr(row, 1, '[ MODULES ]', COL_HEADER)
            row += 1

            modules = can.get('modules', {})

            # MDD1
            mdd = modules.get('MDD1', {})
            app_mode = mdd.get('app_mode', 0)
            sw   = mdd.get('sw', [0, 0, 0, 0])
            err  = mdd.get('err', 0)
            enc  = mdd.get('enc_deg', [0.0, 0.0, 0.0, 0.0])
            tx   = mdd.get('tx_enabled', False)
            mdd_mode_str = 'CTRL' if app_mode == 1 else 'PARAM'
            mdd_col = COL_ONLINE if app_mode == 1 else COL_OFFLINE
            stdscr.addstr(row, 3, 'MDD1 (0x200): ', COL_LABEL)
            stdscr.addstr(row, 17, mdd_mode_str, mdd_col)
            stdscr.addstr(row, 23, f'TX:{"ON " if tx else "OFF"}  SW:{sw}  Err:{err}', COL_NORM)
            row += 1
            enc_str = '  ENC[°]: ' + ' '.join(f'M{i+1}:{enc[i]:+.1f}' for i in range(4))
            stdscr.addstr(row, 3, enc_str, COL_VALUE)
            row += 1

            # SV_1 / SV_2
            for sv_key, sv_addr in [('SV_1', '0x300'), ('SV_2', '0x301')]:
                sv = modules.get(sv_key, {})
                valves = sv.get('valves', 0)
                sv_tx  = sv.get('tx_enabled', False)
                # ビットを表示 (チャンネル1-12)
                valve_bits = ''.join('1' if (valves >> i) & 1 else '0' for i in range(12))
                stdscr.addstr(row, 3, f'{sv_key} ({sv_addr}): ', COL_LABEL)
                stdscr.addstr(row, 17, f'TX:{"ON " if sv_tx else "OFF"}  Valves:[{valve_bits}]', COL_NORM)
                row += 1
            row += 1

            # ── 統計 ──────────────────────────────────
            stdscr.addstr(row, 1, '[ STATISTICS ]', COL_HEADER)
            row += 1

            can_tx  = can.get('tx_count', 0)
            can_rx  = can.get('rx_count', 0)
            can_err = can.get('error_count', 0)
            stdscr.addstr(row, 3, 'CAN    : ', COL_LABEL)
            stdscr.addstr(row, 12, f'TX:{can_tx:6d}  RX:{can_rx:6d}  ERR:{can_err:4d}  Rate:{can_rate:.1f}Hz', COL_VALUE)
            row += 1

            ser_tx  = ser.get('tx_count', 0)
            ser_rx  = ser.get('rx_count', 0)
            ser_err = ser.get('error_count', 0)
            stdscr.addstr(row, 3, 'Serial : ', COL_LABEL)
            stdscr.addstr(row, 12, f'TX:{ser_tx:6d}  RX:{ser_rx:6d}  ERR:{ser_err:4d}  Rate:{serial_rate:.1f}Hz', COL_VALUE)
            row += 1

            stdscr.addstr(row, 3, 'MotorFB: ', COL_LABEL)
            stdscr.addstr(row, 12, f'Rate:{motor_rate:.1f}Hz', COL_VALUE)
            row += 2

            # ── フッター ──────────────────────────────
            footer = ' [q] Quit '
            try:
                stdscr.addstr(h - 1, 0, '=' * (w - 1), COL_HEADER)
                stdscr.addstr(h - 1, 2, footer, COL_NORM)
            except curses.error:
                pass

            stdscr.refresh()
            time.sleep(0.1)   # 10Hz描画

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
