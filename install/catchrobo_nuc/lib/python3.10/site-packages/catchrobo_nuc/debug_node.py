import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import curses
import threading
import time
import json

class PCDebugNode(Node):
    """
    PC側で動作するCUIデバッグモニター。
    ROS2トピックの受信レートと内容を直接表示し、WebSocketを介さない生の通信状態を確認する。
    """
    def __init__(self):
        super().__init__('pc_debug_node')

        # --- 状態バッファ ---
        self.can_status = {}
        self.serial_status = {}
        self.motor_fb = []
        self._lock = threading.Lock()

        # レート計測用
        self.rates = {
            'can':    {'times': [], 'freq': 0.0, 'last': 0.0},
            'serial': {'times': [], 'freq': 0.0, 'last': 0.0},
            'motor':  {'times': [], 'freq': 0.0, 'last': 0.0},
        }

        # --- サブスクライバー ---
        self.create_subscription(String,            '/catchrobo/can_status',    self._on_can_status,    10)
        self.create_subscription(String,            '/catchrobo/serial_status', self._on_serial_status, 10)
        self.create_subscription(Float32MultiArray, '/catchrobo/motor_fb',      self._on_motor_fb,      10)

        # --- 描画スレッド ---
        self._draw_thread = threading.Thread(target=self._run_curses, daemon=True)
        self._draw_thread.start()


    def _update_rate(self, key):
        now = time.time()
        r = self.rates[key]
        r['times'].append(now)
        r['last'] = now
        # 過去2秒間のデータで計算
        r['times'] = [t for t in r['times'] if now - t < 2.0]
        if len(r['times']) > 1:
            r['freq'] = (len(r['times']) - 1) / (r['times'][-1] - r['times'][0] + 1e-9)

    def _on_can_status(self, msg: String):
        with self._lock:
            try:
                self.can_status = json.loads(msg.data)
                self._update_rate('can')
            except: pass

    def _on_serial_status(self, msg: String):
        with self._lock:
            try:
                self.serial_status = json.loads(msg.data)
                self._update_rate('serial')
            except: pass

    def _on_motor_fb(self, msg: Float32MultiArray):
        with self._lock:
            self.motor_fb = list(msg.data)
            self._update_rate('motor')

    def _run_curses(self):
        curses.wrapper(self._curses_main)

    def _curses_main(self, stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)  # Online
        curses.init_pair(2, curses.COLOR_RED, -1)    # Offline/Error
        curses.init_pair(3, curses.COLOR_CYAN, -1)   # Header
        curses.init_pair(4, curses.COLOR_YELLOW, -1) # Value
        curses.init_pair(5, curses.COLOR_WHITE, -1)  # Normal

        while rclpy.ok():
            if stdscr.getch() == ord('q'): break
            stdscr.erase()
            h, w = stdscr.getmaxyx()
            now = time.time()

            # --- タイトル ---
            title = "=== CATCHROBO 2026 PC DEBUG MONITOR (CUI) ==="
            stdscr.addstr(0, max(0, (w - len(title)) // 2), title, curses.color_pair(3) | curses.A_BOLD)
            stdscr.addstr(1, 1, f"Time: {time.strftime('%H:%M:%S')}  [q]: Quit", curses.color_pair(5))

            with self._lock:
                row = 3
                # --- 通信統計 (PCでの受信状況) ---
                stdscr.addstr(row, 0, "[ TOPIC RATES & JITTER ]", curses.color_pair(3) | curses.A_BOLD)
                row += 1
                for label, key in [("CAN Status   ", "can"), ("Serial Status", "serial"), ("Motor FB     ", "motor")]:
                    freq = self.rates[key]['freq']
                    last_time = self.rates[key]['last']
                    dt = now - last_time if last_time > 0 else 999
                    
                    status_col = curses.color_pair(1) if dt < 0.5 else curses.color_pair(2)
                    stdscr.addstr(row, 2, f"{label}: ", curses.A_BOLD)
                    stdscr.addstr(f"{freq:5.1f} Hz", curses.color_pair(4))
                    stdscr.addstr(f"  (Last: {dt:4.2f}s ago) ", status_col)
                    row += 1
                
                row += 1
                # --- MCU接続状態 (NUC側での判定) ---
                stdscr.addstr(row, 0, "[ MCU CONNECTION (from NUC) ]", curses.color_pair(3) | curses.A_BOLD)
                row += 1
                can_conn = self.can_status.get('connected', False)
                ser_conn = self.serial_status.get('connected', False)
                
                stdscr.addstr(row, 2, "CAN    : ")
                stdscr.addstr("ONLINE" if can_conn else "OFFLINE", curses.color_pair(1) if can_conn else curses.color_pair(2))
                if can_conn: stdscr.addstr(f" ({self.can_status.get('port','?')})", curses.color_pair(5))
                row += 1
                stdscr.addstr(row, 2, "Serial : ")
                stdscr.addstr("ONLINE" if ser_conn else "OFFLINE", curses.color_pair(1) if ser_conn else curses.color_pair(2))
                if ser_conn: stdscr.addstr(f" ({self.serial_status.get('port','?')})", curses.color_pair(5))
                row += 2

                # --- モーターフィードバック簡易表示 ---
                if self.motor_fb:
                    stdscr.addstr(row, 0, "[ MOTOR FB (Encoder) ]", curses.color_pair(3) | curses.A_BOLD)
                    row += 1
                    fb_str = " ".join([f"M{i}:{v:>6.1f}°" for i, v in enumerate(self.motor_fb[:5])])
                    stdscr.addstr(row, 2, fb_str, curses.color_pair(4))
                    row += 1
                    if len(self.motor_fb) >= 7:
                        rpm_str = " ".join([f"M{i}:{int(v):>5d}RPM" for i, v in enumerate(self.motor_fb[5:7])])
                        stdscr.addstr(row, 2, rpm_str, curses.color_pair(4))
                    row += 2

                # --- スイッチ状態 (can_status.modules.MDD1.sw) ---
                stdscr.addstr(row, 0, "[ MDD1 SWITCH (sw) ]", curses.color_pair(3) | curses.A_BOLD)
                row += 1
                mdd1 = self.can_status.get('modules', {}).get('MDD1', {})
                sw = mdd1.get('sw', None)
                enc_deg = mdd1.get('enc_deg', [])
                enc_rps = mdd1.get('enc_rps', [])
                can_alive = (now - self.rates['can']['last']) < 0.5 if self.rates['can']['last'] > 0 else False
                if sw is not None:
                    stdscr.move(row, 2)
                    for i, v in enumerate(sw):
                        if row >= h - 2: break
                        disp = "ON " if v else "OFF"
                        c = (curses.color_pair(1) if v else curses.color_pair(2)) if can_alive else curses.color_pair(2)
                        stdscr.addstr(f"SW{i}: ", curses.A_BOLD)
                        stdscr.addstr(disp, c)
                        stdscr.addstr("  ", curses.color_pair(5))
                    row += 1
                else:
                    stdscr.addstr(row, 2, "(can_status に MDD1 データなし)", curses.color_pair(2))
                    row += 1

                # --- MDD1 エンコーダー (enc_deg / enc_rps) ---
                if row < h - 4 and enc_deg:
                    row += 1
                    stdscr.addstr(row, 0, "[ MDD1 ENCODER ]", curses.color_pair(3) | curses.A_BOLD)
                    row += 1
                    deg_str = "  ".join([f"M{i}:{v:>7.2f}deg" for i, v in enumerate(enc_deg)])
                    stdscr.addstr(row, 2, deg_str, curses.color_pair(4))
                    row += 1
                    if enc_rps:
                        rps_str = "  ".join([f"M{i}:{v:>6.2f}rps" for i, v in enumerate(enc_rps)])
                        stdscr.addstr(row, 2, rps_str, curses.color_pair(4))
                        row += 1

            stdscr.refresh()
            time.sleep(0.1)  # 10Hz更新

def main(args=None):
    rclpy.init(args=args)
    node = PCDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
