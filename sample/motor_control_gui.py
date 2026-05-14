import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import struct
import threading
import time


class MotorControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RoboMaster Motor Controller")
        self.root.geometry("620x560")
        self.root.resizable(True, True)
        self.root.minsize(500, 480)

        self.serial_port = None
        self.is_connected = False
        self.is_running = False

        # フィードバックデータ
        self.fb_angles = [0.0] * 5  # RM1, RM2, LM1, LM2, SM1
        self.fb_rpm    = [0, 0]     # RM1, RM2

        # モーター設定（名前: range, step）
        self.motors = {
            "RM1": {"range": (-20.0, 60.0),  "step": 0.5, "val": tk.DoubleVar(value=0.0)},
            "RM2": {"range": (-15.0, 90.0),  "step": 0.5, "val": tk.DoubleVar(value=0.0)},
            "LM1": {"range": (-20.0, 30.0),  "step": 0.5, "val": tk.DoubleVar(value=0.0)},
            "LM2": {"range": (-10.0, 20.0),  "step": 0.5, "val": tk.DoubleVar(value=0.0)},
            "SM1": {"range": (-90.0, 90.0),  "step": 1.0, "val": tk.DoubleVar(value=0.0)},
        }

        self._sliders = {}
        self._spinboxes = {}

        self.setup_ui()

    # ─────────────────────────────────────────
    # UI構築
    # ─────────────────────────────────────────
    def setup_ui(self):
        PAD = dict(padx=6, pady=4)

        # ── シリアル接続 ──
        fr_serial = ttk.LabelFrame(self.root, text="シリアル接続", padding=4)
        fr_serial.pack(fill="x", padx=8, pady=(6, 2))

        self.cb_ports = ttk.Combobox(fr_serial, state="readonly", width=18)
        self.cb_ports.pack(side="left", padx=(0, 4))
        self.refresh_ports()

        ttk.Button(fr_serial, text="更新", width=4,
                   command=self.refresh_ports).pack(side="left", padx=2)
        self.btn_connect = ttk.Button(fr_serial, text="接続", width=5,
                                      command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=2)

        # ── 制御モード（横並び） ──
        fr_mode = ttk.LabelFrame(self.root, text="制御モード", padding=4)
        fr_mode.pack(fill="x", padx=8, pady=2)

        self.mode_var = tk.IntVar(value=0)
        modes = [("■ 停止", 0), ("⟳ PID制御", 1), ("⚡ 同定(開ループ)", 2)]
        for txt, val in modes:
            ttk.Radiobutton(fr_mode, text=txt, variable=self.mode_var,
                            value=val).pack(side="left", padx=8)
        ttk.Label(fr_mode, text="※同定: スライダー×500 を直接出力",
                  foreground="gray", font=("", 8)).pack(side="left", padx=6)

        # ── 目標角度コントロール ──
        fr_motors = ttk.LabelFrame(
            self.root, text="目標角度 [deg]  (PID) / ゲイン係数  (同定)", padding=6)
        fr_motors.pack(fill="both", expand=True, padx=8, pady=2)

        # ヘッダー
        hdr = ttk.Frame(fr_motors)
        hdr.pack(fill="x")
        ttk.Label(hdr, text="Motor", width=5,  anchor="center").grid(row=0, column=0)
        ttk.Label(hdr, text="範囲",  width=10, anchor="center").grid(row=0, column=1)
        ttk.Label(hdr, text="スライダー",      anchor="center").grid(row=0, column=2, sticky="ew")
        ttk.Label(hdr, text="微調整",width=9,  anchor="center").grid(row=0, column=3)
        ttk.Label(hdr, text="値 [°]",width=8,  anchor="center").grid(row=0, column=4)
        ttk.Label(hdr, text="↺",     width=3,  anchor="center").grid(row=0, column=5)
        hdr.columnconfigure(2, weight=1)
        ttk.Separator(fr_motors, orient="horizontal").pack(fill="x", pady=2)

        for name, data in self.motors.items():
            lo, hi = data["range"]
            step   = data["step"]
            var    = data["val"]

            row = ttk.Frame(fr_motors)
            row.pack(fill="x", pady=3)

            # モーター名
            ttk.Label(row, text=name, width=5, font=("", 10, "bold"),
                      anchor="center").grid(row=0, column=0)

            # 範囲表示
            ttk.Label(row, text=f"{lo:+.0f}~{hi:+.0f}",
                      width=10, anchor="center", foreground="gray").grid(row=0, column=1)

            # スライダー
            sl = ttk.Scale(row, from_=lo, to=hi, variable=var,
                           orient="horizontal",
                           command=lambda v, n=name: self._on_slider(n))
            sl.grid(row=0, column=2, sticky="ew", padx=4)
            self._sliders[name] = sl

            # 微調整ボタン
            btn_fr = ttk.Frame(row)
            btn_fr.grid(row=0, column=3)
            ttk.Button(btn_fr, text="−", width=2,
                       command=lambda n=name: self._nudge(n, -1)).pack(side="left")
            ttk.Button(btn_fr, text="＋", width=2,
                       command=lambda n=name: self._nudge(n, +1)).pack(side="left")

            # スピンボックス（直接入力可）
            sp = ttk.Spinbox(row, textvariable=var,
                             from_=lo, to=hi, increment=step,
                             width=7, format="%.1f",
                             command=lambda n=name: self._on_spin(n))
            sp.bind("<Return>",      lambda e, n=name: self._on_spin(n))
            sp.bind("<FocusOut>",    lambda e, n=name: self._on_spin(n))
            sp.grid(row=0, column=4, padx=4)
            self._spinboxes[name] = sp

            # ゼロリセットボタン
            ttk.Button(row, text="↺", width=3,
                       command=lambda n=name: self._reset(n)).grid(row=0, column=5)

            row.columnconfigure(2, weight=1)

        # ── フィードバック（コンパクトテーブル） ──
        fr_fb = ttk.LabelFrame(self.root, text="フィードバック (MCU→PC)", padding=4)
        fr_fb.pack(fill="x", padx=8, pady=2)

        motor_names = ["RM1", "RM2", "LM1", "LM2", "SM1"]
        self.fb_ang_labels = {}
        self.fb_rpm_labels = {}

        fb_grid = ttk.Frame(fr_fb)
        fb_grid.pack(fill="x")

        for col, name in enumerate(motor_names):
            cell = ttk.Frame(fb_grid, relief="groove", borderwidth=1)
            cell.grid(row=0, column=col, padx=2, pady=2, sticky="ew")
            fb_grid.columnconfigure(col, weight=1)

            ttk.Label(cell, text=name, font=("", 9, "bold"),
                      anchor="center").pack(fill="x")
            lbl_ang = ttk.Label(cell, text="----°", width=8,
                                anchor="center", foreground="#0055aa")
            lbl_ang.pack()
            self.fb_ang_labels[name] = lbl_ang

            if name in ("RM1", "RM2"):
                lbl_rpm = ttk.Label(cell, text="-- rpm", width=8,
                                    anchor="center", foreground="#aa5500",
                                    font=("", 8))
                lbl_rpm.pack()
                self.fb_rpm_labels[name] = lbl_rpm

        # ── ステータスバー ──
        self.status_var = tk.StringVar(value="未接続")
        ttk.Label(self.root, textvariable=self.status_var,
                  relief="sunken", anchor="w",
                  padding=(6, 2)).pack(fill="x", padx=8, pady=(2, 4))

    # ─────────────────────────────────────────
    # スライダー / スピンボックス 同期
    # ─────────────────────────────────────────
    def _on_slider(self, name):
        """スライダー操作 → スピンボックスに値を反映"""
        val = self.motors[name]["val"].get()
        # スピンボックスは DoubleVar 共有なので自動更新されるが念のため
        sp = self._spinboxes.get(name)
        if sp:
            sp.set(f"{val:.1f}")

    def _on_spin(self, name):
        """スピンボックス入力 → クランプしてスライダーを追従させる"""
        lo, hi = self.motors[name]["range"]
        try:
            val = float(self._spinboxes[name].get())
        except ValueError:
            val = 0.0
        val = max(lo, min(hi, val))
        self.motors[name]["val"].set(val)

    def _nudge(self, name, sign):
        """微調整ボタン: step 単位で増減"""
        lo, hi = self.motors[name]["range"]
        step    = self.motors[name]["step"]
        current = self.motors[name]["val"].get()
        new_val = max(lo, min(hi, round(current + sign * step, 2)))
        self.motors[name]["val"].set(new_val)
        sp = self._spinboxes.get(name)
        if sp:
            sp.set(f"{new_val:.1f}")

    def _reset(self, name):
        """ゼロリセット"""
        self.motors[name]["val"].set(0.0)
        sp = self._spinboxes.get(name)
        if sp:
            sp.set("0.0")

    # ─────────────────────────────────────────
    # シリアル接続
    # ─────────────────────────────────────────
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        self.cb_ports['values'] = [p.device for p in ports]
        if self.cb_ports['values']:
            self.cb_ports.current(0)

    def toggle_connection(self):
        if not self.is_connected:
            port = self.cb_ports.get()
            if not port:
                messagebox.showerror("エラー", "COMポートを選択してください")
                return
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=0.1)
                self.is_connected = True
                self.is_running = True
                self.btn_connect.config(text="切断")
                self.cb_ports.config(state="disabled")
                self.status_var.set(f"接続中: {port}")
                threading.Thread(target=self.send_loop,    daemon=True).start()
                threading.Thread(target=self.receive_loop, daemon=True).start()
            except Exception as e:
                messagebox.showerror("接続エラー", str(e))
        else:
            self.is_running = False
            self.is_connected = False
            if self.serial_port:
                self.serial_port.close()
            self.btn_connect.config(text="接続")
            self.cb_ports.config(state="readonly")
            self.status_var.set("未接続")

    # ─────────────────────────────────────────
    # CRC-16 (Modbus)
    # ─────────────────────────────────────────
    def calc_crc16(self, data):
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        return crc

    # ─────────────────────────────────────────
    # 送信ループ (50 Hz)
    # ─────────────────────────────────────────
    def send_loop(self):
        while self.is_running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    rm1  = int(self.motors["RM1"]["val"].get() * 10)
                    rm2  = int(self.motors["RM2"]["val"].get() * 10)
                    lm1  = int(self.motors["LM1"]["val"].get() * 10)
                    lm2  = int(self.motors["LM2"]["val"].get() * 10)
                    sm1  = int(self.motors["SM1"]["val"].get() * 10)
                    mode = self.mode_var.get()

                    payload = struct.pack('<BBhhhhhb', 0xAA, 0x55,
                                         rm1, rm2, lm1, lm2, sm1, mode)
                    crc    = self.calc_crc16(payload)
                    packet = payload + struct.pack('<HB', crc, 0x0A)
                    self.serial_port.write(packet)
                except Exception as e:
                    print(f"送信エラー: {e}")
            time.sleep(0.02)

    # ─────────────────────────────────────────
    # 受信ループ (フィードバックパケット解析)
    # ─────────────────────────────────────────
    def receive_loop(self):
        rx_buf = bytearray()
        while self.is_running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    chunk = self.serial_port.read(64)
                    if chunk:
                        rx_buf.extend(chunk)
                        while len(rx_buf) >= 16:
                            idx = rx_buf.find(b'\xBB\x66')
                            if idx == -1:
                                rx_buf.clear()
                                break
                            if idx > 0:
                                del rx_buf[:idx]
                            if len(rx_buf) >= 16:
                                pkt = bytes(rx_buf[:16])
                                del rx_buf[:16]
                                self.parse_feedback(pkt)
                except Exception as e:
                    print(f"受信エラー: {e}")
            time.sleep(0.01)

    def parse_feedback(self, pkt):
        if pkt[0] != 0xBB or pkt[1] != 0x66:
            return
        names  = ["RM1", "RM2", "LM1", "LM2", "SM1"]
        angles = [struct.unpack_from('<h', pkt, 2 + i*2)[0] / 10.0 for i in range(5)]
        rpm1   = struct.unpack_from('<h', pkt, 12)[0]
        rpm2   = struct.unpack_from('<h', pkt, 14)[0]
        rpms   = {"RM1": rpm1, "RM2": rpm2}

        def update_ui():
            for i, name in enumerate(names):
                self.fb_ang_labels[name].config(text=f"{angles[i]:+.1f}°")
            for name, rpm in rpms.items():
                self.fb_rpm_labels[name].config(text=f"{rpm:+d} rpm")

        self.root.after(0, update_ui)


if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlApp(root)
    root.mainloop()