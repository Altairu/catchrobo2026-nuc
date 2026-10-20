# catchrobo2026-nuc

Catchrobo 2026 NUC側 ROS2 ワークスペース。

USB-CAN経由でモジュール回路（MDD/Solenoid）と通信し、シリアル経由でロボマスモーターを制御する。
Open JTalkによる音声アナウンス・対話ノードやデバッグTUIノードを含め、合計4つのノードで構成される。

---

## システム構成

```
catchrobo2026-nuc/
├── src/
│   └── catchrobo_nuc/          # ROS2 パッケージ
│       └── catchrobo_nuc/
│           ├── can_node.py          # USB-CAN 通信ノード
│           ├── serial_motor_node.py # ロボマスモーター シリアル通信ノード
│           ├── sound_node.py        # 音声アナウンス・対話発話ノード (Open JTalk)
│           ├── voice/               # 音声合成モデル (.htsvoice)
│           └── debug_node.py        # curses 全画面デバッグTUI ノード
├── launch/
│   └── nuc.launch.py           # ノード同時起動ファイル
└── sample/
    └── motor_control_gui.py    # ロボマスモーター 動作確認用サンプル (tkinter GUI)
```

---

## ノード詳細

### 1. `can_node` — USB-CAN 通信ノード

| 項目       | 内容                                |
| ---------- | ----------------------------------- |
| 通信方式   | slcan プロトコル (USB-CANアダプタ)  |
| 送信レート | 100Hz (MDD) / 10Hz (Solenoid)       |
| 接続方法   | PCのGUIからポートを指定して動的接続 |

**対象モジュール:**

| 名前   | 種類                      | Base CAN ID   |
| ------ | ------------------------- | ------------- |
| MDD1   | Motor Driver Driver (4ch) | `0x200` (512) |
| SV_1   | Solenoid Valve (12ch)     | `0x300` (768) |
| SV_2   | Solenoid Valve (12ch)     | `0x301` (769) |
| Servo1 | Servo Motor (6ch)         | `0x100` (256) |

**CAN フレーム仕様 (MDD1 / BaseID=0x200):**

| CAN ID          | 方向 | 内容                                      |
| --------------- | ---- | ----------------------------------------- |
| `0x200`~`0x203` | TX   | モーターごとのPIDパラメータ (8byte)       |
| `0x210`         | TX   | 動作モード (4byte)                        |
| `0x220`         | TX   | 目標値 (int16×4, 8byte)                   |
| `0x230`         | RX   | ステータス [SW×4, Err, AppMode]           |
| `0x240`         | RX   | エンコーダ角度 (int16×4, 単位: 0.1deg)    |
| `0x250`         | RX   | エンコーダ角速度 (int16×4, 単位: 0.01rps) |

**Solenoid (SV_1=0x300, SV_2=0x301):**

| CAN ID            | 方向 | 内容                                          |
| ----------------- | ---- | --------------------------------------------- |
| `0x300` / `0x301` | TX   | バルブ状態ビットフィールド (uint16 LE, 2byte) |

---

### 2. `serial_motor_node` — ロボマスモーター シリアルノード

| 項目       | 内容                                    |
| ---------- | --------------------------------------- |
| 通信方式   | UART (115200bps)                        |
| 送信レート | 50Hz                                    |
| 対象軸     | RM1 / RM2 / LM1 / LM2 / SM1 / LM3 (6軸) |

**送信パケット (18byte):**

```
[0xAA][0x55][RM1 int16LE][RM2 int16LE][LM1 int16LE][LM2 int16LE][SM1 int16LE][LM3 int16LE][Mode int8][CRC16 LE][0x0A]
```

- 角度値は `degree × 10` の整数で送信
- CRC16 は Modbus 方式（先頭15バイトに対して計算）

**受信パケット (18byte):**

```
[0xBB][0x66][Ang1~6 int16LE×6][RPM1 int16LE][RPM2 int16LE]
```

**制御モード:**

| 値  | モード           |
| --- | ---------------- |
| `0` | 停止             |
| `1` | PID 制御         |
| `2` | 同定（開ループ） |

---

### 3. `sound_node` — 音声アナウンス & 音声対話ノード

Open JTalk を用いたオフライン音声合成ノード。ロボットの状態変化アナウンスやPC側Gemini対話音声の再生を行う。

| 項目       | 内容                                                              |
| ---------- | ----------------------------------------------------------------- |
| 合成方式   | Open JTalk (HTS Voice)                                            |
| 再生方式   | PulseAudio (`paplay`) / ALSA (`aplay`)                            |
| 音声モデル | 東北大学モデル (`tohoku-f01-neutral`) / Mei (`mei_normal`)        |
| 話速       | 5.0 (標準〜高速アナウンス)                                        |

**2つの動作モード:**

| モード               | 動作概要                                                                                                  |
| -------------------- | --------------------------------------------------------------------------------------------------------- |
| **通常モード (`NORMAL`)** | CAN/シリアル接続・切断、制御モード変更の自動アナウンス ＋ 約1分ごとのランダム雑談アナウンス（関西弁）    |
| **会話モード (`CHAT`)**   | PC側マイク対話優先モード。ランダム雑談タイマーを自動抑制し、`/catchrobo/speech_cmd` の返答音声を即時再生 |

- `/catchrobo/sound_mode` を受信することで動的にモードを切り替え可能。
- 重要警告（通信断など）はどのモードでも最優先で発話されます。

---

### 4. `debug_node` — 全画面デバッグ TUI

- `curses` ライブラリによる全画面ターミナルUI
- `q` キーで終了
- 表示内容:
  - CAN / シリアル 接続状態・ポート名
  - ロボマスモーター 角度・RPM・制御モード
  - MDD1 AppMode・スイッチ状態・エンコーダ値
  - SV_1 / SV_2 バルブ状態 (12bit ビット表示)
  - TX / RX / ERR カウンタ・受信レート

---

## ROS2 トピック一覧

### Subscribe (受信)

| トピック                 | 型                  | 送信元 | 内容                                                |
| ------------------------ | ------------------- | ------ | --------------------------------------------------- |
| `/catchrobo/motor_cmd`   | `Float32MultiArray` | PC     | 6軸目標値 [RM1..LM3] (degree)                       |
| `/catchrobo/motor_mode`  | `String` (JSON)     | PC     | 制御モード `{"mode": 0}`                            |
| `/catchrobo/module_cmd`  | `String` (JSON)     | PC     | MDD/Solenoid 操作コマンド                           |
| `/catchrobo/set_ports`   | `String` (JSON)     | PC     | `{"can_port":"...", "serial_port":"..."}`           |
| `/catchrobo/speech_cmd`  | `String`            | PC/他  | 任意発話テキスト (Open JTalkで合成・再生)           |
| `/catchrobo/sound_mode`  | `String`            | PC     | 音声モード切替 (`"NORMAL"` または `"CHAT"`)         |

### Publish (送信)

| トピック                     | 型                  | 受信先      | 内容                                           |
| ---------------------------- | ------------------- | ----------- | ---------------------------------------------- |
| `/catchrobo/motor_fb`        | `Float32MultiArray` | PC          | 角度×6 + RPM×2                                 |
| `/catchrobo/can_status`      | `String` (JSON)     | PC/デバッグ | CANノード状態・統計                            |
| `/catchrobo/serial_status`   | `String` (JSON)     | PC/デバッグ | シリアルノード状態・統計                       |
| `/catchrobo/available_ports` | `String` (JSON)     | PC          | 利用可能シリアルポート一覧                     |
| `/catchrobo/sound_status`    | `String` (JSON)     | PC          | サウンドノード状態・現在のモード (`mode` など) |

---

## 環境要件

| 項目        | バージョン/内容                           |
| ----------- | ----------------------------------------- |
| OS          | Ubuntu 22.04                              |
| ROS2        | Humble Hawksbill                          |
| Python      | 3.10                                      |
| pyserial    | `pip3 install pyserial`                   |
| Open JTalk  | `sudo apt install open-jtalk open-jtalk-mecab-naist-jdic pulseaudio-utils` |
| WezTerm     | [インストール済み] (デバッグTUI用)        |

---

## ビルド & 起動

### 初回ビルド

```bash
cd ~/catchrobo2026-nuc
source /opt/ros/humble/setup.bash
colcon build --packages-select catchrobo_nuc
```

### 起動 (通常)

```bash
cd ~/catchrobo2026-nuc
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=0 ros2 launch catchrobo_nuc nuc.launch.py
```

※ `debug_node` は自動的に WezTerm の別ウィンドウ/タブで起動します。

### ノードを個別に起動する場合

```bash
# ターミナル 1: CAN ノード
ROS_DOMAIN_ID=0 ros2 run catchrobo_nuc can_node

# ターミナル 2: シリアルモーターノード
ROS_DOMAIN_ID=0 ros2 run catchrobo_nuc serial_motor_node

# ターミナル 3: 音声アナウンス・対話発話ノード
ROS_DOMAIN_ID=0 ros2 run catchrobo_nuc sound_node

# ターミナル 4: デバッグTUI
ROS_DOMAIN_ID=0 ros2 run catchrobo_nuc debug_node
```

---

## サンプルコード (sample/)

### `motor_control_gui.py`

ロボマスモーターの動作確認用 tkinter GUI。ROS2不要で単体動作。

```bash
cd ~/catchrobo2026-nuc/sample
python3 motor_control_gui.py
```

- シリアルポートを選択して「接続」ボタンを押す
- スライダーで各軸の目標角度を設定
- モードを PID / 開ループ に切り替えて制御

---

## トラブルシューティング

### シリアルポートへのアクセス権限エラー

```bash
sudo usermod -aG dialout $USER
# ログアウト・ログインで反映
```

### CAN 接続が認識されない

```bash
# 接続済みポートを確認
ls /dev/ttyUSB* /dev/ttyACM*

# デバイスのベンダーIDを確認
lsusb
```

### ROS2 がPCのトピックを受信できない

```bash
# 両端末で同じ DOMAIN_ID を設定する
export ROS_DOMAIN_ID=0

# トピックが飛んでいるか確認
ros2 topic list
ros2 topic echo /catchrobo/can_status
```

---

## 関連リポジトリ

- [catchrobo2026-pc](../catchrobo2026-pc) — PC側コントローラー WebGUI
- [Altair_module_system_control](../Documents/Altair_module_system_control) — モジュール単体動作確認ツール (Windows/Chrome向け)
