"""
sound_node.py
Open JTalk (オフライン日本語音声合成) を用いたNUC側音声アナウンスノード

監視トピック:
  - /catchrobo/speech_cmd    (String)      : 任意テキストの即時発話
  - /catchrobo/can_status    (String JSON) : CAN接続・切断状態の変化検知
  - /catchrobo/serial_status (String JSON) : モーターシリアル接続・切断状態の変化検知
  - /catchrobo/motor_mode    (String)      : モーター制御モード変化検知
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import subprocess
import threading
import queue
import tempfile
import os
import json
import time
import shutil
from pathlib import Path


class SoundNode(Node):
    def __init__(self):
        super().__init__('sound_node')

        # ─── ROS 2 パラメータ ─────────────────────────
        # オーディオ出力デバイス ('default' で Ubuntu 設定の標準スピーカー/モニターを使用)
        # 必要に応じて 'plughw:1,0' 等の ALSA デバイス名も指定可能
        self.declare_parameter('audio_device', 'default')
        self.declare_parameter('voice_model', '/usr/share/hts-voice/nitech-jp-atr503-m001/nitech_jp_atr503_m001.htsvoice')
        self.declare_parameter('dic_dir', '/var/lib/mecab/dic/open-jtalk/naist-jdic')
        self.declare_parameter('speech_speed', 1.05)  # 話速 (1.0 = 標準)

        self.audio_device = self.get_parameter('audio_device').get_parameter_value().string_value
        self.voice_model = self.get_parameter('voice_model').get_parameter_value().string_value
        self.dic_dir = self.get_parameter('dic_dir').get_parameter_value().string_value
        self.speech_speed = self.get_parameter('speech_speed').get_parameter_value().double_value

        # 女性音声モデル (Mei) を検索して最優先で適用
        mei_candidates = [
            Path(__file__).resolve().parent / 'voice' / 'mei_normal.htsvoice',
            Path('/home/altair/catchrobo2026-nuc/src/catchrobo_nuc/catchrobo_nuc/voice/mei_normal.htsvoice'),
        ]
        for candidate in mei_candidates:
            if candidate.exists():
                self.voice_model = str(candidate)
                self.get_logger().info(f'女性音声モデル (Mei) を適用: {self.voice_model}')
                break

        # ─── 状態管理（前回状態の保持・チャタリング防止） ───
        self.last_can_connected = None
        self.last_serial_connected = None
        self.last_motor_mode = None
        self.state_init_time = time.time()

        # ─── 発声キュー & 再生ワーカースレッド ─────────
        self.speech_queue = queue.Queue()
        self.is_running = True
        self.worker_thread = threading.Thread(target=self._speech_worker, daemon=True)
        self.worker_thread.start()

        # ─── サブスクライバー ──────────────────────────
        self.create_subscription(String, '/catchrobo/speech_cmd', self._on_speech_cmd, 10)
        self.create_subscription(String, '/catchrobo/can_status', self._on_can_status, 10)
        self.create_subscription(String, '/catchrobo/serial_status', self._on_serial_status, 10)
        self.create_subscription(String, '/catchrobo/motor_mode', self._on_motor_mode, 10)

        # 起動アナウンス
        self.speak('システムを起動しました。アームろう、いきまーす！')
        self.get_logger().info(f'サウンドノード 起動完了 (Device: {self.audio_device})')

    def speak(self, text: str):
        """発話キューにテキストを追加"""
        if not text:
            return
        self.get_logger().info(f'発話リクエスト: {text}')
        self.speech_queue.put(text)

    # ─────────────────────────────────────────────────
    # ROS 2 コールバック
    # ─────────────────────────────────────────────────

    def _on_speech_cmd(self, msg: String):
        """任意発話コマンドを受信"""
        text = msg.data.strip()
        if text:
            self.speak(text)

    def _on_can_status(self, msg: String):
        """CAN通信状態の変化を検知"""
        try:
            data = json.loads(msg.data)
            connected = bool(data.get('connected', False))

            # 初回受信時の初期化
            if self.last_can_connected is None:
                self.last_can_connected = connected
                if connected:
                    self.speak('キャン通信、接続完了')
                return

            # 状態変化時
            if connected != self.last_can_connected:
                self.last_can_connected = connected
                if connected:
                    self.speak('キャン通信を接続しました')
                else:
                    self.speak('警告、キャン通信が切断されました')
        except Exception as e:
            self.get_logger().error(f'CANステータス解析エラー: {e}')

    def _on_serial_status(self, msg: String):
        """モーターシリアル通信状態の変化を検知"""
        try:
            data = json.loads(msg.data)
            connected = bool(data.get('connected', False))

            if self.last_serial_connected is None:
                self.last_serial_connected = connected
                if connected:
                    self.speak('モーター通信、接続完了')
                return

            if connected != self.last_serial_connected:
                self.last_serial_connected = connected
                if connected:
                    self.speak('モーター通信を接続しました')
                else:
                    self.speak('警告、モーター通信が切断されました')
        except Exception as e:
            self.get_logger().error(f'シリアルステータス解析エラー: {e}')

    def _on_motor_mode(self, msg: String):
        """モーター制御モード変化を検知"""
        raw_str = msg.data.strip()
        mode_val = None

        # JSON形式の解析 (例: {"mode": 1})
        try:
            parsed = json.loads(raw_str)
            if isinstance(parsed, dict) and 'mode' in parsed:
                mode_val = str(parsed['mode'])
            elif isinstance(parsed, (int, str)):
                mode_val = str(parsed)
        except Exception:
            mode_val = raw_str

        if mode_val is None:
            mode_val = raw_str

        # 重複・変化なしチェック
        if mode_val == self.last_motor_mode:
            return
        self.last_motor_mode = mode_val

        # 起動直後(3秒以内)は初期モード通知をスキップ
        if time.time() - self.state_init_time < 3.0:
            return

        mode_text_map = {
            '0': 'モーター停止',
            '1': 'ピーアイディー制御モード',
            '2': 'オープンループモード',
            'STOP': 'モーター停止',
            'PID': 'ピーアイディー制御モード',
            'OPEN_LOOP': 'オープンループモード',
        }
        speech_text = mode_text_map.get(mode_val)
        if speech_text:
            self.speak(speech_text)

    # ─────────────────────────────────────────────────
    # 音声合成 & 再生ワーカー
    # ─────────────────────────────────────────────────

    def _speech_worker(self):
        """キューからテキストを取り出して順次合成・再生するワーカースレッド"""
        while self.is_running:
            try:
                text = self.speech_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            self._synthesize_and_play(text)
            self.speech_queue.task_done()

    def _synthesize_and_play(self, text: str):
        """Open JTalk で一時WAVファイルを生成し、aplayで再生"""
        # Open JTalkの存在確認
        if not os.path.exists('/usr/bin/open_jtalk'):
            self.get_logger().warn(
                f'open_jtalk コマンドが見つかりません。発話をスキップします: "{text}"'
            )
            return

        # 辞書・音声モデルの確認
        if not os.path.exists(self.voice_model):
            self.get_logger().warn(
                f'音声モデルが見つかりません ({self.voice_model})。発話をスキップします。'
            )
            return

        if not os.path.exists(self.dic_dir):
            self.get_logger().warn(
                f'辞書ディレクトリが見つかりません ({self.dic_dir})。発話をスキップします。'
            )
            return

        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp_wav:
            wav_path = tmp_wav.name

        try:
            # 1. 音声合成 (open_jtalk)
            cmd_jtalk = [
                'open_jtalk',
                '-m', self.voice_model,
                '-x', self.dic_dir,
                '-r', str(self.speech_speed),
                '-ow', wav_path,
            ]
            proc = subprocess.run(
                cmd_jtalk,
                input=text.encode('utf-8'),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=5.0,
            )
            if proc.returncode != 0:
                self.get_logger().error(
                    f'Open JTalk エラー ({proc.returncode}): {proc.stderr.decode(errors="ignore")}'
                )
                return

            # 2. 再生
            # audio_device が 'default' または未指定の場合は、PulseAudio の paplay を優先
            # (Ubuntu のサウンド設定で指定されたモニター等の標準スピーカーから鳴る)
            played = False
            if self.audio_device in ('default', ''):
                if shutil.which('paplay'):
                    res = subprocess.run(['paplay', wav_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10.0)
                    if res.returncode == 0:
                        played = True
                if not played:
                    res = subprocess.run(['aplay', '-q', wav_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10.0)
                    if res.returncode == 0:
                        played = True
            else:
                # 明示的なALSAデバイス指定 (例: plughw:1,0)
                res = subprocess.run(['aplay', '-q', '-D', self.audio_device, wav_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10.0)
                if res.returncode == 0:
                    played = True
                else:
                    self.get_logger().warn(f'デバイス {self.audio_device} での再生に失敗。標準デバイスで再試行します。')
                    if shutil.which('paplay'):
                        subprocess.run(['paplay', wav_path], timeout=10.0)
                    else:
                        subprocess.run(['aplay', '-q', wav_path], timeout=10.0)

        except subprocess.TimeoutExpired:
            self.get_logger().error('音声合成/再生がタイムアウトしました。')
        except Exception as e:
            self.get_logger().error(f'音声再生例外: {e}')
        finally:
            # 一時ファイル削除
            if os.path.exists(wav_path):
                try:
                    os.remove(wav_path)
                except OSError:
                    pass

    def destroy_node(self):
        self.is_running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SoundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
