"""
nuc.launch.py
NUC側の全ノードを起動するランチファイル

起動ノード:
  - can_node         : USB-CAN変換器経由でモジュール回路と通信 (slcan直接方式)
  - serial_motor_node: マイコンとシリアル通信でロボマスモーターを制御
  - debug_node       : デバッグ・監視用ノード
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='catchrobo_nuc',
            executable='can_node',
            name='can_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='catchrobo_nuc',
            executable='serial_motor_node',
            name='serial_motor_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='catchrobo_nuc',
            executable='debug_node',
            name='debug_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
