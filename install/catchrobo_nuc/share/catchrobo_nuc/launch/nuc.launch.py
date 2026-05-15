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
from launch.actions import ExecuteProcess

# source済みのsetupスクリプトを引き継いで新タブでdebug_nodeを起動するコマンド
_DEBUG_CMD = (
    'source /opt/ros/humble/setup.bash'
    ' && source "$(dirname $(dirname $(which ros2)))/../catchrobo2026-nuc/install/setup.bash" 2>/dev/null'
    ' || source ~/catchrobo2026-nuc/install/setup.bash;'
    ' ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}'
    ' ros2 run catchrobo_nuc debug_node;'
    ' echo "[終了] Enterで閉じる"; read'
)

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
        # デバッグモニターを WezTerm の新タブで起動
        ExecuteProcess(
            cmd=['wezterm', 'cli', 'spawn', '--', 'bash', '-c', _DEBUG_CMD],
            output='screen',
            shell=False,
        ),
    ])
