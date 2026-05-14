"""catchrobo_nuc launch: CAN・シリアルモーター・デバッグの3ノードを起動する"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='catchrobo_nuc',
            executable='can_node',
            name='can_node',
            output='screen',
        ),
        Node(
            package='catchrobo_nuc',
            executable='serial_motor_node',
            name='serial_motor_node',
            output='screen',
        ),
        Node(
            package='catchrobo_nuc',
            executable='debug_node',
            name='debug_node',
            output='screen',
            prefix='xterm -fullscreen -e',   # 全画面ターミナルで起動
        ),
    ])
