"""加载默认参数并启动Sflare小游戏节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """返回使用仓库默认配置的单节点启动描述。"""
    package_share = get_package_share_directory('sflare_game_controller')
    config_path = os.path.join(package_share, 'config', 'game.yaml')
    return LaunchDescription([
        Node(
            package='sflare_game_controller',
            executable='game_node',
            name='sflare_game_node',
            output='screen',
            parameters=[config_path],
        ),
    ])
