from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动tanh_controller节点，并加载yaml参数。"""
    pkg_share = get_package_share_directory('tanh_controller')
    params_file = os.path.join(pkg_share, 'config', 'tanh_controller.yaml')

    return LaunchDescription([
        Node(
            package='tanh_controller',
            executable='tanh_controller_node',
            name='tanh_controller',
            output='screen',
            parameters=[params_file],
        )
    ])

