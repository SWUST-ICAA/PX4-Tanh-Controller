from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动figure8_path_publisher节点，并加载yaml参数。"""
    pkg_share = get_package_share_directory('figure8_path_publisher')
    params_file = os.path.join(pkg_share, 'config', 'figure8_path_publisher.yaml')

    return LaunchDescription([
        Node(
            package='figure8_path_publisher',
            executable='figure8_path_publisher_node',
            name='figure8_path_publisher',
            output='screen',
            parameters=[params_file],
        )
    ])

