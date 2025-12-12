from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """同时启动8字轨迹发布器与tanh控制器。"""
    tanh_share = get_package_share_directory('tanh_controller')
    fig8_share = get_package_share_directory('figure8_path_publisher')

    tanh_params = os.path.join(tanh_share, 'config', 'tanh_controller.yaml')
    fig8_params = os.path.join(fig8_share, 'config', 'figure8_path_publisher.yaml')

    return LaunchDescription([
        Node(
            package='figure8_path_publisher',
            executable='figure8_path_publisher_node',
            name='figure8_path_publisher',
            output='screen',
            parameters=[fig8_params],
        ),
        Node(
            package='tanh_controller',
            executable='tanh_controller_node',
            name='tanh_controller',
            output='screen',
            parameters=[tanh_params],
        )
    ])

