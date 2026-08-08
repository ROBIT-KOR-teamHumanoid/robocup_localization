from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_dir = os.path.join(
        get_package_share_directory('robocup_localization'),
        'config',
        'config_localization.yaml'
    )

    localization_node = Node(
        package='robocup_localization',
        executable='robocup_localization',
        name='robocup_localization',
        output='screen',
        parameters=[config_dir],
    )

    return LaunchDescription([
        localization_node,
    ])
