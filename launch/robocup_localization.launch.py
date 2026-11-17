import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def initial_pose_params() -> dict:
    """Not a package.xml dependency: robocup_task_planner already depends on this package
    (a cycle), so it is looked up at launch time and may be missing (poses default to 0)."""

    try:
        path = os.path.join(
            get_package_share_directory("robocup_task_planner"),
            "config",
            "state_pose.yaml",
        )
        with open(path) as handle:
            document = yaml.safe_load(handle)
    except Exception as error:
        print(
            "[robocup_localization] state_pose.yaml 을 읽지 못해 INITIAL 배치가"
            f" 비어 있다: {error}"
        )
        return {}

    poses = document["/**/task_planner_node"]["ros__parameters"]["initial_pose"]

    params = {}
    for name, pose in poses.items():
        for axis in ("x", "y", "yaw_deg"):
            params[f"initial_pose.{name}.{axis}"] = float(pose[axis])
    return params


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
        parameters=[config_dir, initial_pose_params()],
    )

    return LaunchDescription([
        localization_node,
    ])
