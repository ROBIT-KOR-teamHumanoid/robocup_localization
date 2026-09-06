import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def initial_pose_params() -> dict:
    """robocup_task_planner 의 INITIAL 배치를 이 노드의 파라미터로 옮긴다.
    package.xml 에 robocup_task_planner 를 적지 않는다: 그쪽이 이미 이 패키지의
    메시지에 의존하고 있어서 반대 방향을 걸면 순환이 되어 colcon 이 빌드
    순서를 못 정한다. 런치 시점의 경로 조회라 빌드 순서와는 무관하지만, 그만큼
    없을 수도 있으므로 못 찾으면 경고만 남기고 넘어간다 (그 경우 배치 값이
    0 이 되어 자동 배치 버튼이 필드 중앙을 가리킨다).
    """

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
