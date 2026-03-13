import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    this_dir = os.path.dirname(os.path.abspath(__file__))
    default_params = os.path.join(this_dir, "ar_overlay.params.yaml")
    node_script = os.path.join(this_dir, "ar_overlay_node.py")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to AR overlay ROS2 parameter YAML",
    )

    run_overlay = ExecuteProcess(
        cmd=[
            "python3",
            node_script,
            "--ros-args",
            "--params-file",
            LaunchConfiguration("params_file"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            params_file_arg,
            run_overlay,
        ]
    )
