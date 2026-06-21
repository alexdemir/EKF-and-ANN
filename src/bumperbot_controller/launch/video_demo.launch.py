import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bumperbot_description = get_package_share_directory("bumperbot_description")
    bumperbot_localization = get_package_share_directory("bumperbot_localization")
    bumperbot_controller = get_package_share_directory("bumperbot_controller")

    laps_arg = DeclareLaunchArgument(
        "scripted_laps",
        default_value="1",
        description="Number of scripted route laps for the video demo.",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Use false for recording the Gazebo GUI.",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bumperbot_description, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    localization = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        bumperbot_localization,
                        "launch",
                        "local_localization.launch.py",
                    )
                ),
                launch_arguments={
                    "run_ann_pseudo": "true",
                    "run_ann_trainer": "false",
                    "run_ekf_logger": "true",
                    "ann_forced_dropout_windows": "18:8,34:14,54:12",
                    "ann_force_gps_dropout_after_sec": "-1.0",
                    "ann_force_gps_dropout_duration_sec": "0.0",
                }.items(),
            )
        ],
    )

    controller = TimerAction(
        period=9.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bumperbot_controller, "launch", "controller.launch.py")
                ),
                launch_arguments={
                    "run_scripted_trajectory": "true",
                    "scripted_trajectory_profile": "paper_mismatch_waypoint",
                    "scripted_speed_scale": "1.0",
                    "scripted_laps": LaunchConfiguration("scripted_laps"),
                    "paper_slip_mode": "true",
                    "scripted_physical_slip_start_sec": "38.0",
                    "scripted_physical_slip_duration_sec": "18.0",
                    "scripted_physical_slip_linear_scale": "0.0",
                    "scripted_physical_slip_angular_scale": "0.0",
                    "slip_start_sec": "38.0",
                    "slip_duration_sec": "18.0",
                    "slip_linear_scale": "2.5",
                    "slip_angular_scale": "2.5",
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        laps_arg,
        headless_arg,
        gazebo,
        localization,
        controller,
    ])
