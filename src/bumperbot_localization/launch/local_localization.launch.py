from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("bumperbot_localization")

    ekf_local_config = os.path.join(pkg_dir, "config", "ekf.yaml")
    navsat_config = os.path.join(pkg_dir, "config", "navsat_transform.yaml")
    ekf_global_config = os.path.join(pkg_dir, "config", "ekf_global.yaml")

    imu_republisher = Node(
        package="bumperbot_localization",
        executable="imu_republisher.py",
        name="imu_republisher_node",
        output="screen",
    )

    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_local_config,
            {"use_sim_time": True}
        ],
        remappings=[
            ("/odometry/filtered", "/odometry/local")
        ]
    )

    gps_odom_republisher = Node(
    package="bumperbot_localization",
    executable="gps_odom_republisher.py",
    name="gps_odom_republisher_node",
    output="screen",
    parameters=[
        {"use_sim_time": True}
    ]
    )

    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global_filter_node",
        output="screen",
        parameters=[
            ekf_global_config,
            {"use_sim_time": True}
        ],
        remappings=[
            ("/odometry/filtered", "/odometry/global")
        ]
    )

    #gps_republisher = Node(
    #package="bumperbot_localization",
    #executable="gps_republisher.py",
    #name="gps_republisher_node",
    #output="screen",
    #)
    imu_sim_republisher = Node(
    package="bumperbot_localization",
    executable="imu_sim_republisher.py",
    name="imu_sim_republisher_node",
    output="screen",
    parameters=[
        {"use_sim_time": True}
    ],
    )

    return LaunchDescription([
        imu_republisher,
        ekf_local,
        gps_odom_republisher,
        ekf_global,
        #gps_republisher,
        imu_sim_republisher,
    ])