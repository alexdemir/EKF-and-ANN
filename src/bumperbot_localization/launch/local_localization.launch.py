from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("bumperbot_localization")
    run_legacy_filters = LaunchConfiguration("run_legacy_filters")
    run_ann_pseudo = LaunchConfiguration("run_ann_pseudo")
    run_ann_trainer = LaunchConfiguration("run_ann_trainer")
    run_ekf_logger = LaunchConfiguration("run_ekf_logger")
    ann_force_gps_dropout_after_sec = LaunchConfiguration("ann_force_gps_dropout_after_sec")
    ann_force_gps_dropout_duration_sec = LaunchConfiguration("ann_force_gps_dropout_duration_sec")

    ekf_local_config = os.path.join(pkg_dir, "config", "ekf.yaml")
    navsat_config = os.path.join(pkg_dir, "config", "navsat_transform.yaml")
    ekf_global_config = os.path.join(pkg_dir, "config", "ekf_global.yaml")
    ekf_gps_imu_config = os.path.join(pkg_dir, "config", "ekf_gps_imu.yaml")
    ekf_gps_odom_config = os.path.join(pkg_dir, "config", "ekf_gps_odom.yaml")
    complementary_filter_config = os.path.join(pkg_dir, "config", "complementary_filter.yaml")
    ann_trainer_config = os.path.join(pkg_dir, "config", "ann_trainer.yaml")
    ann_pseudo_gps_config = os.path.join(pkg_dir, "config", "ann_pseudo_gps.yaml")
    gps_hold_config = os.path.join(pkg_dir, "config", "gps_hold.yaml")
    fuzzy_localization_config = os.path.join(pkg_dir, "config", "fuzzy_localization.yaml")
    ekf_output_logger_config = os.path.join(pkg_dir, "config", "ekf_output_logger.yaml")

    imu_republisher = Node(
        package="bumperbot_localization",
        executable="imu_republisher.py",
        name="imu_republisher_node",
        output="screen",
        condition=IfCondition(run_legacy_filters),
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
        ],
        condition=IfCondition(run_legacy_filters),
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

    ekf_gps_imu = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_gps_imu_filter_node",
        output="screen",
        parameters=[
            ekf_gps_imu_config,
            {"use_sim_time": True}
        ],
        remappings=[
            ("/odometry/filtered", "/odometry/kf_gps_imu")
        ]
    )

    ekf_gps_odom = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_gps_odom_filter_node",
        output="screen",
        parameters=[
            ekf_gps_odom_config,
            {"use_sim_time": True}
        ],
        remappings=[
            ("/odometry/filtered", "/odometry/kf_gps_odom")
        ]
    )

    complementary_filter = Node(
        package="bumperbot_localization",
        executable="complementary_filter.py",
        name="complementary_filter_node",
        output="screen",
        parameters=[
            complementary_filter_config,
            {"use_sim_time": True}
        ],
    )

    ann_trainer = Node(
        package="bumperbot_localization",
        executable="ann_trainer.py",
        name="ann_trainer_node",
        output="screen",
        parameters=[
            ann_trainer_config,
            {
                "force_gps_dropout_after_sec": ann_force_gps_dropout_after_sec,
                "force_gps_dropout_duration_sec": ann_force_gps_dropout_duration_sec,
            },
            {"use_sim_time": True}
        ],
        condition=IfCondition(run_ann_trainer),
    )

    ann_pseudo_gps = Node(
        package="bumperbot_localization",
        executable="ann_pseudo_gps.py",
        name="ann_pseudo_gps_node",
        output="screen",
        parameters=[
            ann_pseudo_gps_config,
            {
                "force_gps_dropout_after_sec": ann_force_gps_dropout_after_sec,
                "force_gps_dropout_duration_sec": ann_force_gps_dropout_duration_sec,
            },
            {"use_sim_time": True}
        ],
        condition=IfCondition(run_ann_pseudo),
    )

    gps_hold = Node(
        package="bumperbot_localization",
        executable="gps_hold.py",
        name="gps_hold_node",
        output="screen",
        parameters=[
            gps_hold_config,
            {
                "force_gps_dropout_after_sec": ann_force_gps_dropout_after_sec,
                "force_gps_dropout_duration_sec": ann_force_gps_dropout_duration_sec,
            },
            {"use_sim_time": True}
        ],
    )

    fuzzy_localization = Node(
        package="bumperbot_localization",
        executable="fuzzy_localization.py",
        name="fuzzy_localization_node",
        output="screen",
        parameters=[
            fuzzy_localization_config,
            {"use_sim_time": True}
        ],
        condition=IfCondition(run_ann_pseudo),
    )

    ekf_output_logger = Node(
        package="bumperbot_localization",
        executable="ekf_output_logger.py",
        name="ekf_output_logger_node",
        output="screen",
        parameters=[
            ekf_output_logger_config,
            {"use_sim_time": True}
        ],
        condition=IfCondition(run_ekf_logger),
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
        ],
        condition=IfCondition(run_legacy_filters),
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
        DeclareLaunchArgument(
            "run_legacy_filters",
            default_value="false",
            description="Run the old local/global EKF nodes in addition to the paper-style ANN training pipeline.",
        ),
        DeclareLaunchArgument(
            "run_ann_pseudo",
            default_value="true",
            description="Run the saved ANN pseudo-GPS node for GPS dropout testing.",
        ),
        DeclareLaunchArgument(
            "run_ann_trainer",
            default_value="true",
            description="Run online ANN trainer. Set false when testing a saved model only.",
        ),
        DeclareLaunchArgument(
            "run_ekf_logger",
            default_value="true",
            description="Log EKF, ANN pseudo-GPS, odometry, and GPS outputs for evaluation.",
        ),
        DeclareLaunchArgument(
            "ann_force_gps_dropout_after_sec",
            default_value="15.0",
            description="Seconds after pseudo-GPS startup before ignoring GPS and publishing ANN.",
        ),
        DeclareLaunchArgument(
            "ann_force_gps_dropout_duration_sec",
            default_value="0.0",
            description="Forced GPS dropout duration. Use 0.0 to keep dropout active after it starts.",
        ),
        imu_republisher,
        ekf_local,
        gps_odom_republisher,
        gps_hold,
        ekf_gps_imu,
        ekf_gps_odom,
        complementary_filter,
        ann_trainer,
        ann_pseudo_gps,
        fuzzy_localization,
        ekf_output_logger,
        ekf_global,
        #gps_republisher,
        imu_sim_republisher,
    ])
