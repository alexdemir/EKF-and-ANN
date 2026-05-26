from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def noisy_controller(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    odom_error_profile = LaunchConfiguration("odom_error_profile").perform(context)
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))
    slip_start_sec = float(LaunchConfiguration("slip_start_sec").perform(context))
    slip_duration_sec = float(LaunchConfiguration("slip_duration_sec").perform(context))
    slip_linear_scale = float(LaunchConfiguration("slip_linear_scale").perform(context))
    slip_angular_scale = float(LaunchConfiguration("slip_angular_scale").perform(context))
    encoder_noise_std = float(LaunchConfiguration("encoder_noise_std").perform(context))
    linear_bias = float(LaunchConfiguration("linear_bias").perform(context))
    angular_bias = float(LaunchConfiguration("angular_bias").perform(context))
    yaw_random_walk_std = float(LaunchConfiguration("yaw_random_walk_std").perform(context))
    odom_pose_position_variance = float(LaunchConfiguration("odom_pose_position_variance").perform(context))
    odom_pose_yaw_variance = float(LaunchConfiguration("odom_pose_yaw_variance").perform(context))
    odom_twist_linear_variance = float(LaunchConfiguration("odom_twist_linear_variance").perform(context))
    odom_twist_angular_variance = float(LaunchConfiguration("odom_twist_angular_variance").perform(context))

    if odom_error_profile == "realistic":
        encoder_noise_std = 0.0008
        linear_bias = 1.01
        angular_bias = 1.02
        yaw_random_walk_std = 0.0005
        odom_pose_position_variance = 0.35
        odom_pose_yaw_variance = 0.35
        odom_twist_linear_variance = 0.15
        odom_twist_angular_variance = 0.15
    elif odom_error_profile == "rough":
        encoder_noise_std = 0.0012
        linear_bias = 1.02
        angular_bias = 1.035
        yaw_random_walk_std = 0.0008
        odom_pose_position_variance = 0.50
        odom_pose_yaw_variance = 0.50
        odom_twist_linear_variance = 0.20
        odom_twist_angular_variance = 0.20
    elif odom_error_profile != "manual":
        raise RuntimeError(
            "odom_error_profile must be one of: manual, realistic, rough"
        )

    noisy_controller_py = Node(
        package="bumperbot_controller",
        executable="noisy_controller.py",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,
             "wheel_separation": wheel_separation + wheel_separation_error,
             "slip_start_sec": slip_start_sec,
             "slip_duration_sec": slip_duration_sec,
             "slip_linear_scale": slip_linear_scale,
             "slip_angular_scale": slip_angular_scale,
             "encoder_noise_std": encoder_noise_std,
             "linear_bias": linear_bias,
             "angular_bias": angular_bias,
             "yaw_random_walk_std": yaw_random_walk_std,
             "odom_pose_position_variance": odom_pose_position_variance,
             "odom_pose_yaw_variance": odom_pose_yaw_variance,
             "odom_twist_linear_variance": odom_twist_linear_variance,
             "odom_twist_angular_variance": odom_twist_angular_variance,
             "use_sim_time": use_sim_time}],
    )

    return [noisy_controller_py]


def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )
    odom_error_profile_arg = DeclareLaunchArgument(
        "odom_error_profile",
        default_value="manual",
        description="manual keeps the explicit odom parameters. realistic/rough apply repeatable odometry error profiles.",
    )
    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True",
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17",
    )
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.0",
    )
    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.0",
    )
    slip_start_sec_arg = DeclareLaunchArgument(
        "slip_start_sec",
        default_value="-1.0",
    )
    slip_duration_sec_arg = DeclareLaunchArgument(
        "slip_duration_sec",
        default_value="0.0",
    )
    slip_linear_scale_arg = DeclareLaunchArgument(
        "slip_linear_scale",
        default_value="1.0",
    )
    slip_angular_scale_arg = DeclareLaunchArgument(
        "slip_angular_scale",
        default_value="1.0",
    )
    encoder_noise_std_arg = DeclareLaunchArgument(
        "encoder_noise_std",
        default_value="0.0002",
    )
    linear_bias_arg = DeclareLaunchArgument(
        "linear_bias",
        default_value="1.0",
    )
    angular_bias_arg = DeclareLaunchArgument(
        "angular_bias",
        default_value="1.0",
    )
    yaw_random_walk_std_arg = DeclareLaunchArgument(
        "yaw_random_walk_std",
        default_value="0.0002",
    )
    odom_pose_position_variance_arg = DeclareLaunchArgument(
        "odom_pose_position_variance",
        default_value="0.25",
    )
    odom_pose_yaw_variance_arg = DeclareLaunchArgument(
        "odom_pose_yaw_variance",
        default_value="0.25",
    )
    odom_twist_linear_variance_arg = DeclareLaunchArgument(
        "odom_twist_linear_variance",
        default_value="0.10",
    )
    odom_twist_angular_variance_arg = DeclareLaunchArgument(
        "odom_twist_angular_variance",
        default_value="0.10",
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # HATALI KISIM BURASIYDI: Condition yapısını UnlessCondition ile düzelttim
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bumperbot_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller),
    )

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["simple_velocity_controller", 
                        "--controller-manager", 
                        "/controller_manager"
                ]
            ),
            Node(
                package="bumperbot_controller",
                executable="simple_controller.py",
                parameters=[
                    {"wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation,
                    "use_sim_time": use_sim_time}],
            ),
        ]
    )

    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    return LaunchDescription(
        [
            use_sim_time_arg,
            odom_error_profile_arg,
            use_simple_controller_arg,
            wheel_radius_arg,
            wheel_separation_arg,
            wheel_radius_error_arg,
            wheel_separation_error_arg,
            slip_start_sec_arg,
            slip_duration_sec_arg,
            slip_linear_scale_arg,
            slip_angular_scale_arg,
            encoder_noise_std_arg,
            linear_bias_arg,
            angular_bias_arg,
            yaw_random_walk_std_arg,
            odom_pose_position_variance_arg,
            odom_pose_yaw_variance_arg,
            odom_twist_linear_variance_arg,
            odom_twist_angular_variance_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            simple_controller,
            noisy_controller_launch,
        ]
    )
