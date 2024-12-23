import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true"
    )



    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )


    laser_driver = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sllidar_ros2"),
            "launch",
            "sllidar_a1_launch.py"
        ),
    )

    laser_odometry = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rf2o_laser_odometry"),
            "launch",
            "rf2o_laser_odometry.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        )
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    safety_stop = Node(
        package="bumperbot_utils",
        executable="safety_stop.py",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    
    return LaunchDescription([
        use_slam_arg,
        hardware_interface,
        laser_driver,
        laser_odometry,
        controller,
        joystick,
        safety_stop,
        localization,
        slam,
    ])