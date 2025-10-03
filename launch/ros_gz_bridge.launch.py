from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

ARGUMENTS = [
    DeclareLaunchArgument(
        "namespace", default_value="empty_namespace", description="Robot namespace"
    ),
    DeclareLaunchArgument("world", default_value="unspecified", description="World name"),
]


def generate_launch_description():

    ros_gz_bridge = GroupAction(
        [
            PushRosNamespace(LaunchConfiguration("namespace")),
            # TODO: Update to ROS Kilted will add support for custom URDF frames making this file unnecessary
            # The new launchfile for models integrates the ros_gz_bridge https://github.com/gazebosim/ros_gz/blob/kilted/ros_gz_sim/launch/ros_gz_spawn_model.launch.py
            # Depth Camera
            # Lidar
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="rplidar_bridge",
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    [
                        "/world/",
                        LaunchConfiguration("world"),
                        "/model/",
                        LaunchConfiguration("namespace"),
                        "/link/",
                        "base_link/sensor/rplidar/scan"
                        + "@sensor_msgs/msg/LaserScan"
                        + "[gz.msgs.LaserScan",
                    ],
                    [
                        "/world/",
                        LaunchConfiguration("world"),
                        "/model/",
                        LaunchConfiguration("namespace"),
                        "/link/",
                        "base_link/sensor/rplidar/scan/points"
                        + "@sensor_msgs/msg/PointCloud2"
                        + "[gz.msgs.PointCloudPacked",
                    ],
                ],
                remappings=[
                    (
                        [
                            "/world/",
                            LaunchConfiguration("world"),
                            "/model/",
                            LaunchConfiguration("namespace"),
                            "/link/",
                            "base_link/sensor/rplidar/scan",
                        ],
                        ["sensor/lidar/scan"],
                    ),
                    (
                        [
                            "/world/",
                            LaunchConfiguration("world"),
                            "/model/",
                            LaunchConfiguration("namespace"),
                            "/link/",
                            "base_link/sensor/rplidar/scan/points",
                        ],
                        ["sensor/lidar/scan/points"],
                    ),
                ],
            ),
        ]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ros_gz_bridge)
    return ld
