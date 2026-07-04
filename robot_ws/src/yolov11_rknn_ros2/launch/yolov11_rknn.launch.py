from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("yolov11_rknn_ros2")
    default_model = PathJoinSubstitution([package_share, "rknnModel", "best.rknn"])
    default_params = PathJoinSubstitution([package_share, "config", "yolov11_rknn.yaml"])

    model_arg = DeclareLaunchArgument("model_path", default_value=default_model)
    video_device_arg = DeclareLaunchArgument("video_device", default_value="/dev/video21")
    params_arg = DeclareLaunchArgument("params_file", default_value=default_params)

    node = Node(
        package="yolov11_rknn_ros2",
        executable="detector_node",
        name="yolov11_rknn_detector",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "model_path": LaunchConfiguration("model_path"),
                "video_device": LaunchConfiguration("video_device"),
            },
        ],
    )

    return LaunchDescription([model_arg, video_device_arg, params_arg, node])
