import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():

    # Launch arguments
    model = LaunchConfiguration("model")
    model_cmd = DeclareLaunchArgument(
        "model",
        default_value="yolov8s.pt",
        description="Model name or path")

    tracker = LaunchConfiguration("tracker")
    tracker_cmd = DeclareLaunchArgument(
        "tracker",
        default_value="bytetrack.yaml",
        description="Tracker name or path")

    device = LaunchConfiguration("device")
    device_cmd = DeclareLaunchArgument(
        "device",
        default_value="cpu",
        description="Device to use GPU or CPU")

    enable = LaunchConfiguration("enable")
    enable_cmd = DeclareLaunchArgument(
        "enable",
        default_value="True",
        description="Wheter to start inference")

    threshold = LaunchConfiguration("threshold")
    threshold_cmd = DeclareLaunchArgument(
        "threshold",
        default_value="0.5",
        description="Minimum probability of a detection to be published")

    input_image_topic = LaunchConfiguration("input_image_topic")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/camera/image_raw",
        description="Topic name of the input image message")

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="yolo",
        description="Namespace for the nodes")
    
    use_rviz = LaunchConfiguration("use_rviz")
    use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Wheter to start RViz")
    
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(get_package_share_directory('ultralytics_ros2'), 'rviz', 'view_pointcloud_and_yolo.rviz'),
        description="Path to RViz configuration file"
    )


    # Nodes
    detector_node_cmd = Node(
        package="ultralytics_ros2",
        executable="yolov8_node",
        name="yolov8_node",
        namespace=namespace,
        parameters=[{"model": model,
                     "tracker": tracker,
                     "device": device,
                     "enable": enable,
                     "threshold": threshold}],
        remappings=[("image_raw", input_image_topic)]
    )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
                    '-d', rviz_config_file,
                ],
    )

    # Add everything to launch description and return
    ld = LaunchDescription()

    ld.add_action(model_cmd)
    ld.add_action(tracker_cmd)
    ld.add_action(device_cmd)
    ld.add_action(enable_cmd)
    ld.add_action(threshold_cmd)
    ld.add_action(input_image_topic_cmd)
    ld.add_action(namespace_cmd)
    ld.add_action(use_rviz_cmd)
    ld.add_action(rviz_config_file_cmd)

    ld.add_action(detector_node_cmd)
    ld.add_action(start_rviz_cmd)

    return ld