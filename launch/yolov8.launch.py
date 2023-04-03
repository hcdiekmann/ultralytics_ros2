
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


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

    # Node
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

    ld = LaunchDescription()

    ld.add_action(model_cmd)
    ld.add_action(tracker_cmd)
    ld.add_action(device_cmd)
    ld.add_action(enable_cmd)
    ld.add_action(threshold_cmd)
    ld.add_action(input_image_topic_cmd)
    ld.add_action(namespace_cmd)

    ld.add_action(detector_node_cmd)

    return ld