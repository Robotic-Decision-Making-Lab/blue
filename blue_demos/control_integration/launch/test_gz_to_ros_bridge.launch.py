from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

# add ros_gz_bridge to the Python path
# ros_gz_bridge_root = os.path.join(os.path.dirname(__file__), "..", "..")
# sys.path.insert(0, os.path.abspath(ros_gz_bridge_root))

# from ros_gz_bridge import mappings  # noqa: E402


def bridge_setup(context, *args, **kwargs):
    # gz_msgs_ver = LaunchConfiguration("gz_msgs_ver").perform(context)
    # gz_msgs_ver = tuple(map(int, gz_msgs_ver.split(".")))

    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/model/obs_1/pose@geometry_msgs/msg/Pose[gz.msgs.Pose"],
        output="screen",
    )
    return [bridge]


# f'/{m.unique()}@{m.ros2_string()}[{m.gz_string()}'
#           for m in mappings(gz_msgs_ver)


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=bridge_setup)])
