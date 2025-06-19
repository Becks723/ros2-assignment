from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_share_dir = get_package_share_directory("node")

    return LaunchDescription([
        Node(
            package="node",
            executable="serialnode",
            parameters=[
                pkg_share_dir + "/configs/config.yaml",
            ],
        ),
        Node(
            package="node",
            executable="pubnode",
            name="pubnode",
            parameters=[
                pkg_share_dir + "/configs/config.yaml",
            ],
        ),
    ])