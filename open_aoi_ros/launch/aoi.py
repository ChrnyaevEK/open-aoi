import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
)


def generate_launch_description():

    aoi = Node(package="open_aoi_ros", executable="image_acquisition_service")
    bridge = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch/rosbridge_websocket_launch.xml",
            ),
        )
    )

    return LaunchDescription([aoi, bridge])
