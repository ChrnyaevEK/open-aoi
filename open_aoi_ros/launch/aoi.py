import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.events.process.process_exited import ProcessExited
from launch.event_handlers.on_process_exit import OnProcessExit


def on_exit_restart(event: ProcessExited, context):
    print(
        "[Custom handler] Process [{}] exited, pid: {}, return code: {}\n\n".format(
            event.action.name, event.pid, event.returncode
        )
    )
    if event.returncode != 0 and "aoi" in event.action.name:
        print(f"Respawning AOI service: {event.action.name}")
        return aoi_launch_description()  # Respawn node


def aoi_launch_description():
    aoi = Node(
        package="open_aoi_ros",
        executable="aoi_image_acquisition_service",
    )
    return LaunchDescription([aoi])


def generate_launch_description():
    bridge = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch/rosbridge_websocket_launch.xml",
            ),
        )
    )

    return LaunchDescription(
        [
            aoi_launch_description(),
            bridge,
            RegisterEventHandler(event_handler=OnProcessExit(on_exit=on_exit_restart)),
        ]
    )
