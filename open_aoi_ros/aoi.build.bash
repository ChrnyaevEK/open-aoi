source /opt/ros/foxy/setup.bash

# rosdep update --include-eol-distros
# rosdep install --from-paths src -y --ignore-src

colcon build --packages-select open_aoi_interfaces
colcon build --packages-select open_aoi_ros

source install/setup.bash