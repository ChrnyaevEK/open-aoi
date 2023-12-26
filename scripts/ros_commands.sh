# Source ROS to terminal
source /opt/ros/noetic/setup.bash

# Source catkin worspace
cd source/ros_catkin_ws
source devel/setup.bash
echo $ROS_PACKAGE_PATH