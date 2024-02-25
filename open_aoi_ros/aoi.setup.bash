# This script install custom python dependencies to 
# ament folder structure. This script is ment to be used
# for package development. Make sure to run `colcon build` first
# to initiate required folder structure.Python packages will be available using global interpreter after running
# following commands (packages are not installed globally, only added to python path,
# see `echo $PYTHONPATH`):
# >>> source /opt/ros/foxy/setup.bash && source install/setup.bash

WORKSPACE_SITE_PACKAGES='install/open_aoi_ros/lib/python3.8/site-packages'

# Exit on error
set -e

# Check python
python3 -V

# Install python dependencies
if test -d $WORKSPACE_SITE_PACKAGES; then
    echo 'Install python dependencies'
    pip3 install -r requirements.txt --target=$WORKSPACE_SITE_PACKAGES
else
    echo "Unable to install dependencies - ament workspace is not initialized. Not found: $WORKSPACE_SITE_PACKAGES"
fi