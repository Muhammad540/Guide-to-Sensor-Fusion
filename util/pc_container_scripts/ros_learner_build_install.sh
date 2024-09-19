#!/usr/bin/env bash
source ~/.bashrc
source /opt/ros/humble/setup.bash

cd /home/ros_learner/mnt/ws
# remove the build log and install files before doing colcon build.
rm -rf build log install
# observation: without uninstalling the previous remnants of colcon build, when rebuilding container the colcon build fails giving symlink errors
colcon build --symlink-install
source install/setup.bash
