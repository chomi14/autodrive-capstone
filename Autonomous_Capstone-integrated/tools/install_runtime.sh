#!/usr/bin/env bash
set -e
sudo apt update
sudo apt install -y \
  python3-opencv python3-serial python3-scipy v4l-utils \
  ros-humble-cv-bridge ros-humble-rqt-image-view ros-humble-rviz2 \
  ros-humble-tf2-ros ros-humble-std-srvs

# Keep numpy 1.x for maximum compatibility with Ubuntu 22.04 / ROS2 Humble cv_bridge.
python3 -m pip install --user 'numpy<2' ultralytics pyyaml

echo
printf 'Add yourself to dialout if not already present:\n  sudo usermod -aG dialout $USER\nThen log out and back in.\n'
