#!/usr/bin/env bash
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
#
# Installs ROS 2 Jazzy, Gazebo Harmonic and the tools this branch needs on Ubuntu 24.04.
# Run it from the repository folder: ./install_deps.sh
set -euo pipefail

if [[ "$(. /etc/os-release && echo "$VERSION_ID")" != "24.04" ]]; then
  echo "This script supports Ubuntu 24.04 only (ROS 2 Jazzy needs it)." >&2
  exit 1
fi

SUDO=""
[[ "$(id -u)" -ne 0 ]] && SUDO="sudo"

echo "==> System packages"
$SUDO apt-get update
$SUDO apt-get install -y curl lsb-release python3-tk python3-numpy python3-matplotlib

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "==> ROS 2 Jazzy"
  $SUDO apt-get install -y software-properties-common
  $SUDO add-apt-repository -y universe
  VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | cut -d'"' -f4)
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VERSION}/ros2-apt-source_${VERSION}.noble_all.deb"
  $SUDO apt-get install -y /tmp/ros2-apt-source.deb
  $SUDO apt-get update
  $SUDO apt-get install -y ros-jazzy-desktop
fi

echo "==> ROS packages used by this branch (includes Gazebo Harmonic)"
$SUDO apt-get install -y python3-colcon-common-extensions ros-jazzy-ros-gz

echo
echo "Done. Next step: the 'Build' section of README.md."
