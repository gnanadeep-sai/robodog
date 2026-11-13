#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
[ -f install/setup.bash ] || colcon build --symlink-install
source install/setup.bash
exec "$@"
