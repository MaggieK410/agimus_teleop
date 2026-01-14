#!/bin/bash -e

cd /home/gepetto/ros2_ws

colcon build \
        --symlink-install \
        --parallel-workers $(nproc) \
        --cmake-args -Wall -Wextra -Wpedantic \
        --cmake-force-configure

