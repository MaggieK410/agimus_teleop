#!/bin/bash -e

cd /home/gepetto/ros2_ws

sudo apt update

rosdep update --rosdistro $ROS_DISTRO
rosdep install -y -i \
    --from-paths src \
        --rosdistro $ROS_DISTRO \
        --skip-keys libfranka \
        --skip-keys eigenpy \
        --skip-keys hpp-fcl \
        --skip-keys coal \
        --skip-keys pinocchio \
        --skip-keys example-robot-data \
        --skip-keys crocoddyl \
        --skip-keys mim_solvers \
        --skip-keys colmpc \
        --skip-keys hpp-util \
        --skip-keys hpp-template \
        --skip-keys hpp-statistics \
        --skip-keys hpp-environments \
        --skip-keys hpp-pinocchio \
        --skip-keys hpp-constraints \
        --skip-keys proxsuite \
        --skip-keys hpp-core \
        --skip-keys hpp-corbaserver \
        --skip-keys hpp-manipulation \
        --skip-keys hpp-manipulation-urdf \
        --skip-keys hpp-manipulation-corba \
        --skip-keys gepetto-viewer \
        --skip-keys gepetto-viewer-corba \
        --skip-keys hpp-gepetto-viewer \
        --skip-keys hpp-plot \
        --skip-keys hpp-gui \
        --skip-keys hpp-bin-picking \
        --skip-keys sea_state_broadcaster \
        --skip-keys allegro_hand_description \
        --skip-keys allegro_hand_controller_configuration \
        --skip-keys collision_aware_joint_trajectory_wrapper \
        --skip-keys sea_transmissions \
        --skip-keys realsense_simulation \
        --skip-keys tiago_pro_head_description \
        --skip-keys tiago_pro_bringup \
        --skip-keys omni_drive_controller \
        --skip-keys allegro_hand_system \
        --skip-keys pal_sea_arm_controller_configuration \
        --skip-keys pal_sea_arm_description \
        --skip-keys tiago_pro_description \
        --skip-keys tiago_pro_controller_configuration
