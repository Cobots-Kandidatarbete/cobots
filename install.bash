#!/usr/bin/env bash
PATH_TO_SRC="./dep_ws/src/" 
PATH_TO_WS_SRC="./ws/src/"
git clone https://github.com/sequenceplanner/gui_tools.git $PATH_TO_SRC/gui_tools
git clone https://github.com/sequenceplanner/mp_msgs.git $PATH_TO_SRC/mp_msgs
git clone https://github.com/sequenceplanner/opcua_ros2_bridge.git $PATH_TO_SRC/opcua_ros2_bridge
git clone https://github.com/sequenceplanner/tf_tools.git $PATH_TO_SRC/tf_tools
git clone https://github.com/sequenceplanner/ur_script_driver.git $PATH_TO_SRC/ur_script_driver
git clone https://github.com/sequenceplanner/ur_script_msgs.git $PATH_TO_SRC/ur_script_msgs
git clone https://github.com/sequenceplanner/ur_tools.git $PATH_TO_SRC/ur_tools
git clone https://github.com/sequenceplanner/viz_tools.git $PATH_TO_SRC/viz_tools
git clone https://github.com/sequenceplanner/tf_tools_msgs.git $PATH_TO_SRC/tf_tools_msgs
git clone https://github.com/sequenceplanner/ur_tools_msgs.git $PATH_TO_SRC/ur_tools_msgs

git clone https://github.com/Cobots-Kandidatarbete/aruco_handler.git $PATH_TO_WS_SRC/aruco_handler
git clone https://github.com/Cobots-Kandidatarbete/camera-calibration.git $PATH_TO_WS_SRC/camera-calibration

cd dep_ws/src/ur_tools
git reset --hard 850bc0440a740f85bccb25f00d5fbe642f819a6f
cd ..
cd ..
colcon build
cd ..
