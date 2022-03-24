#!/usr/bin/env bash
PATH_TO_SRC="./dep_ws/src/" 
PATH_TO_WS_SRC="./ws/src/"

git clone https://github.com/Cobots-Kandidatarbete/dep-ws.git ./dep_ws
git clone https://github.com/Cobots-Kandidatarbete/aruco_handler.git $PATH_TO_WS_SRC/aruco_handler
git clone https://github.com/Cobots-Kandidatarbete/camera-calibration.git $PATH_TO_WS_SRC/camera-calibrator

