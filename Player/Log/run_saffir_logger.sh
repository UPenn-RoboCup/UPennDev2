#!/bin/sh
killall -q lua luajit luajit2
killall -q screen

SCREEN_FLAGS="-dm -L -S"
LUA=/usr/bin/luajit

echo "Start IMU recording"
screen -dm -L -S log_imu $LUA log_ardu_imu.lua
echo "Start LIDAR record"
screen $SCREEN_FLAGS log_lidar $LUA log_lidar.lua
echo "Start FLIR record"
screen $SCREEN_FLAGS log_flir ./mnt/shadwell/log_flir
