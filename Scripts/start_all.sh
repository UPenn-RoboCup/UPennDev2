#!/bin/sh
killall screen lua luajit
RUN_DIR=/home/thor/THOR/RunPenn
cd $RUN_DIR/scripts
sh start_dcm.sh
cd $RUN_DIR
screen -S camera -L -dm lua camera_wizard.lua
sleep .1
screen -S lidar -L -dm lua lidar_wizard.lua
sleep .1
screen -S slam -L -dm lua slam_wizard.lua
sleep .1
screen -S control -L -dm lua control_wizard.lua

