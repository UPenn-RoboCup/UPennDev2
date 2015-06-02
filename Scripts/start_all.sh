#!/bin/sh
killall screen lua luajit
RUN_DIR=/home/thor/UPennDev2/Run
cd $RUN_DIR
sleep .1
screen -S fb -L -dm luajit feedback_wizard.lua
sleep .1
screen -S rpc -L -dm luajit rpc_wizard.lua
sleep .1
screen -S lidar -L -dm luajit lidar_wizard.lua
sleep .1
screen -S camera1 -L -dm luajit camera_wizard.lua 1
sleep .1
screen -S camera2 -L -dm luajit camera_wizard.lua 2
sleep .1
screen -S mesh -L -dm luajit mesh_wizard.lua
sleep .1
screen -S slam -L -dm luajit slam_wizard2.lua
