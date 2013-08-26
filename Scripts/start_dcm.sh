#!/bin/sh
RUN_DIR=/home/thor/THOR/RunPenn
cd $RUN_DIR

# Kill any current lidar instances
kill `screen -ls | grep dcm | sed 's/\.dcm.*//' | sed 's/\s\+//g'` 2>/dev/null && echo "Killed a previous instance!"
# Start it again after some sleep
sleep .2
screen -S dcm3 -L -dm lua joint_wizard3.lua
sleep .2
screen -S dcm1 -L -dm lua joint_wizard1.lua
sleep .2
screen -S dcm2 -L -dm lua joint_wizard2.lua

