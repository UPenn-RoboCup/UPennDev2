#!/bin/sh
RUN_DIR=/home/thor/THOR/RunPenn
cd $RUN_DIR

# Kill any current lidar instances
kill `screen -ls | grep lidar | sed 's/\.lidar.*//' | sed 's/\s\+//g'` 2>/dev/null && echo "Killed a previous instance!"
# Start it again after some sleep
sleep .1
screen -S lidar -L -dm lua lidar_wizard.lua
