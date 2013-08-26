#!/bin/sh
RUN_DIR=/home/thor/THOR/RunPenn
cd $RUN_DIR

# Kill any current slam instances
kill `screen -ls | grep slam | sed 's/\.slam.*//' | sed 's/\s\+//g'` 2>/dev/null && echo "Killed a previous instance!"
# Start it again after some sleep
sleep .1
screen -S slam -L -dm lua slam_wizard.lua
