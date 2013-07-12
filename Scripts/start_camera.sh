#!/bin/sh
RUN_DIR=/home/thor/THOR/RunPenn
cd $RUN_DIR

# Kill any current camera instances
kill `screen -ls | grep camera | sed 's/\.camera.*//' | sed 's/\s\+//g'` 2>/dev/null && echo "Killed a previous instance!"
# Start it again after some sleep
sleep .1
screen -S camera -L -dm lua camera_wizard.lua
