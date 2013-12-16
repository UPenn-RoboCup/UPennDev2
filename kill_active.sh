#!/bin/sh

# Kill the items that do not need power
#screen -X -S state quit
kill `screen -ls | grep state | sed 's/\.state.*//' | sed 's/\s\+//g'` 2>/dev/null && echo "Killed a previous instance!"

#
screen -X -S lidar quit
