#!/bin/sh
# Run the items that need power

# Kill previous instances
pkill -f state_wizard.lua
pkill -f lidar_wizard.lua

sleep .1

# Go to the correct directory
cd /home/thor/UPennDev/Player
screen -S lidar -L -dm lua lidar_wizard.lua
sleep .1
screen -S state -L -dm lua state_wizard.lua

# Double check that state wizard is *actually* running
#test ! `pgrep -f test_scripts.lua` && sleep 1 && screen -S state -L -dm lua state_wizard.lua
