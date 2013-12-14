#!/bin/sh
RUN_DIR=/home/thor/UPennDev/Player

# Enter the correct directory
cd $RUN_DIR

# Run the items that need power
screen -S lidar -L -dm lua lidar_wizard.lua
screen -S state -L -dm lua state_wizard.lua