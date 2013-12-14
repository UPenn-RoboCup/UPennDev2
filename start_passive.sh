#!/bin/sh
RUN_DIR=/home/thor/UPennDev/Player

# Enter the correct directory
cd $RUN_DIR

# Run the items that do not need power
screen -S camera -L -dm lua camera_wizard.lua
screen -S mesh   -L -dm lua mesh_wizard.lua
screen -S rpc    -L -dm lua rpc_wizard.lua