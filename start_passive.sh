#!/bin/sh
RUN_DIR=/home/thor/UPennDev/Player

# Enter the correct directory
cd $RUN_DIR

# Run the items that do not need power
screen -X -S camera quit
screen -S camera -L -dm lua camera_wizard.lua
#
screen -X -S mesh quit
screen -S mesh -L -dm lua mesh_wizard.lua
#
screen -X -S rpc quit
screen -S rpc -L -dm lua rpc_wizard.lua