#!/bin/sh
# Run the items that do not need power
cd /home/thor/UPennDev/Player
screen -S camera -L -dm lua camera_wizard.lua
screen -S mesh   -L -dm lua mesh_wizard.lua
screen -S rpc    -L -dm lua rpc_wizard.lua
screen -S audio  -L -dm lua audio_wizard.lua