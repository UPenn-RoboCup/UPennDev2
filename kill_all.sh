# Kill all of our named screen sessions
screen -X -S camera quit
screen -X -S mesh quit
screen -X -S test quit
screen -X -S test quit

#!/bin/sh
killall screen lua luajit
RUN_DIR=/home/thor/UPennDev/Player
cd $RUN_DIR/scripts
sh start_dcm.sh
cd $RUN_DIR
screen -S camera -L -dm lua camera_wizard.lua
screen -S mesh -L -dm lua slam_wizard.lua

# Estopped items
screen -S lidar -L -dm lua lidar_wizard.lua
screen -S control -L -dm lua control_wizard.lua