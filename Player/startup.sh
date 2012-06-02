#!/bin/sh

echo "Starting Player on DARwIn platform..."
PLAYER_DIR=/home/darwin/current/Player

# Kill all processes to reset player
killall lua
killall luajit
killall luajit2
killall screen

echo "Starting DCM..."
cd $PLAYER_DIR/Lib
screen -dm -L -S dcm /usr/bin/luajit run_dcm.lua
# Allow DCM some time to start
sleep 1

echo "Starting Cognition..."
cd $PLAYER_DIR
screen -dm -L -S cognition /usr/bin/luajit run_cognition.lua
# Allow Cognition some time to start
sleep 1

echo "Starting Player..."
cd $PLAYER_DIR
screen -dm -L -S player /usr/bin/luajit run_main_op.lua

echo "Rock and Roll!"

# 139.140.218.255Make sure /home/darwin/current links to the right UPennDev
# Put these lines in /etc/rc.local
# #!/bin/bash
# su -c "sh /home/darwin/current/Player/startup.sh" darwin
