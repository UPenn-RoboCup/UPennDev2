#!/bin/sh

KERNEL=`uname -r`

# Kill all processes to reset player
killall naoqi-bin
killall naoqi
killall espeak
killall lua
killall luajit
killall luajit2
killall screen
#

run_nao(){
        echo "Starting Naoqi..."
        screen -dm -L -S naoqi /usr/bin/naoqi-bin -v
        ## Allow DCM some time to start
        sleep 3
        #
        echo "Starting Cognition..."
        cd $PLAYER_DIR
        screen -dm -L -S cognition /usr/local/bin/lua run_cognition.lua
        ## Allow Cognition some time to start
        sleep 3
        #
        echo "Starting Test Vision..."
        cd $PLAYER_DIR
        screen -m -L -S test /usr/local/bin/lua test_vision.lua
        #
        echo "Rock and Roll!"
}

run_op(){
        echo "Starting DCM..."
        cd $PLAYER_DIR/Lib
        screen -dm -L -S dcm /usr/bin/luajit run_dcm.lua
        ## Allow DCM some time to start
        sleep 1
        #
        echo "Starting Cognition..."
        cd $PLAYER_DIR
        screen -dm -L -S cognition /usr/bin/luajit run_cognition.lua
        ## Allow Cognition some time to start
        sleep 1
        #
        echo "Starting Test Vision..."
        cd $PLAYER_DIR
        screen -m -L -S test /usr/bin/luajit test_vision.lua
        #
        echo "Rock and Roll!"
}

case "$KERNEL" in
        *aldebaran*) 
                echo "Starting Player on Nao platform" 
                PLAYER_DIR=/home/nao/Player
                run_nao ;; 
        *) echo "op"
                echo "Starting Player on DARwIn platform..."
                PLAYER_DIR=/home/darwin/current/Player
                run_op ;;         
esac


# Make sure /home/darwin/current links to the right UPennDev
# Put these lines in /etc/rc.local
# #!/bin/bash
# su -c "sh /home/darwin/current/Player/startup.sh" darwin
