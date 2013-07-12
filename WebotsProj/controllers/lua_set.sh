#!/bin/sh

##############################
##### USER ADJUSTABLE SETTINGS
##############################
EXT_TERM=0
LUA=lua
#LUA=luajit

# Go into the Player directory
cd Player
#echo $PWD
#echo $PATH
#echo $OSTYPE
#echo `which xterm`

# Set up the right settings for a mac
COMPUTER=`uname`
export COMPUTER
if [ "$COMPUTER" = "Darwin" ]
then
  #export OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
	eval `/usr/libexec/path_helper -s`
	source ~/.bash_profile
fi
TERM=`which xterm`

# On Linux, need to verify that xterm is not setgid
# Otherwise, LD_LIBRARY_PATH gets unset in xterm
export PLAYER_ID=$1
export TEAM_ID=$2
export PLATFORM=webots

# Spawn the right terminal
if [ "$EXT_TERM" -gt "0" ]
then
  # In separate xterms
  exec $TERM -l -e "$LUA Test/test_ladder.lua"
else
  # In webots console
  exec $LUA Test/test_ladder.lua
fi

#exec luajit -l controller start.lua
#exec xterm -l -e "/usr/bin/gdb --args lua start.lua"
#exec xterm -l -e "valgrind --tool=memcheck --leak-check=yes --dsymutil=yes luajit start.lua"
