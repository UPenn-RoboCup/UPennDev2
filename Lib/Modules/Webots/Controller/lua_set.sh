#!/bin/sh

# On Linux, need to verify that xterm is not setgid
# Otherwise, LD_LIBRARY_PATH gets unset in xterm

COMPUTER=`uname`
export COMPUTER

export PLAYER_ID=$1
export TEAM_ID=$2

PLATFORM=webots
export PLATFORM

#exec xterm -l -e "lua start.lua"
#exec xterm -l -e "luajit start.lua"
#exec luajit -l controller start.lua
exec lua start.lua
#exec xterm -l -e "/usr/bin/gdb --args lua start.lua"
#exec xterm -l -e "valgrind --tool=memcheck --leak-check=yes --dsymutil=yes luajit start.lua"
#exec xterm -l -e "/Users/yida/Downloads/lua-5.2.1/src/lua start.lua"
