#!/bin/sh

EXT_TERM=0
LUA=lua
#LUA=luajit

# On Linux, need to verify that xterm is not setgid
# Otherwise, LD_LIBRARY_PATH gets unset in xterm

COMPUTER=`uname`
export COMPUTER

export PLAYER_ID=$1
export TEAM_ID=$2

PLATFORM=webots
export PLATFORM

echo `which xterm`
echo $PATH

TERM=xterm
if [ "$COMPUTER" = "Darwin" ]
then
  TERM=/opt/X11/bin/xterm
fi

if [ "$EXT_TERM" -gt "0" ]
then
  # In separate xterms
  exec $TERM -l -e "$LUA start.lua"
  #exec luajit -l controller start.lua
else
  # In webots console
  exec $LUA start.lua
fi


#exec xterm -l -e "/usr/bin/gdb --args lua start.lua"
#exec xterm -l -e "valgrind --tool=memcheck --leak-check=yes --dsymutil=yes luajit start.lua"
#exec xterm -l -e "/Users/yida/Downloads/lua-5.2.1/src/lua start.lua"
