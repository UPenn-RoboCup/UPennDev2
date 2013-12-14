#!/bin/sh

# Kill all processes
killall screen lua luajit

# Begin running everything
sh start_passive.sh
sh active_passive.sh