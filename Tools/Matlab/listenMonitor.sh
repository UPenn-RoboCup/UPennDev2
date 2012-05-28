#!/bin/bash

cd ../../Player
export COMPUTER=`uname -s`
echo $1
if [ "$1" == "team" ]
	then 
		exec lua listen_team_monitor.lua
	else
		exec lua listen_monitor.lua
fi
