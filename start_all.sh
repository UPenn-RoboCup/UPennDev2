#!/bin/sh
sleep .2
# Kill everything first
sh kill_all.sh
sleep .1
# Begin running everything
sh start_passive.sh
sh start_active.sh
