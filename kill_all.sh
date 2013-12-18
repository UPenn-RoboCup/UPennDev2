#!/bin/sh
# Kill anything that we may have started
sh kill_active.sh
sh kill_passive.sh
sleep .1
killall screen lua luajit
rm -f /dev/shm/*cm*
