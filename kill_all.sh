#!/bin/sh
# Kill anything that we may have started
sh kill_active.sh
sh kill_passive.sh
killall screen lua luajit
rm /dev/shm/*cm*
