#!/bin/sh
# Kill anything that we may have started
killall screen lua luajit
rm /dev/shm/*cm*
