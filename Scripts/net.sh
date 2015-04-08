#!/bin/sh
sudo ifconfig wlan0 up
sudo iwconfig wlan0 essid robocup ap any
#sudo dhclient wlan0
