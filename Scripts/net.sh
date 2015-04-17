#!/bin/sh
sudo ifconfig wlan0 up
sudo iwconfig wlan0 essid robocup ap any
#sudo dhclient wlan0
sudo ip route add 192.168.1.0/24 via 192.168.123.1
