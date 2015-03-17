#!/bin/sh
sudo ifconfig wlan0 up
sudo iwconfig wlan0 essid AirPennNet-Guest ap any
sudo dhclient wlan0
