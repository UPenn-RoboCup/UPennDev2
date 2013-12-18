#!/bin/sh
# Kill everything first
sh kill_all.sh
# Begin running everything
sh start_passive.sh
sh start_active.sh