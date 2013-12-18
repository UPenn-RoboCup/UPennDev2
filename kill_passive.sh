#!/bin/sh
# Kill the items that do not need power
pkill -f headcam_wizard.lua
pkill -f handcam_wizard.lua
pkill -f mesh_wizard.lua
pkill -f audio_wizard.lua
pkill -f rpc_wizard.lua
#pkill -f multicamera_wizard.lua
#pkill -f slam_wizard.lua
#pkill -f unlogger_wizard.lua
#pkill -f log_wizard.lua