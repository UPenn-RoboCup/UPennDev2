#!/usr/local/bin/luajit
local unix = require'unix'
local x = io.popen('./tutorials/emg_stty /dev/ttyACM0 115200')
--print(x)
--unix.usleep(2e6)
print('what')
local v = x:read()
print(v)

--while true do
	print('count')
	
--	unix.usleep(1)
--end