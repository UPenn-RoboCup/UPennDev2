--/usr/local/bin/luajit
--package.path = package.path .. ';shm/shm.lua'
local unix = require'unix'
local shm = require'shm'
t = shm.new('estop')
--print('New...',t)
--t:set('a')

while true do

	local x = io.popen('./tutorials/emg_stty /dev/ttyACM0 115200')
	x:setvbuf'no'
	local v = x:read()
	t:set('a',v)
	--t:set('t.a',v)

	print(t:get('a'))
	--print(t:get('t.a'))

	--print(v)
	unix.usleep(6e4)
end
--[[
print(x)
x:setvbuf'no'
unix.usleep(2e6)
local v = x:read()
print(v)

--while true do
	print('count')
	
--	unix.usleep(1)
--end
]]--