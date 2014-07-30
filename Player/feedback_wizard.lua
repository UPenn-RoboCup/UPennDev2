-- (c) 2013, 2014 Stephen McGill, Seung-Joon Yi
dofile'../include.lua'
local si = require'simple_ipc'

-- 2 Hz feedback
local t_sleep = 1e6 / 2

local running = true
local feedback_udp_ch =
si.new_sender(Config.net.operator.wired, Config.net.feedback)

local data, ret, err
local function send_status_feedback()
	data = {
		pose = wcm.get_robot_pose(),
		battery = Body.get_sensor_battery(),
		rpy = Body.get_sensor_rpy(),
		t   = unix.time(),
	}
	ret, err = feedback_udp_ch:send( mp.pack(data) )
	if err then print('Feedback UDP error',err) end
end

while running do
	send_status_feedback()
	unix.usleep(t_sleep)
end
