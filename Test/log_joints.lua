dofile'../include.lua'
local signal = require'signal'.signal
local running = true

local function shutdown()
	running = false
end
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)


local si = require'simple_ipc'
local libLog = require'libLog'
local Body = require('Body')
local logger = libLog.new'joint'
local sample_hz = 125

local get_time, usleep, max = unix.time, unix.usleep, math.max
local t0, t_sleep = get_time(), 1 / sample_hz
local count, t, t_last, t_debug = 0, t0, t0, t0
print('Begin logging joints...')
local entry
while running do
  count = count + 1
  t_last = t
  t = get_time()
  entry = {
    n = count,
    t = t,
    cp = Body.get_command_position(),
    p = Body.get_position(),
    ft_l = Body.get_lfoot(),
    ft_r = Body.get_rfoot(),
    gyro = Body.get_gyro(),
    acc = Body.get_accelerometer(),
  }
	-- Write the log
  logger:record(entry)
	-- Status message
	if t - t_debug > 1 then
		t_debug = t
		print('Joint Logger', count)
	end
  --
	collectgarbage('step')
  t_diff = get_time() - t
  usleep(1e6 * max(t_sleep - t_diff, 0))
end

logger:stop()
