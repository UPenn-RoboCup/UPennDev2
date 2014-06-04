dofile'../include.lua'
local signal = require'signal'
local libLog = require'libLog'
local Body = require(Config.dev.body)
local logger = libLog.new'joint'

function shutdown()
  logger:stop()
  os.exit()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

local sample_hz = 125

local get_time, usleep, max = unix.time, unix.usleep, math.max
local t0, t_sleep = get_time(), 1 / sample_hz
local count, t, t_last, t_debug = 0, t0, t0, t0
print('Begin logging joints...')
while true do
  count = count + 1
  t_last = t
  t = get_time()
  local entry = {
    n = count,
    t = t,
    cp = Body.get_command_position(),
    p = Body.get_position(),
    ft_l = Body.get_lfoot(),
    ft_r = Body.get_rfoot(),
    gyro = Body.get_gyro(),
    acc = Body.get_accelerometer(),
    rpy = Body.get_rpy(),
    mag = Body.get_magnetometer(),
  }
  logger:record(entry)
	--
	if t - t_debug>1 then
		t_debug = t
		print('Joint Logger', count)
	end
  --
	collectgarbage('step')
  t_diff = get_time() - t
  usleep(1e6 * max(t_sleep - t_diff, 0))
end
