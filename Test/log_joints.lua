dofile'../include.lua'
local signal = require'signal'
local libLog = require'libLog'
local logger = libLog.new'joint'

function shutdown()
  logger:stop()
  os.exit()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

local sample_hz = 125

local get_time = unix.time
local t0, t_sleep = get_time(), 1 / sample_hz
local count, t, t_last = 0, t0, t0
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
  t_diff = get_time() - t
  usleep(1e6 * max(t_sleep - t_diff, 0))
end
