---------------------------
-- World Manager --
-- (c) Stephen McGill 2014    --
---------------------------
dofile'include.lua'
local Body = require(Config.dev.body)
local lW = require'libWorld'
local si = require'simple_ipc'
-- Cache some functions
local get_time, usleep = Body.get_time, unix.usleep
-- Subscribe to important messages
local vision_ch = si.new_subscriber'vision'
-- SHM
require'wcm'
require'mcm'

-- Cleanly exit on Ctrl-C
local running, signal = true, nil

local signal = require'signal'
local function shutdown ()
  running = false
  --os.exit()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

local uOdometry
vision_ch.callback = function(skt)
  local detections = skt:recv_all()
  -- First, update the odometry
  uOdometry = mcm.get_status_odometry()
  lW.update_odometry(uOdometry)
  local pose_odom = lW.get_pose()
  -- Only use the last vision detection
  lW.update_vision(detections[#detections])
  local pose = lW.get_pose()
  wcm.set_robot_pose(pose)
end

-- Entry
lW.entry()
-- Timing
local TIMEOUT = 1 / 10
-- Timeout in milliseconds
local TIMEOUT_MS = TIMEOUT * 1e3
local poller.wait_on_channels{vision_ch}
local npoll
local t0, t = get_time()
local debug_interval, t_debug = 1, t0
while running do
  npoll = poller:poll(TIMEOUT_MS)
  if npoll==0 then
    -- If no frames, then just update by odometry
    uOdometry = mcm.get_status_odometry()
    lW.update_odometry(uOdometry)
    -- Update the pose here
    wcm.set_robot_pose(lW.get_pose())
  end
  t = get_time()
  if t - t_debug > debug_interval then
    t_debug = t
    print(string.format('World | Uptime: %.2f sec, Mem: %d kB', t-t0, collectgarbage('count')))
  end
end

lW.exit()
