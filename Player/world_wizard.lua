---------------------------
-- World Manager --
-- (c) Stephen McGill 2014    --
---------------------------
dofile'include.lua'
local Body = require(Config.dev.body)
local lW = require'libWorld'
local si = require'simple_ipc'
local mp = require'msgpack.MessagePack'
-- Cache some functions
local get_time, usleep = Body.get_time, unix.usleep
-- Subscribe to important messages
local vision_ch = si.new_subscriber'vision'
-- UDP channel
local udp = require'udp'
local operator = Config.net.operator.wired
local udp_ch = udp.new_sender(operator, Config.camera[1].udp_port)
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
util = require'util'
local uOdometry
vision_ch.callback = function(skt)
  local detections = skt:recv_all()
  -- Only use the last vision detection
	local detection = mp.unpack(detections[#detections])

  -- Update localization based onodometry and vision
  uOdometry = mcm.get_status_odometry()
  lW.update(uOdometry, detection)
  -- Update pose
	local pose = lW.get_pose()
  wcm.set_robot_pose(pose)
  
  -- Send localization info to monitor
  local metadata = {}
  metadata.id = 'world'
  metadata.world = lW.send()
  if detection.posts then
    local goal = {}
    goal.type = detection.posts[1].type
    goal.v1 = detection.posts[1].v
    if goal.type==3 then
      goal.v2 = detection.posts[2].v
    end
    metadata.world.goal = goal
  end
  -- Send!
  udp_ch:send(mp.pack(metadata))
end

-- Entry
lW.entry()
-- Timing
local TIMEOUT = 1 / 10
-- Timeout in milliseconds
local TIMEOUT_MS = TIMEOUT * 1e3
local poller = si.wait_on_channels{vision_ch}
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
