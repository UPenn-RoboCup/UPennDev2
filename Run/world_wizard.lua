#!/usr/bin/env luajit
---------------------------
-- World Manager --
-- (c) Stephen McGill 2014    --
---------------------------
dofile'../include.lua'
local Body = require(Config.dev.body)
local lW = require'libWorld'
local si = require'simple_ipc'
local mp = require'msgpack.MessagePack'
local util = require'util'
-- Cache some functions
local get_time = Body.get_time
-- Subscribe to important messages
local vision_ch = si.new_subscriber'vision'
-- UDP channel
local operator
if Config.net.use_wireless then
	operator = Config.net.operator.wireless
else
	operator = Config.net.operator.wired_broadcast
end
local ENABLE_SEND = false
local udp_ch = si.new_sender(operator, Config.net.streams.camera0.udp)
-- SHM
require'wcm'
require'mcm'
require'hcm'

-- Cleanly exit on Ctrl-C
local running, signal = true, nil

local signal = require'signal'
local function shutdown()
  running = false
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

local uOdometry0, uOdometry
local t_send, send_interval = 0

vision_ch.callback = function(skt)
  local detections = skt:recv_all()
  -- Only use the last vision detection
	local detection = mp.unpack(detections[#detections])

-- Update localization based onodometry and vision
--Should use the differential of odometry!
  if not uOdometry0 then uOdometry0 = mcm.get_status_odometry()
  else uOdometry0 = uOdometry end
  uOdometry = mcm.get_status_odometry()
  dOdometry = util.pose_relative(uOdometry,uOdometry0)
  
  lW.update(dOdometry, detection)
  
end

-- Timeout in milliseconds
local TIMEOUT = 1 / 10 * 1e3
local poller = si.wait_on_channels{vision_ch}
local npoll
local t0, t = get_time()
local debug_interval, t_debug = 1, t0

local function update()
	send_interval = 1 / hcm.get_monitor_fps()
  npoll = poller:poll(TIMEOUT)
  if npoll==0 then
    -- If no frames, then just update by odometry
    --Should use the differential of odometry!
    if not uOdometry0 then uOdometry0 = mcm.get_status_odometry()
    else uOdometry0 = uOdometry end
    uOdometry = mcm.get_status_odometry()
    dOdometry = util.pose_relative(uOdometry,uOdometry0)
    lW.update_odometry(dOdometry)
    
    -- Update the pose here
    wcm.set_robot_pose(lW.get_pose())
  end
  t = get_time()
	if ENABLE_SEND and t-t_send > send_interval then
		-- Send localization info to monitor
		local metadata = {}
		metadata.id = 'world'
		metadata.world = lW.send()
		-- Send!
		local ret, err = udp_ch:send(mp.pack(metadata))
		--if err then print(ret, err) end
		t_send = t
	end
end

if ... and type(...)=='string' then
	TIMEOUT = 0
	return {entry=lW.entry, update=update, exit=lW.exit}
end

lW.entry()
while running do
	update()
	if t - t_debug > debug_interval then
    t_debug = t
    print(string.format('World | Uptime: %.2f sec, Mem: %d kB', t-t0, collectgarbage('count')))
  end
end
lW.exit()