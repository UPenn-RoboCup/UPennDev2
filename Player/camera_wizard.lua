-----------------------------------------------------------------
-- Combined Lidar manager for Team THOR
-- Reads and sends raw lidar data
-- As well as accumulate them as a map
-- and send to UDP/TCP
-- (c) Stephen McGill, Seung Joon Yi, 2013
---------------------------------
-- TODO: Critical section for include
-- Something there is non-reentrant
dofile'../include.lua'
-- Going to be threading this
local simple_ipc = require'simple_ipc'
-- Are we a child?
local IS_CHILD, pair_ch = false, nil
local CTX, metadata = ...
if CTX and type(metadata)=='table' then
	IS_CHILD=true
	simple_ipc.import_context( CTX )
	pair_ch = simple_ipc.new_pair(metadata.ch_name)
	print('CHILD CTX')
end
-- For all threads
local mp     = require'msgpack.MessagePack'

local cameras = {
	'camera0' = {
		dev = '/dev/video0',
	}
}

meta.t     = get_time()
			meta.n     = #camera_arr
			meta.w     = w
			meta.h     = h

-- The main thread's job is to give:
-- joint angles and network request
if not IS_CHILD then
	-- TODO: Signal of ctrl-c should kill all threads...
	local signal = require'signal'
	local util = require'util'

	-- Only the parent accesses shared memory
	require'vcm'
	local poller, channels = nil, {}
	-- Spawn children
	for _,cam in ipairs(Config.camera) do
		print(util.color('Setting up','green'),name,cam.device)
		-- Open the camera
		local camera = setup_camera(cam,name)
		-- The thread will automagically start detached upon gc
		local ch, thread = simple_ipc.new_thread('mesh_wizard.lua',cam.name,cam)
		-- Deal with any data from the camera
		ch.callback = function(s)
			local data, has_more = poller.lut[s]:receive()
			--print('child tread data!',data)
		end
		-- Add the channel for our poller to sample
		channels[cam.name] = ch
		-- Start detached
		thread:start(true,true)
	end
	-- TODO: Also poll on mesh requests, instead of using net_settings...
	-- This would be a Replier (then people ask for data, or set data)
	-- Instead of changing shared memory a lot...
	local replier = simple_ipc.new_replier'mesh'
	replier.callback = function(s)
		local data, has_more = poller.lut[s]:receive()
		-- Send message to a thread
		local cmd = mp.unpack(data)
		if cmd.id==0 then
	end
	table.insert(channels,replier)
	-- Ensure that we shutdown the threads properly
	signal.signal("SIGINT", os.exit)
	signal.signal("SIGTERM", os.exit)
	-- Just wait for requests from the children
	poller = simple_ipc.wait_on_channels(channels)
	poller:start()
	return
end

-- Libraries
local util   = require'util'
local udp    = require'udp'
local uvc    = require'uvc'
local jpeg   = require'jpeg'
jpeg.set_quality( 95 )

local function setup_camera(cam)
	-- Get the config values
	local dev = cam.device
	local fps = cam.fps
	local fmt = cam.format
	local width, height = unpack(cam.resolution)
	-- Save the metadata
	local meta = {
		t = 0,
		c = 'jpeg',
		w = width,
		h = height
		count = 0,
		name = cam.name..'_camera',
	}
	-- Open the camera
	local dev = uvc.init(dev, width, height, fmt, 1, fps)
	-- Open the unreliable network channel
	local camera_udp_ch = udp.new_sender(operator, Config.net.camera[name])
	-- Open the reliable network channel
	--local camera_tcp_ch = simple_ipc.new_publisher(Config.net.reliable_camera[name],false,'*')
	-- Open the local channel for logging
	--local cam_pub_ch = simple_ipc.new_publisher(meta.name)
	-- Export
	local camera = {
		meta = meta,
		dev  = dev,
		fmt = fmt,
		udp  = camera_udp_ch,
		--pub  = cam_pub_ch,
		--tcp  = camera_tcp_ch
	}
	return camera
end
-- Setup the camera from our metadata
local camera = setup_camera(metadata)
while true do
	local img, sz, cnt, t = camera:get_image()
	print('img',img)
end
