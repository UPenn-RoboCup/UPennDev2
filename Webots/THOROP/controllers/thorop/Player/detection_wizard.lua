---------------------------------
-- Detection wizard for Team THOR
-- Camera Vision stuff
-- Listen to camera images via
-- raw device or zmq channel
-- (c) Stephen McGill, 2013
---------------------------------
dofile'../include.lua'
local simple_ipc = require'simple_ipc'
local wait_channels = {}
local mp = require'msgpack'
local vector = require'vector'
local util = require'util'
local cutil = require'cutil'
local Body = require'Body'
local torch = require'torch'
torch.Tensor = torch.DoubleTensor
local t0 = Body.get_time()

-- Detection process
local detect = function( camera )
	print( 'Detecting', type(camera.raw), camera.ts, camera.t )
end

-- Setup a new camera device
local function setup_dev( dev, meta )
  -- Open the camera
	local w, h = unpack(meta.res)
  dev.uvc = uvc.init(meta.fname, w, h, meta.fmt, 1, meta.fps)
	-- Allow for polling by ZeroMQ
	dev.socket_handle = dev.uvc:descriptor()
  -- Open the local channel for logging
  --dev.ch = simple_ipc.new_publisher(meta.name)
	-- Common metadata
	dev.cnt = 0
end

-- Setup a new camera channel
local function setup_ch( ch, meta )
	ch.n = meta.n
	ch.w = meta.w
	ch.h = meta.h
	ch.fmt = meta.fmt
	--
	ch.raw = torch.ByteTensor(ch.w*ch.h*3)
	ch.cnt = 0
end

local dev_cb = function(sh)
	local dev = wait_channels.lut[sh]
	local img, img_sz = dev.uvc:get_image()
	dev.t = Body.get_time()
	dev.ts = dev.t - t0
	-- TODO: Ensure img_sz is correct? If JPEG though...
	dev.raw = img
	detect( dev )
end

-- Listen for camera
local ch_cb = function(sh)
	local ch = wait_channels.lut[sh]
	localt = Body.get_time()
	local meta, ranges
	repeat
		-- Do not block
    local metadata, has_more = ch:receive(true)
		-- If no msg, then process
		if not metadata then break end
		-- Must have a pair with the range data
		assert(has_more,"metadata and not raw data!")
		meta = mp.unpack(metadata)
		ranges, has_more = ch:receive(true)
	until false
	-- Update the points
	if meta.n~=ch.n then
		print('Set up the channel!',ch.n,meta.n)
		setup_ch(ch,meta)
	end
	-- Update the count
	ch.cnt = ch.cnt + 1
	ch.t = meta.t
	ch.ts = Body.get_time() - t0
	-- Place into storage
	cutil.string2storage(ranges,ch.raw:storage())
	-- Perform detection
	detect( ch )
end

-- Begin the subscriptions
local camera0_ch = simple_ipc.new_subscriber'camera0'
camera0_ch.name = 'head'
camera0_ch.callback = ch_cb
table.insert(wait_channels, camera0_ch)

-- Begin the devices
if OPERATING_SYSTEM~='darwin' then
	local camera0_dev = {}
	camera0_dev.name = 'camera0'
	camera0_dev.callback = dev_cb
	setup_dev(camera0_dev,Config.camera.head)
	table.insert(wait_channels, camera0_dev)
end

-- Close the camera properly upon Ctrl-C
local signal = require'signal'
local function shutdown()
  print'Shutting down the Cameras...'
  for _,item in ipairs(wait_channels) do
		if item.uvc then
    	item.uvc:close()
    	print('Closed camera',item.name)
		end
  end
  os.exit()
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

-- Make the poller
local channel_poll = simple_ipc.wait_on_channels( wait_channels )
channel_poll:start()
