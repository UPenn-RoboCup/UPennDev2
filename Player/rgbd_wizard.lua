dofile'../include.lua'
local openni = require 'openni'
local signal = require'signal'
local n_users = openni.startup()

assert(n_users==0,'Should not use skeletons')

-- Verify stream
local depth_info, color_info = openni.stream_info()
assert(depth_info.width==320,'Bad depth resolution')
assert(color_info.width==320,'Bad color resolution')
print('Verified depth information...')

--[[
local jp = require'jpeg'
local mp = require'msgpack'

-- Set up the UDP sending
local udp = require'udp'
local udp_jdepth = udp.new_sender('localhost',43230)
local udp_jcolor = udp.new_sender('localhost',43231)

-- Set up the ZMQ sending
local simple_ipc = require'simple_ipc'
rgbd_color_ch = simple_ipc.new_publisher('rgbd_color')
rgbd_depth_ch = simple_ipc.new_publisher('rgbd_depth')
--]]

-- Set up timing debugging
local cnt = 0;
local t_last = unix.time()
local t_debug = 1

function shutdown()
  print'Shutting down the OpenNI device...'
  openni.shutdown()
  error('Finished!')
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

-- Start loop
while true do
	
	-- Acquire the Data
	local depth, color = openni.update_rgbd()
	
	-- Check the time of acquisition
	local t = unix.time()
  --[[
  
	-- Save the metadata
	local metadata = {}
	metadata.t = t
	local packed_metadata = mp.pack(metadata)
  
  -- Compress the payload
	jdepth = jpeg.compress_16(depth,320,240,4)
	jcolor = jpeg.compress_rgb(color,320,240)
	
	-- Send over UDP
	local packed_depth = mp.pack({meta=metadata,raw=jdepth})
	udp_jdepth:send( packed_depth )
	local packed_color = mp.pack({meta=metadata,raw=jcolor})
	udp_jcolor:send( packed_color )
	
	-- Send over ZMQ
	rgbd_depth_ch:send({packed_metadata,jdepth})
	rgbd_color_ch:send({packed_metadata,jcolor})
	
  --]]
  
	-- Debug the timing
  cnt = cnt+1;
  if t-t_last>t_debug then
    local msg = string.format("%.2f FPS", cnt/t_debug)
    io.write("RGBD Wizard | ",msg,'\n')
    t_last = t
    cnt = 0
  end
	
end