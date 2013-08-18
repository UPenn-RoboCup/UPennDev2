dofile'../include.lua'
local openni = require 'openni'
local signal = require'signal'
local n_users = openni.startup()
assert(n_users==0,'Should not use skeletons')

-- Verify stream
local depth_info, color_info = openni.stream_info()
assert(depth_info.width==320,'Bad depth resolution')
assert(color_info.width==320,'Bad color resolution')

-- Require some modules
require'vcm'
local jp = require'jpeg'
local mp = require'msgpack'

-- Settings
--use_shm=true
use_zmq=true
--use_udp=true

-- Set up the UDP sending
if use_udp then
  local udp = require'udp'
  udp_jdepth = udp.new_sender('localhost',43230)
  udp_jcolor = udp.new_sender('localhost',43231)
end

-- Set up the ZMQ sending
if use_zmq then
  local simple_ipc = require'simple_ipc'
  rgbd_color_ch = simple_ipc.new_publisher'rgbd_color'
  rgbd_depth_ch = simple_ipc.new_publisher'rgbd_depth'
end

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
  
	-- Save the metadata  
  vcm.set_kinect_t(t)
  -- Pack it, if needed...
  if use_zmq or use_udp then
    local metadata = {}
    metadata.t = t
    meta = mp.pack(meta)
  end
  
  --[[
  if use_shm then
    -- Store in SHM
    vcm.set_kinect_color(color)
    vcm.set_kinect_depth(depth)
  end
  --]]
  t_shm = unix.time()
	
  if use_zmq then
    -- Send over ZMQ
    rgbd_color_ch:send({meta,color})
    rgbd_depth_ch:send({meta,depth})
  end
  t_zmq = unix.time()

  if use_udp then
    -- Compress the payload
    jdepth = jpeg.compress_16(depth,depth_info.width,depth_info.height,4)
    jcolor = jpeg.compress_rgb(color,color_info.width,color_info.height)
    -- Send over UDP
    udp_jdepth:send( meta..jdepth )
    udp_jcolor:send( meta..jcolor )
  end
  t_udp = unix.time()
  
  print('SHM:',t_shm-t,'ZMQ:',t_zmq-t_shm,'UDP:',t_udp-t_zmq)
  print('Color:',#color,'Depth:',#depth)
  if use_udp then
    print('JColor:',#jcolor,'JDepth:',#jdepth)
  end
  
	-- Debug the timing
  cnt = cnt+1;
  if t-t_last>t_debug then
    local msg = string.format("%.2f FPS", cnt/t_debug)
    io.write("RGBD Wizard | ",msg,'\n')
    t_last = t
    cnt = 0
  end
	
end