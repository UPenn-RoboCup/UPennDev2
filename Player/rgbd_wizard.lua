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
local jpeg = require'jpeg'
jpeg.set_quality( 80 )
local png = require'png'
local mp = require'msgpack'

-- Access point
local depth, color

-- Settings
--use_zmq=true
use_udp=true

-- Set up the UDP sending
local udp_depth, udp_color
if use_udp then
  local udp = require'udp'
  udp_depth = udp.new_sender('localhost',43230)
  udp_color = udp.new_sender('localhost',43231)
end

-- Set up the ZMQ sending
local rgbd_color_ch, rgbd_depth_ch
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
signal.signal("SIGINT",  shutdown)
signal.signal("SIGTERM", shutdown)

-- TODO: single frame is reliable TCP, not UDP
local t_last_color_udp = unix.time()
local function send_color_udp(metadata)
  local net_settings = vcm.get_kinect_net_color()
  -- Streaming
  if net_settings[1]==0 then return end
  -- Compression
  local c_color
  if net_settings[2]==1 then
    metadata.c = 'jpeg'
    c_color = jpeg.compress_rgb(color,color_info.width,color_info.height)
  elseif net_settings[2]==2 then
    metadata.c = 'png'
    c_color = png.compress(color, color_info.width, color_info.height)
  end
  if not c_color then return end
  -- Metadata
  local meta = mp.pack(metadata)
  -- Send over UDP
  local ret_c,err_c = udp_color:send( meta..c_color )
  t_last_color_udp = unix.time()
  if net_settings[1]==1 then
    net_settings[1] = 0
    vcm.set_kinect_net_color(net_settings)
    return
  end
end

local t_last_depth_udp = unix.time()
local function send_depth_udp(metadata)
  local net_settings = vcm.get_kinect_net_depth()
  -- Streaming
  if net_settings[1]==0 then return end
  -- Compression
  local c_depth
  if net_settings[2]==1 then
    metadata.c = 'jpeg'
    c_depth = jpeg.compress_16(depth,depth_info.width,depth_info.height,4)
  elseif net_settings[2]==2 then
    metadata.c = 'png'
    c_depth = png.compress(depth, depth_info.width,depth_info.height, 2)
  end
  -- Metadata
  local meta = mp.pack(metadata)
  -- Send over UDP
  local ret_d,err_d = udp_depth:send( meta..c_depth )
  t_last_depth_udp = unix.time()
  if net_settings[1]==1 then
    net_settings[1] = 0
    vcm.set_kinect_net_depth(net_settings)
    return
  end
end

-- Start loop
while true do

  -- Acquire the Data
  depth, color = openni.update_rgbd()
  -- Check the time of acquisition
  local t = unix.time()
  
  -- Save the metadata  
  vcm.set_kinect_t(t)
  local metadata = {}
  metadata.t = t
  
  if use_zmq then
    local meta = mp.pack(meta)
    -- Send over ZMQ
    rgbd_color_ch:send({meta,color})
    rgbd_depth_ch:send({meta,depth})
  end

  if use_udp then
    send_color_udp(metadata)
    send_depth_udp(metadata)
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