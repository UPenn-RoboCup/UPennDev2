dofile'../include.lua'
local Body   = require'Body'
local openni = require 'openni'
local signal = require'signal'
local carray = require'carray'
local torch  = require'torch'
torch.Tensor = torch.DoubleTensor
require'vcm'

local n_users = openni.startup()
assert(n_users==0,'Should not use skeletons')

-- Verify stream
local WIDTH, HEIGHT = 320, 240
local DEPTH_NELEMENTS = WIDTH*HEIGHT
local depth_info, color_info = openni.stream_info()
assert(depth_info.width==WIDTH,'Bad depth resolution')
assert(color_info.width==WIDTH,'Bad color resolution')

-- Require some modules
local png  = require'png'
local mp   = require'msgpack'
local jpeg = require'jpeg'

-- Access point
local depth, color
-- Double for computation effectiveness
local depths_t = torch.Tensor(DEPTH_NELEMENTS):zero()
-- Byte for range reduction
local d_byte = torch.ByteTensor(DEPTH_NELEMENTS):zero()

-- Settings
--use_zmq=true
use_udp=true

-- Set up the UDP sending
local udp_depth, udp_color
if use_udp then
  local udp = require'udp'
  local dport,cport = Config.net.rgbd_depth,Config.net.rgbd_color
  local op_addr = Config.net.operator.wired
  print('Connected to ports',dport,cport)
  print('Operator:',op_addr)
  udp_depth = udp.new_sender(op_addr,dport)
  udp_color = udp.new_sender(op_addr,cport)
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
local t_last = Body.get_time()
local t_debug = 5

function shutdown()
  print'Shutting down the OpenNI device...'
  openni.shutdown()
  os.exit()
end
signal.signal("SIGINT",  shutdown)
signal.signal("SIGTERM", shutdown)

-- TODO: single frame is reliable TCP, not UDP
local t_last_color_udp = Body.get_time()
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
  t_last_color_udp = Body.get_time()
  if net_settings[1]==1 then
    net_settings[1] = 0
    vcm.set_kinect_net_color(net_settings)
    return
  end
end

local t_last_depth_udp = Body.get_time()
local function send_depth_udp(metadata)

  local net_settings = vcm.get_kinect_net_depth()
  -- Streaming
  local stream = net_settings[1]
  if stream==0 then return end

  -- Dynamic Range Compression
  local container_t = torch.ShortTensor(torch.ShortStorage(depth,DEPTH_NELEMENTS))
  -- Sensitivity range in meters
  local depths = vcm.get_kinect_depths()
  metadata.depths = depths
  -- convert to millimeters
  local near = depths[1]*1000
  local far  = depths[2]*1000
  -- Safety check
  if near>=far then return end
  -- reduce the range
  depths_t:copy(container_t):add(-near):mul(255/(far-near))
  -- Ensure that we are between 0 and 255
  depths_t[torch.lt(depths_t,0)] = 0
  depths_t[torch.gt(depths_t,255)] = 255
  -- Copy to a byte for use in compression
  d_byte:copy(depths_t)
  -- Compression
  local c_depth
  if net_settings[2]==1 then
    metadata.c = 'jpeg'
    jpeg.set_quality( net_settings[3] )
    c_depth = jpeg.compress_gray(d_byte:storage():pointer(),
      depth_info.width,depth_info.height)
  elseif net_settings[2]==2 then
    -- zlib
    metadata.c = 'zlib'
    c_mesh = zlib.compress(
      d_byte:storage():pointer(),
      d_byte:nElement()
    )
  elseif net_settings[2]==3 then
    -- png
    metadata.c = 'png'
    c_depth = png.compress(d_byte:storage():pointer(),
     depth_info.width, depth_info.height, 1)
  else
    -- raw data?
    return
  end
  -- Metadata
  local meta = mp.pack(metadata)
  -- Send over UDP
  local ret_d,err_d = udp_depth:send( meta..c_depth )
  --print('sent',ret_d)
  if err_d then print(err_d) end
  t_last_depth_udp = Body.get_time()
  -- Tidy
  if stream==1 then
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
  local t = Body.get_time()
  
  -- Save the metadata  
  vcm.set_kinect_t(t)
  local metadata = {}
  metadata.t = t
  
  if use_zmq then
    local meta = mp.pack(meta)
    -- Send over ZMQ
    rgbd_color_ch:send({meta,carray.byte(color,WIDTH*HEIGHT*3)})
    rgbd_depth_ch:send({meta,carray.short(depth,WIDTH*HEIGHT)})
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