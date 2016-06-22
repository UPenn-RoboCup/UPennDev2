#!/usr/bin/env luajit
local ENABLE_NET = true
local ENABLE_LOG = true
-----------------------------------
-- Camera manager
-- (c) Stephen McGill, 2014
-- Deng, Xiang 2016 dxiang@seas.upenn.edu
-- Modify and Config Astra
-----------------------------------
dofile'../include.lua'
if type(arg)~='table' then IS_WEBOTS=true end
local si = require'simple_ipc'
local mp = require'msgpack.MessagePack'
local jpeg = require'jpeg'
local Body = require'Body'
print('here')
require'hcm'

local openni = require 'openni'
local get_time = Body.get_time

-- Grab the metadata for this camera
local metadata, camera_id
if type(arg)~='table' or not arg[1] then
  -- TODO: Find the next available camera
  camera_id = 1
  metadata = Config.camera[1]
else
  camera_id = tonumber(arg[1])
  if camera_id then
    metadata = assert(Config.camera[camera_id], 'Bad Camera ID')
  else
    for id, c in ipairs(Config.camera) do
      if arg[1] == c.name then
        camera_id = id
        metadata = c
        break
      end
    end
    assert(metadata, 'Bad camera name')
  end
end
print('here')
--[[
local hz_wrist_send = .5
local dt_wrist_send = 1/hz_wrist_send
local hz_head_send = 0.4
local dt_head_send = 1/hz_head_send
--]]
-- local hz_send_itty = metadata.name=='wrist' and 0.5 or 0.4
-- local dt_send_itty = 1/hz_send_itty

-- JPEG Compressor
local c_grey = jpeg.compressor('y')
local c_yuyv = jpeg.compressor('yuyv')
local c_yuyv2 = jpeg.compressor('yuyv')
-- c_yuyv2:quality(metadata.quality)
-- c_yuyv2:downsampling(metadata.downsampling)
-- c_grey:quality(metadata.quality)
-- c_grey:downsampling(metadata.downsampling)

local t_log = -math.huge
local LOG_INTERVAL = 1/5

local libLog, logger

-- Extract metadata information
-- local w = metadata.w
-- local h = metadata.h
-- local name = metadata.name
-- Who to send to
local operator
--operator = Config.net.operator.wireless
operator = Config.net.operator.wired
-- Network Channels/Streams
local camera_identifier = 'camera'..(camera_id-1)
camera_identifier='kinect2_color'
local stream = Config.net.streams[camera_identifier]
local udp_ch = stream and stream.udp and si.new_sender(operator, stream.udp)
local camera_ch = stream and stream.sub and si.new_publisher(stream.sub)
--
local ittybitty_identifier = 'ittybitty'..(camera_id-1)
local itty_stream = Config.net.streams[ittybitty_identifier]
local ittybitty_ch = si.new_publisher(itty_stream.sub)
local ittybitty_udp_ch = si.new_sender(Config.net.operator.wired, itty_stream.udp)
--
local field_tcp_ch = si.new_publisher(stream.tcp);


local depth_streams = Config.net.streams['camera1']
local depth_udp_ch = depth_streams and depth_streams.udp and si.new_sender(operator, depth_streams.udp)
local depth_net_ch = si.new_publisher(depth_streams.tcp)
local depth_ch = depth_streams and depth_streams.sub and si.new_publisher(depth_streams.sub)

-- print('Camera | ', operator, camera_identifier, stream.udp, udp_ch)

-- Metadata for the operator for compressed image data
local c_meta = {
  -- Required for rendering
  sz = 0,
  c = 'jpeg',
  -- Extra information
  t = 0,
  --id = name..'_camera',
  id = 'k2_rgb',
  n = 0,
}

local has_detection, detection = pcall(require, metadata.detection)
-- Send which camera we are using
-- if has_detection then detection.entry(metadata) end

-- LOGGING
if ENABLE_LOG then
  libLog = require'libLog'
  logger = libLog.new('yuyv', true)
end

local nlog = 0
local udp_ret, udp_err, udp_data
local t0 = get_time()
local t_debug = 0

--
local buffer = {}
local hz_buffer = 1
local dt_buffer = 1/hz_buffer
local nbuffer = 1
--
local hz_open_send = 0.5
local dt_open_send = 1/hz_open_send
--
local hz_outdoor_send = 15
local dt_outdoor_send = 1/hz_outdoor_send
--
local hz_indoor_send = 3
local dt_indoor_send = 1/hz_indoor_send
--
local t_buffer = -math.huge
local t_send = -math.huge

local depth, color, depth255

local n_users = openni.startup()


local function check_send(ch,udpch,msg)
local is_indoors = hcm.get_network_indoors()
local t = Body.get_time()

  -- Check the buffer
  local dt_buffer0 = t - t_buffer
  if is_indoors>0 and dt_buffer0 > dt_buffer then
    t_buffer = t
    table.insert(buffer, 1, msg)
    if #buffer>nbuffer then table.remove(buffer) end
  end
  if is_indoors==0 then
    buffer = {msg}
  end

  -- Check the sending
  local dt_send0 = t - t_send
  if is_indoors==0 and dt_send0 < dt_outdoor_send then return end
  if is_indoors>0 and dt_send0 < dt_indoor_send then return end
  t_send = t

  for i,m in ipairs(buffer) do
    if ch then ch:send(m) end
    --if udp_ch then udp_ret, udp_err = udp_ch:send(table.concat(m)) end
    if udpch then udp_ret, udp_err = udpch:send_triple(table.concat(m)) end
  end

end


local t_send_itty = -math.huge
local dt_send_field = 1
local t_send_field = -math.huge
local function update(img, sz, cnt, t)
  -- Update metadata
  c_meta.t = t
  c_meta.n = cnt
  local c_img = c_yuyv:compress(img, w, h)

  --[[
  c_meta.sz = #ittybitty_img
  local msg = {mp.pack(c_meta), ittybitty_img}
  --]]
  c_meta.sz = #c_img
  local msg = {mp.pack(c_meta), c_img}

  check_send(msg)

  -- local is_indoors = hcm.get_network_indoors()
  -- local dt_send_itty0 = t - t_send_itty
  -- if is_indoors==camera_id+1 and dt_send_itty0 > dt_send_itty then
  -- local ittybitty_img
  -- if metadata.crop then
  --   ittybitty_img = c_yuyv2:compress_crop(img, w, h, unpack(metadata.crop))
  -- else
  --     --ittybitty_img = c_yuyv2:compress(img, w, h)
  --     ittybitty_img = c_grey:compress(img, w, h)
  --   end
  --   local ret, err = ittybitty_udp_ch:send(ittybitty_img)
  --   t_send_itty = t
  --   print('Sent ittybitty', ret, err)
  -- end

  local dt_send_field0 = t - t_send_field
  if field_tcp_ch and dt_send_field0 > dt_send_field then
  field_tcp_ch:send(msg)
  t_send_field = t
end

  -- Do the logging if we wish
  if ENABLE_LOG and (t - t_log > LOG_INTERVAL) then
    t_log = t
    nlog = nlog + 1
    -- metadata.rsz = sz
    -- metadata.head = Body.get_head_position()
    -- metadata.rpy = Body.get_rpy() 
    --for pname, p in pairs(pipeline) do metadata[pname] = p.get_metadata() end
    -- logger:record(metadata, img, sz)
    -- if nlog % 10 == 0 then
    --   print("# camera logs: "..nlog)
    --   if nlog % 100 == 0 then
    --     logger:stop()
    --     logger = libLog.new('yuyv', true)
    --     print('Open new log!')
    --   end
    -- end
  end

end

-- If required from Webots, return the table
if ... and type(...)=='string' and not tonumber(...) then
  return {entry=nil, update=update, exit=nil}
end

-- Open the camera
-- print('Opening', metadata.dev)
-- local camera = require'uvc'.init(metadata.dev, w, h, metadata.format, 1, metadata.fps)
--[[
os.execute('uvcdynctrl -d'..metadata.dev..' -s "Exposure, Auto 1"')
-- Set the params
for i, param in ipairs(metadata.auto_param) do
  local name, value = unpack(param)
  local before = camera:get_param(name)
  local ok = camera:set_param(name, value)
--  unix.usleep(1e5)
--  local now = camera:get_param(name)
  if not ok then
    print(string.format('Failed to set %s: from %d to %d', name, before, value))
  end
end
--]]
-- Set the params
-- for i, param in ipairs(metadata.param) do
--   local name, value = unpack(param)
--   camera:set_param(name, value)
--   unix.usleep(1e5)
--   local now = camera:get_param(name)
--   -- TODO: exposure
--   local count = 0
--   --while count<5 and now~=value do
--   local ok
--   while count<5 and not ok do
--     ok = camera:set_param(name, value)
--     unix.usleep(1e5)
--     count = count + 1
--     --now = camera:get_param(name)
--   end
--   if now~=value then
--     print(string.format('Failed to set %s: %d -> %d',name, value, now))
--   end
-- end

-- Cleanly exit on Ctrl-C
local running = true
local function shutdown()
  print('Goodbye stupid')
  openni.shutdown()
  running = false
end

local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

while running do
 --local img, sz, cnt, t = camera:get_image()  
 local t = Body.get_time()
  --update(img, sz, cnt, t)
  depth, color, depth255 = openni.update_rgbd()
  local c_depth = jpeg.compressor('y')
  local c_rgb = jpeg.compressor('rgb')
  WIDTH=320
  HEIGHT=240
  c_meta.t = t
  c_meta.n = 0
  --local c_img = c_rgb:compress(color, WIDTH,HEIGHT)
  --local c_img = c_depth:compress(depth, WIDTH,HEIGHT)

  local c_img = c_depth:compress(depth255, WIDTH,HEIGHT)  
  c_meta.sz = #c_img
  local msg = {mp.pack(c_meta), c_img}
  check_send(depth_ch, depth_udp_ch, msg)

    c_img = c_rgb:compress(color, WIDTH,HEIGHT)
  c_meta.sz = #c_img
  msg = {mp.pack(c_meta), c_img}
  check_send(camera_ch, udp_ch, msg)

  if t-t_debug>1 then
    t_debug = t
    local kb = collectgarbage('count')
    local debug_str = {
    string.format("Cam %s | %ds, %d kB", name, t-t0, kb),
  }
  -- print(table.concat(debug_str,'\n'))
end
collectgarbage'step'
end

-- #!/usr/bin/env luajit
-- dofile'../include.lua'
-- local signal = require'signal'
-- local Body   = require'Body'
-- local openni = require 'openni'

-- local ffi = require'ffi'

-- --------------------------------
-- local si = require'simple_ipc'
-- require'hcm'
-- local jpeg = require'jpeg'
-- camera_id=1
-- local camera_identifier = 'camera'..(camera_id-1)
-- name='head'
-- local stream = Config.net.streams[camera_identifier]
-- local field_tcp_ch = si.new_publisher(stream.tcp);
-- local camera_ch = stream and stream.sub and si.new_publisher(stream.sub)
-- local udp_ch = stream and stream.udp and si.new_sender(operator, stream.udp)

-- local dt_send_field = 1
-- local t_send_field = -math.huge
-- local t_buffer = -math.huge
-- local t_send = -math.huge
-- local hz_outdoor_send = 15
-- local dt_outdoor_send = 1/hz_outdoor_send
-- --
-- local hz_indoor_send = 3
-- local dt_indoor_send = 1/hz_indoor_send
-- local c_yuyv = jpeg.compressor('yuyv')
-- -- Metadata for the operator for compressed image data
-- local c_meta = {
--   -- Required for rendering
--   sz = 0,
--   c = 'jpeg',
--   -- Extra information
--   t = 0,
--   id = name..'_camera',
--   n = 0,
-- }
-- --------------------------------


-- --local carray = require'carray'
-- local torch  = require'torch'
-- torch.Tensor = torch.DoubleTensor
-- if torch then
--   print('torch')
-- end
-- require'mcm'
-- require'wcm'
-- require'gcm'

-- local n_users = openni.startup()
-- assert(n_users==0,'Should not use skeletons')

-- -- Verify stream
-- local WIDTH, HEIGHT = 320, 240
-- local DEPTH_NELEMENTS = WIDTH*HEIGHT
-- local depth_info, color_info = openni.stream_info()
-- assert(depth_info.width==WIDTH,'Bad depth resolution')
-- assert(color_info.width==WIDTH,'Bad color resolution')

-- -- Require some modules
-- local png  = require'png'
-- local mp   = require'msgpack'
-- --local jpeg = require'jpeg'

-- -- Access point
-- local depth, color, depth255
-- -- Double for computation effectiveness
-- --local depths_t = torch.Tensor(DEPTH_NELEMENTS):zero()
-- -- Byte for range reduction
-- --local d_byte = torch.ByteTensor(DEPTH_NELEMENTS):zero()

-- -- Settings
-- use_zmq=false
-- use_udp=false

-- -- Set up the UDP sending
-- local udp_depth, udp_color
-- if use_udp then
--   local udp = require'udp'
--   local dport,cport = Config.net.rgbd_depth,Config.net.rgbd_color
--   local op_addr = Config.net.operator.wired
--   print('Connected to ports',dport,cport)
--   print('Operator:',op_addr)
--   udp_depth = udp.new_sender(op_addr,dport)
--   udp_color = udp.new_sender(op_addr,cport)
-- end

-- -- Set up the ZMQ sending
-- local rgbd_color_ch, rgbd_depth_ch

-- ----------------------
-- local ok, ffi = pcall(require, 'ffi')
-- local t_log = -math.huge
-- local LOG_INTERVAL = 1/5
-- local ENABLE_LOG = false
-- local libLog, logger
-- if ENABLE_LOG then
--   libLog = require'libLog'
--   logger = libLog.new('yuyv', true)
-- end
-- -----------------------------------


-- if use_zmq then
--   local simple_ipc = require'simple_ipc'
--   rgbd_color_ch = simple_ipc.new_publisher'rgbd_color'
--   rgbd_depth_ch = simple_ipc.new_publisher'rgbd_depth'
-- end

-- -- Set up timing debugging
-- local cnt = 0;
-- local t_last = Body.get_time()
-- local t_debug = 5

-- function shutdown()
--   print'Shutting down the OpenNI device...'
--   openni.shutdown()
--   os.exit()
-- end
-- if signal.signal then
-- signal.signal("SIGINT",  shutdown)
-- signal.signal("SIGTERM", shutdown)
-- end

-- -- TODO: single frame is reliable TCP, not UDP
-- local t_last_color_udp = Body.get_time()
-- local function send_color_udp(metadata)
--   local net_settings = vcm.get_kinect_net_color()
--   -- Streaming
--   if net_settings[1]==0 then return end
--   -- Compression
--   local c_color
--   if net_settings[2]==1 then
--     metadata.c = 'jpeg'
--     c_color = jpeg.compress_rgb(color,color_info.width,color_info.height)
--   elseif net_settings[2]==2 then
--     metadata.c = 'png'
--     c_color = png.compress(color, color_info.width, color_info.height)
--   end
--   if not c_color then return end
--   -- Metadata
--   local meta = mp.pack(metadata)
--   -- Send over UDP
--   local ret_c,err_c = udp_color:send( meta..c_color )
--   t_last_color_udp = Body.get_time()
--   if net_settings[1]==1 then
--     net_settings[1] = 0
--     vcm.set_kinect_net_color(net_settings)
--     return
--   end
-- end

-- local t_last_depth_udp = Body.get_time()

-- -- the actual code that send the data
-- local function check_send(msg)
--   local is_indoors = hcm.get_network_indoors()
--   local t = Body.get_time()

--   -- Check the buffer
--   local dt_buffer0 = t - t_buffer
--   if is_indoors>0 and dt_buffer0 > dt_buffer then
--     t_buffer = t
--     table.insert(buffer, 1, msg)
--     if #buffer>nbuffer then table.remove(buffer) end
--   end
--   if is_indoors==0 then
--     buffer = {msg}
--   end

--   -- Check the sending
--   local dt_send0 = t - t_send
--   if is_indoors==0 and dt_send0 < dt_outdoor_send then return end
--   if is_indoors>0 and dt_send0 < dt_indoor_send then return end
--   t_send = t

--   for i,m in ipairs(buffer) do
--     if camera_ch then camera_ch:send(m) end
--     --if udp_ch then udp_ret, udp_err = udp_ch:send(table.concat(m)) end
--     if udp_ch then udp_ret, udp_err = udp_ch:send_triple(table.concat(m)) end
--   end

-- end

-- local function send_rgb_ipc(img,t)
--   c_meta.t = t
--   c_meta.n = 0
--   local c_img = c_yuyv:compress(img, WIDTH, HEIGHT)

--   --[[
--   c_meta.sz = #ittybitty_img
--   local msg = {mp.pack(c_meta), ittybitty_img}
--   --]]
--   c_meta.sz = #c_img
--   local msg = {mp.pack(c_meta), c_img}
--   local dt_send_field0 = t - t_send_field
--   if field_tcp_ch and dt_send_field0 > dt_send_field then
--     field_tcp_ch:send(msg)
--     t_send_field = t
--   end
-- end


-- local function send_depth_udp(metadata)

--   local net_settings = vcm.get_kinect_net_depth()
--   -- Streaming
--   local stream = net_settings[1]
--   if stream==0 then return end

--   -- Dynamic Range Compression
--   local container_t = torch.ShortTensor(torch.ShortStorage(depth,DEPTH_NELEMENTS))
--   -- Sensitivity range in meters
--   local depths = vcm.get_kinect_depths()
--   metadata.depths = depths
--   -- convert to millimeters
--   local near = depths[1]*1000
--   local far  = depths[2]*1000
--   -- Safety check
--   if near>=far then return end
--   -- reduce the range
--   depths_t:copy(container_t):add(-near):mul(255/(far-near))
--   -- Ensure that we are between 0 and 255
--   depths_t[torch.lt(depths_t,0)] = 0
--   depths_t[torch.gt(depths_t,255)] = 255
--   -- Copy to a byte for use in compression
--   d_byte:copy(depths_t)
--   -- Compression
--   local c_depth
--   if net_settings[2]==1 then
--     metadata.c = 'jpeg'
--     jpeg.set_quality( net_settings[3] )
--     c_depth = jpeg.compress_gray(d_byte:storage():pointer(),
--       depth_info.width,depth_info.height)
--   elseif net_settings[2]==2 then
--     -- zlib
--     metadata.c = 'zlib'
--     c_mesh = zlib.compress(
--       d_byte:storage():pointer(),
--       d_byte:nElement()
--     )
--   elseif net_settings[2]==3 then
--     -- png
--     metadata.c = 'png'
--     c_depth = png.compress(d_byte:storage():pointer(),
--      depth_info.width, depth_info.height, 1)
--   else
--     -- raw data?
--     return
--   end
--   -- Metadata
--   local meta = mp.pack(metadata)
--   -- Send over UDP
--   local ret_d,err_d = udp_depth:send( meta..c_depth )
--   --print('sent',ret_d)
--   if err_d then print(err_d) end
--   t_last_depth_udp = Body.get_time()
--   -- Tidy
--   if stream==1 then
--     net_settings[1] = 0
--     vcm.set_kinect_net_depth(net_settings)
--     return
--   end
-- end

-- -- Start loop
-- while true do

--   -- Acquire the Data
--   depth, color, depth255 = openni.update_rgbd()
--   -- print(depth255)
--   -- print(depth)
--   ---print(color)
--   --print(type(depth))
--   -- Check the time of acquisition
--   local t = Body.get_time()

--   -- Save the metadata  
--   --vcm.set_kinect_t(t)
--   local metadata = {}
--   metadata.t = t

--   local metadata_camera = {
--     head = Body.get_head_position(),
--     --head = Body.get_head_command_position(),
--     w = w,
--     h = h,
--     t = t,
--     c = 'yuyv',
--     id = camera_name,
--     --rpy = Body.get_rpy(),
--     tfL16 = tfL_flat,
--     tfG16 = tfG_flat,
--     HeadFSM = gcm.get_fsm_Head(),
--    -- GameFSM = gcm.get_fsm_Game(),
--     MotionFSM = gcm.get_fsm_Motion(),
--     BodyFSM = gcm.get_fsm_Body(),
--     tleft = gcm.get_game_timeleft(),
--   } 
--   sz=WIDTH*HEIGHT

--   local img_str = ffi.string(color, sz)

--    --print(img_str)
--   -- Log raw frames
--   if logger and (t - t_log > LOG_INTERVAL) then
--     t_log = t
--     metadata_camera.rsz = #img_str
--     logger:record(metadata_camera, img_str)
--     if logger.n % 10 == 0 then
--       print("# camera logs: ", logger.n)
--       if logger.n % 100 == 0 then
--         logger:stop()
--         print('Open new log!')
--         logger = libLog.new('yuyv', true)
--       end
--     end
--   end
-- --[
--   local jpeg = require'jpeg'
--   local c_rgb = jpeg.compressor('rgb')

--   local compress_str = c_rgb:compress(color, WIDTH, HEIGHT)
--   print('COMPRESSED', #compress_str)
--   local f = io.open('/tmp/img_.jpg','w')
--   f:write(compress_str)
--   f:close()

-- --------------------------------
--   -- Update metadata and send camera 

--   local c_depth = jpeg.compressor('y')
--   c_meta.t = t
--   c_meta.n = 0
--   --local c_img = c_rgb:compress(color, WIDTH,HEIGHT)
--   --local c_img = c_depth:compress(depth, WIDTH,HEIGHT)
--     local c_img = c_depth:compress(depth255, WIDTH,HEIGHT) 

--   --trash


--   ---local dstr=ffi.string(color, sz)
--   --local c_img = c_depth:compress(dstr, WIDTH,HEIGHT)
--   --print(depth)
--   --local ddata = depth:data()
--    --print(depth->getData())

--   c_meta.sz = #c_img
--   local msg = {mp.pack(c_meta), c_img}
--   check_send(msg)
--   --send_rgb_ipc(color,t)

-- --------------------------------

-- --os.exit()
-- --]]
--   if use_zmq then
--     local meta = mp.pack(meta)
--     -- Send over ZMQ
--     rgbd_color_ch:send({meta,ffi.string(color,WIDTH*HEIGHT*3)})
--     rgbd_depth_ch:send({meta,ffi.string(depth,WIDTH*HEIGHT)})
--   end

--   if use_udp then
--     send_color_udp(metadata)
--     send_depth_udp(metadata)
--     io.write("here \n")
--   end

--   -- Debug the timing
--   cnt = cnt+1;
--   if t-t_last>t_debug then
--     local msg = string.format("%.2f FPS", cnt/t_debug)
--     io.write("RGBD Wizard | ",msg,'\n')
--     t_last = t
--     cnt = 0
--   end
--   --print('eee')
-- end
