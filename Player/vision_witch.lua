-----------------------------------------------------------------
-- Combined Lidar manager for Team THOR
-- Reads and sends raw lidar data
-- As well as accumulate them as a map
-- and send to UDP/TCP
-- (c) Stephen McGill, Seung Joon Yi, 2013
---------------------------------
local IS_LOG = false
-- TODO: Critical section for include
-- Something there is non-reentrant
dofile'../include.lua'
-- Going to be threading this
local simple_ipc = require'simple_ipc'
-- For all threads
local mp = require'msgpack.MessagePack'
-- Camera
local cam_metadata = Config.camera[tonumber(arg[1]) or 1]

-- Libraries
local mp    = require'msgpack.MessagePack'
local lV    = require'libVision'
local uvc   = require'uvc'
local udp   = require'udp'
local torch = require'torch'

-- Extract cam_metadata information
local w = cam_metadata.w
local h = cam_metadata.h
local fps = cam_metadata.fps
local fmt = cam_metadata.format
local name = cam_metadata.name
local dev = cam_metadata.dev
local udp_port = cam_metadata.port

-- For broadcasting the labeled image
local zlib = require'zlib.ffi'
local c_zlib = zlib.compress_cdata
-- For broadcasting the YUYV image
local jpeg = require'jpeg'
c_yuyv = jpeg.compressor'yuyv'

-- For colortable logging, set fps to 5
fps = 5
local libLog, logger
if IS_LOG then
  libLog = require'libLog'
  -- Make the logger
  logger = libLog.new('yuyv',true)
end

-- Setup the Vision system
lV.setup(w, h)
lV.load_lut(HOME.."/Data/"..cam_metadata.lut)
-- Open the camera
local camera = uvc.init(dev, w, h, fmt, 1, fps)
-- Setup the parameters
for _,v in ipairs(cam_metadata.auto_param) do
  local param, val = v[1], v[2]
  camera:set_param(param,val)
  unix.usleep(1e5)
  camera:get_param(param)
end
for _,v in ipairs(cam_metadata.param) do
  local param, val = v[1], v[2]
  camera:set_param(param,val)
  unix.usleep(1e5)
  camera:get_param(param)
end

-- Network
local operator = Config.net.operator.wired
local udp_ch = udp.new_sender(operator, udp_port)

-- Remove unneeded
cam_metadata = nil
Config = nil

-- Loop vars
local img, sz, cnt, t
local labelA_t, labelB_t = lV.get_labels()
local a_sz = labelA_t:nElement()
local lA_d = labelA_t:data()
local yuyv_j, lA_z

-- On ctrl-c, saev some data
local function shutdown()
  -- Stop logging
  if IS_LOG then logger:stop() end
  -- Save data to files
  print('Saving labelA',labelA_t:size(1),labelA_t:size(2))
  local f_l = torch.DiskFile('labelA.raw', 'w')
  f_l.binary(f_l)
  f_l:writeByte(labelA_t:storage())
  f_l:close()
  --
  print('Saving zlib labelA',#lA_z)
  local f_z = io.open('labelA_z.raw','w')
  f_z:write(lA_z)
  f_z:close()
  --[[
  --MATLAB
  fid = fopen('labelA_z.raw');Az = fread(fid,Inf,'uint8');fclose(fid);
  A = reshape(zlibUncompress(cast(Az,'uint8')),[320,240]);
  --]]
  --
  local str = c_yuyv:compress(img,w,h)
  local f_y = io.open('labelA.jpeg','w')
  f_y:write(str)
  f_y:close()
  --
  os.exit()
end
local signal = require'signal'
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

local meta = {
  w = labelA_t:size(2),
  h = labelA_t:size(1),
  c = 'zlib',
}

-- YUYV size
-- We do not trust the Aldebaran implementation
local sz = w * h * 2
while true do
  -- Grab the image
  img, sz_bad, cnt, t = camera:get_image()
  local t0 = unix.time()
  -- Set into a torch container
  lV.yuyv_to_labelA(img)
  lV.form_labelB()
  local ball = lV.ball()
  --if type(ball)=='string' then print(ball) end
  meta.ball = ball
  -- Now we can detect the ball, etc.
  -- Send to the monitor
  local t1 = unix.time()
  print(t1-t0)
  -- Log the raw YUYV for colortables
  if IS_LOG then
    logger:record({w = w, h = h, rsz=sz}, img, sz)
  end
  -- Send labelA
  lA_z = c_zlib( lA_d, a_sz, true )
  meta.c = 'zlib'
  local udp_ret, err = udp_ch:send( mp.pack(meta)..lA_z )
  -- Send JPEG image
  yuyv_j = c_yuyv:compress(img,w,h)
  meta.c = 'jpeg'
  local udp_ret, err = udp_ch:send( mp.pack(meta)..yuyv_j )
end
