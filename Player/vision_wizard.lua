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
-- For all threads
local mp = require'msgpack.MessagePack'
-- Camera
local metadata = Config.camera[1]

-- Libraries
local lV    = require'libVision'
local uvc   = require'uvc'
local torch = require'torch'

-- Extract metadata information
local w = metadata.width
local h = metadata.height
local fps = metadata.fps
local fmt = metadata.format
local name = metadata.name
local dev = metadata.dev
metadata = nil
-- Extract Config information
local operator = Config.net.operator.wired
local udp_port = Config.net.camera[name]
Config = nil

-- Load the LUT
local lut_name = HOME.."/Data/lut_NaoV4_Grasp.raw"
--print("lut_name",lut_name)
local f_lut = torch.DiskFile(lut_name, 'r')
--print('Opened',f_lut)
f_lut.binary(f_lut)
-- We know the size of the LUT
local lut_s = f_lut:readByte(262144)
f_lut:close()
local lut = lut_s:data()

-- Open the camera
local camera = uvc.init(dev, w, h, fmt, 1, fps)

-- Torch container (Just to a pointer)
local yuyv_s = torch.ByteStorage(h*w*2,0)
local yuyv_t = torch.ByteTensor(yuyv_s)
local yuyv_sc = yuyv_s:cdata()
local labelA_t = torch.ByteTensor(h/2,w/2)
local color_count = torch.IntTensor(256):zero()

-- Loop temp vars
local img, sz, cnt, t

-- On ctrl-c
local function shutdown()
  -- Save to file
  print('Saving labelA',labelA_t:size(1),labelA_t:size(2))
  local f_l = torch.DiskFile('labelA.raw', 'w')
  f_l.binary(f_l)
  f_l:writeByte(labelA_t:storage())
  f_l:close()
  --
  local jpeg = require'jpeg'
  c_yuyv = jpeg.compressor('yuyv')
  local str = c_yuyv:compress(img,w,h)
  local f_y = io.open('yuyv.jpeg','w')
  f_y:write(str)
  f_y:close()
  --
  os.exit()
end
local signal = require'signal'
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

while true do
	-- Grab the image
	img, sz, cnt, t = camera:get_image()
  local t0 = unix.time()
  -- Set into a torch container
  yuyv_sc.data = ffi.cast("uint8_t*",img)
  lV.yuyv_to_labelA(yuyv_t, labelA_t, lut, color_count, w, h)
  local t1 = unix.time()
  --print(t1-t0)
end
