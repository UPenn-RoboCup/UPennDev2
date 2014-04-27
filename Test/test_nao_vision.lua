dofile'../include.lua'
local libLog = require'libLog'
local torch = require'torch'
local util = require'util'
local cutil = require'cutil'
-- Test the old ImageProc
local ImageProc = require'ImageProc'
local ImageProc2 = require'ImageProc.ffi'
-- Test Ball Detection
local lV = require'libVision'

lut = 'nao3'
date = '09.17.2009.00.09.00'
DIR = HOME..'/Data/'
local replay = libLog.open(DIR,date,'yuyv')
local metadata = replay:unroll_meta()
print('Unlogging',#metadata,'images')
--
local w, h = metadata[1].w, metadata[1].h

-- For broadcasting the labeled image
local zlib = require'zlib.ffi'
local c_zlib = zlib.compress_cdata
local mp    = require'msgpack.MessagePack'
local udp = require'udp'
local udp_ch = udp.new_sender('127.0.0.1', 33333)
local jpeg = require'jpeg'
local c_yuyv = jpeg.compressor'yuyv'

local scaleA, scaleB = 2, 2

local meta_a = {
  w = w / scaleA,
  h = h / scaleA,
  c = 'zlib',
}
local meta_b = {
  w = (w / scaleA) / scaleB,
  h = (h / scaleA) / scaleB,
  c = 'zlib',
}
local meta_j = {
  w = w,
  h = h,
  c = 'jpeg',
}

-- nElement
local a_sz = w / scaleA * h / scaleA
local b_sz = a_sz / (scaleB * scaleB)

local labelA_t, labelB_t, labelA_ud, labelB_ud

-- Load the one colortable
local lut_id = ImageProc2.load_lut (HOME.."/Data/lut_"..lut..".raw")
ImageProc2.setup(w, h)
lut_t = ImageProc2.get_lut(lut_id)
lut_ud = cutil.torch_to_userdata(lut_t) 
lut = lut_t:data()

lV.entry (w, h, 2, 2)

while true do
local d = replay:log_iter(metadata,'yuyv')
local meta, yuyv_t, image, t0, t1
for i,m,r in d do
	t0 = unix.time()
	meta = m
  
  -- Form the images from the log
	--yuyv_t = torch.ByteTensor(r:storage(),1,torch.LongStorage({h,2*w}))
  yuyv_t = torch.ByteTensor(r:storage())
  image = cutil.torch_to_userdata(yuyv_t)
  
  -- Test the old method
  ----[[
  t0 = unix.time()
  labelA_ud = ImageProc.yuyv_to_label(image, lut_ud, w, h, scaleA)
  cc = ImageProc.color_count(labelA_ud, a_sz)
  labelB_ud = ImageProc.block_bitor(labelA_ud, w, h, scaleB, scaleB)
  t1 = unix.time()
  print('Old Processing Time (ms)', 1e3*(t1-t0))
  --]]
  
  -- Test the new method
  t0 = unix.time()
  labelA_t = ImageProc2.yuyv_to_label(image, lut)
  cc_t = ImageProc2.color_count(labelA_t)
  labelB_t = ImageProc2.block_bitor(labelA_t)
  t1 = unix.time()
  print('New Processing Time (ms)', 1e3*(t1-t0))
  
  -- Vision pipeline

  local ball = lV.ball(labelA_t, labelB_t, cc_t)
  if type(ball)=='string' then print(ball) end
  meta_a.ball = ball
  meta_b.ball = ball
  
  -- Send on UDP
  -- Send labelA
  lA_z = c_zlib( labelA_t:data(), a_sz, true )
  local udp_ret, err = udp_ch:send( mp.pack(meta_a)..lA_z )
  -- Or Send labelB :)
  --lB_z = c_zlib( labelB_t:data(), b_sz, true )
  --local udp_ret, err = udp_ch:send( mp.pack(meta_b)..lB_z )
  -- Send JPEG image
  yuyv_j = c_yuyv:compress(yuyv_t,w,h)
  local udp_ret, err = udp_ch:send( mp.pack(meta_j)..yuyv_j )
  unix.usleep(5e5)
end
-- Loop forever
end

-- Save the data
-- Save to file
print('Saving labelA',labelA_t:size(1),labelA_t:size(2))
local f_l = torch.DiskFile('labelA.raw', 'w')
f_l.binary(f_l)
f_l:writeByte(labelA_t:storage())
f_l:close()
--[[ In MATLAB:
fid = fopen('labelA.raw');A = fread(fid);fclose(fid);
A = reshape(A,[160,120])'; figure(1); imagesc(A);
--]]

print('Saving labelB',labelB_t:size(1),labelB_t:size(2))
local f_l = torch.DiskFile('labelB.raw', 'w')
f_l.binary(f_l)
f_l:writeByte(labelB_t:storage())
f_l:close()
--[[ In MATLAB:
fid = fopen('labelB.raw');B = fread(fid);fclose(fid);
B = reshape(B,[80,60])'; figure(2); imagesc(B);
--]]

--[[
local jpeg = require'jpeg'
c_yuyv = jpeg.compressor('yuyv')
local str = c_yuyv:compress(img, w, h)
local f_y = io.open('yuyv.jpeg','w')
f_y:write(str)
f_y:close()
--]]