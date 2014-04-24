dofile'../include.lua'
local libLog = require'libLog'
local torch = require'torch'
local util = require'util'
local lV = require'libVision'

lut = 'nao3'
date = '09.17.2009.00.09.00'
DIR = HOME..'/Data/'
local replay = libLog.open(DIR,date,'yuyv')
local metadata = replay:unroll_meta()
print('Unlogging',#metadata,'images')
--
local w, h = metadata[1].w, metadata[1].h
lV.setup(w, h)
lV.load_lut(HOME.."/Data/lut_"..lut..".raw")

local labelA_t, labelB_t = lV.get_labels()

-- For broadcasting the labeled image
local zlib = require'zlib.ffi'
local c_zlib = zlib.compress_cdata
local a_sz = labelA_t:nElement()
local lA_d = labelA_t:data()
-- Send on localhost
local mp    = require'msgpack.MessagePack'
local udp = require'udp'
local udp_ch = udp.new_sender('127.0.0.1', 33333)
local jpeg = require'jpeg'
local c_yuyv = jpeg.compressor'yuyv'

local meta_a = {
  w = labelA_t:size(2),
  h = labelA_t:size(1),
  c = 'zlib',
}
local meta_j = {
  w = w,
  h = h,
  c = 'jpeg',
}

while true do
local d = replay:log_iter(metadata,'yuyv')
local meta, yuyv_t
for i,m,r in d do
	local t0 = unix.time()
	meta = m
	yuyv_t = torch.ByteTensor(r:storage(),1,torch.LongStorage({h,w}))
  local t0 = unix.time()
  -- Set into a torch container
  lV.yuyv_to_labelA(yuyv_t:data())
  lV.form_labelB()
  local ball = lV.ball()
  print(ball)
  meta_a.ball = ball
  local t1 = unix.time()
  -- Send on UDP
  -- Send labelA
  lA_z = c_zlib( lA_d, a_sz, true )
  local udp_ret, err = udp_ch:send( mp.pack(meta_a)..lA_z )
  -- Send JPEG image
  yuyv_j = c_yuyv:compress(yuyv_t)
  local udp_ret, err = udp_ch:send( mp.pack(meta_j)..yuyv_j )
	print('Processing Time (ms)', 1e3*(t1-t0))
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