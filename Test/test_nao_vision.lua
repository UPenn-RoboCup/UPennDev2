dofile'../include.lua'
local libLog = require'libLog'
local torch = require'torch'
local util = require'util'
local lV = require'libVision'

date = '04.17.2014.16.34.17'
DIR = HOME..'/Logs/'
local replay = libLog.open(DIR,date)
local metadata = replay:unroll_meta()
print('Unlogging',#metadata,'images')
local d = replay:log_iter(metadata)
--
lV.setup(metadata[1].w, metadata[1].h)
lV.load_lut(HOME.."/Data/lut_low_contrast_pink_n_green.raw")

local meta, yuyv_t
for i,m,r in d do
	if i>#metadata/2 then break end
	local t0 = unix.time()
	meta = m
	yuyv_t = r
  local t0 = unix.time()
  -- Set into a torch container
  lV.yuyv_to_labelA(yuyv_t:data())
  lV.form_labelB()
  local t1 = unix.time()
	print('Processing Time (ms)', 1e3*(t1-t0))
end

-- Save the data
local labelA_t, labelB_t = lV.get_labels()
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