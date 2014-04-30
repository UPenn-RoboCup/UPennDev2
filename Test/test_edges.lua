dofile'../include.lua'
local libLog = require'libLog'
local torch = require'torch'
local ImageProc2 = require'ImageProc.ffi'
local util = require'util'
local bit = require'bit'

date = '04.17.2014.16.34.17'
DIR = HOME..'/Logs/'
local replay = libLog.open(DIR,date,'uvc')
local metadata = replay:unroll_meta()
print('Unlogging',#metadata,'images')
local d = replay:log_iter()

local w, h = metadata[1].w, metadata[1].h
ImageProc2.setup(w, h, 2, 2)

local meta, yuyv, edge_t, edge_char_t
for i,m,r in d do
	if i>#metadata/2 then break end
	local t0 = unix.time()
	meta = m
	yuyv = r
  edge_t, edge_char_t = ImageProc2.yuyv_to_edge(yuyv:data())
end

print('edge',edge_t:size(1),edge_t:size(2))

-- Now let's save this to a JPEG for viewing
--[[
local jpeg = require'jpeg'
c_gray = jpeg.compressor('gray')
local str = c_gray:compress(edge_char_t)
local f_y = io.open('edge_bin.jpeg','w')
f_y:write(str)
f_y:close()
--]]

f_y = torch.DiskFile('../Data/edge.raw', 'w')
f_y.binary(f_y)
f_y:writeInt(edge_t:storage())
f_y:close()

f_y = torch.DiskFile('../Data/edge_char.raw', 'w')
f_y.binary(f_y)
f_y:writeChar(edge_char_t:storage())
f_y:close()