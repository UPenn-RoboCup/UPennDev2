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

-- Convolution kernel
local kern = torch.IntTensor({
	{0, 0, 2, 0, 0,},
	{0, 2, 4, 2, 0,},
	{2, 4, -30, 4, 2,},
	{0, 2, 4, 2, 0,},
	{0, 0, 2, 0, 0,}
})
local kern = torch.IntTensor({
	{0, 0, 0, 0, 0,},
	{0, 1, 2, 1, 0,},
	{2, 4, -20, 4, 2,},
	{0, 1, 2, 1, 0,},
	{0, 0, 0, 0, 0,}
})

-- TODO: Convolution placeholder for speed
local label = bit.bor(0x10,0x08)
print('label',label)
local y_int = torch.IntTensor()
local edge_bin = torch.ByteTensor()
local meta, yuyv, yuyv_sub, y_plane, u_plane, v_plane
local edge = torch.IntTensor()
local THRESH, lines = 500, nil
for i,m,r in d do
	if i>#metadata/2 then break end
	local t0 = unix.time()
	meta = m
	yuyv = r
	local w, h = meta.w, meta.h
	-- Get the sub_sampled planes
	yuyv_sub = r:reshape(h/2,w,4):sub(1,-1,1,w/2)
	-- Get the y-plane
	y_plane = yuyv_sub:select(3,1)
	u_plane = yuyv_sub:select(3,2)
	v_plane = yuyv_sub:select(3,3)
	--y1_plane = yuyv_sub:select(3,4)
	-- Perform the convolution on the Int y-plane
	y_int:resize(y_plane:size()):copy(y_plane)
	edge:resize(
		y_plane:size(1)+kern:size(1)/2,
		y_plane:size(2)+kern:size(2)/2
	)
	torch.conv2(edge,y_int,kern)
	-- Threshold (Somewhat not working...
	edge_bin:resize(edge:size())
	edge_bin[edge:lt(0)] = 0
	edge_bin[edge:gt(THRESH)] = label
	-- Run the Radon
	lines = ImageProc2.line_stats(edge_bin)
  if lines then util.ptable(lines) end
	local t1 = unix.time()
	--print('Processing Time',t1-t0,edge_bin,lines)
end
util.ptable(lines)
os.exit()

-- Now let's save this to a JPEG for viewing
local jpeg = require'jpeg'
c_gray = jpeg.compressor('gray')
--
local str = c_gray:compress(y_plane:clone())
local f_y = io.open('y_plane.jpeg','w')
f_y:write(str)
f_y:close()
--
local str = c_gray:compress(edge_bin)
local f_y = io.open('edge_bin.jpeg','w')
f_y:write(str)
f_y:close()

print('edge',edge:size(1),edge:size(2))
f_y = torch.DiskFile('edge.raw', 'w')
f_y.binary(f_y)
f_y:writeInt(edge:storage())
f_y:close()
