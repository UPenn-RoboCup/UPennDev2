dofile'../include.lua'
local libLog = require'libLog'
local torch = require'torch'

date = '04.17.2014.16.34.17'
DIR = HOME..'/Logs/'
local replay = libLog.open(DIR,date)
local metadata = replay:unroll_meta()
print('Unlogging',#metadata,'images')

local d = replay:log_iter(metadata)

local meta, yuyv
for i,m,r in d do
	meta = m
	yuyv = r
end

-- Extract the Y-plane
-- Assume LuaJIT right now...
print('yuyv',yuyv,meta.w,meta.h)
--local y_plane = ffi.new('uint8_t[?]',meta.w*meta.h)
local y_img = torch.ByteTensor(meta.w,meta.h)
local y_plane = torch.ByteTensor(y_img)
local n = y_img:nElement()
y_plane:resizeAs(torch.ByteTensor(n))

local yi = 0
for i=1,n do
	y_plane[i] = yuyv[yi]
	yi = yi+2
end

-- Let's perform a convolution...
local kconv = torch.ByteTensor({
	{0,0,0},
	{0,1,0},
	{0,0,0}
})

--[[
local y2 = torch.DoubleTensor(y_img:size())
y2:copy(y_img)
local y3 = torch.ByteTensor(im2:size())
y3:copy(im2)
local y3 = torch.conv2(y2, kconv, 'F')
--]]
local y2 = torch.conv2(y_img, kconv, 'F')
y2:narrow(1,2,meta.w):narrow(2,2,meta.h)
y3 = y2:clone()

--[[
util = require'util'
util.ptorch(kconv)
--]]

-- Now let's save this to a JPEG for viewing
local jpeg = require'jpeg'
c_gray = jpeg.compressor('gray')
local str = c_gray:compress(y_img)
local f_y = io.open('y.jpg','w')
f_y:write(str)
f_y:close()

f_y = torch.DiskFile('y.raw', 'w')
f_y.binary(f_y)
f_y:writeByte(y_img:storage())
f_y:close()
