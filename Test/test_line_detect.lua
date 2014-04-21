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

-- Now let's save this to a JPEG for viewing
local jpeg = require'jpeg'
c_gray = jpeg.compressor('gray')
local str = c_gray:compress(y_img)
print('I got',type(str),#str)

local f_y = io.open('y.jpg','w')
f_y:write(str)
f_y:close()
