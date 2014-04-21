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
-- Do this in Int Space! Need negatives...
-- In the future, this will be a combo of Y U and V
local y_img = torch.IntTensor(meta.w,meta.h)
local y_plane = torch.IntTensor(y_img)
local n = y_img:nElement()
y_plane:resizeAs(torch.IntTensor(n))

local yi = 0
for i=1,n do
	y_plane[i] = yuyv[yi]
	yi = yi+2
end

-- Attempt conv2
k = torch.IntTensor({
	{0, 0, 1, 0, 0,},
	{0, 1, 2, 1, 0,},
	{1, 2, -15, 2, 1,},
	{0, 1, 2, 1, 0,},
	{0, 0, 1, 0, 0,}
})
c = torch.conv2(y_img,k,'V')
print("Conv2",c:size(1),c:size(2))

-- Now let's save this to a JPEG for viewing
local y_img_byte = torch.ByteTensor(meta.w,meta.h)
y_img_byte:copy(y_img)
local jpeg = require'jpeg'
c_gray = jpeg.compressor('gray')
local str = c_gray:compress(y_img_byte)
local f_y = io.open('y.jpg','w')
f_y:write(str)
f_y:close()

f_y = torch.DiskFile('y.raw', 'w')
f_y.binary(f_y)
f_y:writeInt(y_img:storage())
f_y:close()


f_y = torch.DiskFile('c.raw', 'w')
f_y.binary(f_y)
f_y:writeInt(c:storage())
f_y:close()
