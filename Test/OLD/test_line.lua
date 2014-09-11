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
	if i>#metadata/2 then break end
end
local w, h = meta.w, meta.h
print(yuyv:size(1),w*h*2)
yuyv = yuyv:reshape(h/2,w,4)
print("sub sz",yuyv:size(1),yuyv:size(2))
yuyv_sub = yuyv:sub(1,-1,1,w/2):select(3,1):clone()
print("sub sz",yuyv_sub:size(1),yuyv_sub:size(2))
yuyv = yuyv:data()
--os.exit()

-- Extract the Y-plane
-- Do this in Int Space! Need negatives...
-- In the future, this will be a combo of Y U and V
local y_img = torch.IntTensor(h,w)
local y_plane = torch.IntTensor(y_img)
local n = y_img:nElement()
y_plane:resizeAs(torch.IntTensor(n))

local yi = 0
for i=1,n do
	y_plane[i] = yuyv[yi]
	yi = yi+2
end

-- Downsampled
local y_img2 = torch.IntTensor(h/2,w/2)
local y_plane2 = torch.IntTensor(y_img)
local y_img_b2 = torch.ByteTensor(h/2,w/2)
local y_img_plane2 = torch.ByteTensor(y_img_b2)
local n2 = y_img2:nElement()
y_plane2:resizeAs(torch.IntTensor(n2))
y_img_plane2:resizeAs(torch.ByteTensor(n2))
local yi = 0
local stride = 4 * w -- 4 bytes for w??? no...
local col = 1
for i=1,n2 do
	local y0 = yuyv[yi]
	local u  = yuyv[yi+1]
	local y1 = yuyv[yi+2]
	local v  = yuyv[yi+3]
	y_plane2[i] = y0
	y_img_plane2[i] = y0
	yi = yi+4
	if col==meta.w then
		yi = yi + stride
		col = 1
	else
		col = col + 1
	end
end

-- Attempt conv2
k = torch.IntTensor({
	{0, 0, 2, 0, 0,},
	{0, 2, 4, 2, 0,},
	{2, 4, -30, 4, 2,},
	{0, 2, 4, 2, 0,},
	{0, 0, 2, 0, 0,}
})
c = torch.conv2(y_img,k,'V')
c2 = torch.conv2(y_img2,k,'V')

-- Now let's save this to a JPEG for viewing
local jpeg = require'jpeg'
c_gray = jpeg.compressor('gray')
local str = c_gray:compress(y_img_b2)
local f_y = io.open('y.jpeg','w')
f_y:write(str)
f_y:close()
--
local str = c_gray:compress(yuyv_sub)
local f_y = io.open('y_sub.jpeg','w')
f_y:write(str)
f_y:close()

print('y_nosub',y_img:size(1),y_img:size(2))
f_y = torch.DiskFile('y.raw', 'w')
f_y.binary(f_y)
f_y:writeInt(y_img:storage())
f_y:close()

print('subdim',y_img2:size(1),y_img2:size(2))
f_y = torch.DiskFile('y2.raw', 'w')
f_y.binary(f_y)
f_y:writeInt(y_img2:storage())
f_y:close()

print("Conv",c:size(1),c:size(2))
f_y = torch.DiskFile('c.raw', 'w')
f_y.binary(f_y)
f_y:writeInt(c:storage())
f_y:close()

print("Conv",c2:size(1),c2:size(2))
f_y = torch.DiskFile('c2.raw', 'w')
f_y.binary(f_y)
f_y:writeInt(c2:storage())
f_y:close()
