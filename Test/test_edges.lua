dofile'../include.lua'
local libLog = require'libLog'
local torch = require'torch'
local ImageProc2 = require'ImageProc.ffi'
local util = require'util'
local bit = require'bit'
local cutil = require'cutil'

date = '04.17.2014.16.34.17'
DIR = HOME..'/Logs/'
local replay = libLog.open(DIR,date,'uvc')
local metadata = replay:unroll_meta()
print('Unlogging',#metadata,'images')
local d = replay:log_iter()

local w, h = metadata[1].w, metadata[1].h
ImageProc2.setup(w, h, 2, 2)

local meta, yuyv_t, edge_t, edge_char_t
for i,m,r in d do
	if i>#metadata/2 then break end
	local t0 = unix.time()
	meta = m
	yuyv_t = r
end

edge_t, edge_char_t, grey_t = ImageProc2.yuyv_to_edge(yuyv_t:data(),{61, 91, 11, 111})

print('edge',edge_t:size(1),edge_t:size(2))

f_y = torch.DiskFile('../Data/edge.raw', 'w')
f_y.binary(f_y)
f_y:writeInt(edge_t:storage())
f_y:close()

f_y = torch.DiskFile('../Data/edge_char.raw', 'w')
f_y.binary(f_y)
f_y:writeChar(edge_char_t:storage())
f_y:close()

-- Now let's save this to a JPEG for viewing
local jpeg = require'jpeg'
c_gray = jpeg.compressor('gray')
c_yuyv = jpeg.compressor('yuyv')
-- -1 is 255 since 0xFF is -1 in signed stuff :)
--local str = c_gray:compress(edge_char_t:mul(-1))
local str = c_gray:compress(edge_char_t:mul(127):add(127))
local f_y = io.open('../Data/edge_bin.jpeg','w')
f_y:write(str)
f_y:close()

-- Save the image
str = c_yuyv:compress(yuyv_t, w, h )
f_y = io.open('../Data/edge_img.jpeg','w')
f_y:write(str)
f_y:close()

-- Save the greyscale
local grey_t2 = torch.ByteTensor(grey_t:size())
grey_t2:copy(grey_t)
str = c_gray:compress(grey_t2)
f_y = io.open('../Data/edge_gray.jpeg','w')
f_y:write(str)
f_y:close()

-- Try some line detection
--edge_t:zero():select(1, 40):fill(2)
local lines = ImageProc2.line_stats(edge_t,1)

f_y = torch.DiskFile('../Data/edge.raw', 'w')
f_y.binary(f_y)
f_y:writeInt(edge_t:storage())
f_y:close()

util.ptable(lines)

RadonTransform = ImageProc2.get_radon()
local counts_t, line_sums_t, line_min_t, line_max_t = RadonTransform.get_population()

print('edge',counts_t:size(1),counts_t:size(2))

f_y = torch.DiskFile('../Data/line_cnts.raw', 'w')
f_y.binary(f_y)
f_y:writeLong(counts_t:storage())
f_y:close()

f_y = torch.DiskFile('../Data/line_sums.raw', 'w')
f_y.binary(f_y)
f_y:writeLong(line_sums_t:storage())
f_y:close()

f_y = torch.DiskFile('../Data/line_min.raw', 'w')
f_y.binary(f_y)
f_y:writeLong(line_min_t:storage())
f_y:close()

f_y = torch.DiskFile('../Data/line_max.raw', 'w')
f_y.binary(f_y)
f_y:writeLong(line_max_t:storage())
f_y:close()

-- PCA on a bounding box
-- Save the cropped JPEG :)
c_yuyv:downsampling(2)
-- crop coords in orginal dimensions
str = c_yuyv:compress_crop(yuyv_t, w, h, 201, 100, 21, 200 )
f_y = io.open('../Data/edge_focus_img.jpeg','w')
f_y:write(str)
f_y:close()

-- Use 1 based indexing here...
-- dimensions in the subsample space... TODO: Make clearer
e, v, u, ys, us, vs, trans = ImageProc2.yuyv_color_stats(yuyv_t:data(), {61, 91, 11, 111})

--e, v, u, ys, us, vs, trans = ImageProc2.yuyv_color_stats(yuyv_t:data(), {11, 41, 11, 51})

print(ys:size(1),ys:size(2))
f_y = torch.DiskFile('../Data/ys_bbox.raw', 'w')
f_y.binary(f_y)
f_y:writeByte(ys:clone():storage())
f_y:close()

f_y = torch.DiskFile('../Data/us_bbox.raw', 'w')
f_y.binary(f_y)
f_y:writeByte(us:clone():storage())
f_y:close()

f_y = torch.DiskFile('../Data/vs_bbox.raw', 'w')
f_y.binary(f_y)
f_y:writeByte(vs:clone():storage())
f_y:close()

f_y = torch.DiskFile('../Data/trans_bbox.raw', 'w')
f_y.binary(f_y)
f_y:writeDouble(trans:storage())
f_y:close()