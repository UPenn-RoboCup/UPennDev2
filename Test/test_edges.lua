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
  edge_t, edge_char_t, grey_t = ImageProc2.yuyv_to_edge(yuyv_t:data())
end

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
edge_t:zero()
edge_t:select(1, 40):fill(2)
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