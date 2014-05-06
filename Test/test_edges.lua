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

local jpeg = require'jpeg'
c_gray = jpeg.compressor('gray')
c_yuyv = jpeg.compressor('yuyv')

local meta, yuyv_t, edge_t, edge_char_t
local computation_times, n_over = {}, 0
for i,m,r in d do
	if i>#metadata/2 then break end
	local t0 = unix.time()
	meta = m
	yuyv_t = r
  
  ----[[
  -- Collect garbage in regular intervals :)
  local gc_kb = collectgarbage('count')
  local t0_gc = unix.time()
  collectgarbage()
  local t1_gc = unix.time()
  local t_gc = t1_gc - t0_gc

  -- Form the edges
  local t0_edge = unix.time()
  edge_t, grey_t = ImageProc2.yuyv_to_edge(yuyv_t:data(),{61, 91, 11, 111})
  local t1_edge = unix.time()
  local t_edge = t1_edge-t0_edge
  
  -- Old Line detection
  --local t0_old = unix.time()
  --ImageProc2.line_stats_old(edge_t,1)
  --local t1_old = unix.time()
  --local t_old = t1_old-t0_old
  
  -- New line detection
  local t0_new = unix.time()
  ImageProc2.line_stats(edge_t,1)
  local t1_new = unix.time()
  local t_new = t1_new-t0_new
  
  local t_total = t_gc + t_edge + t_new
  if t_total > 1/30 then
    print('\n',i)
    print('GC (ms)', t_gc*1e3, collectgarbage('count'))
    print('Over time! (ms)', i, t_total*1e3)
    print("yuyv_to_edge (ms)", t_edge*1e3)
    print("line_stats (ms)", t_new*1e3)
    --print("line_stats_old (ms)", t_old*1e3)
    n_over = n_over + 1
  end
  -- Save the times
  table.insert(computation_times, t_total)
  --print('GC (ms), (kB)', t_gc*1e3, gc_kb)
  --]]
end

----[[
local t_total, t_max = 0, -math.huge
for _, t in ipairs(computation_times) do
  t_total = t_total + t
  if t>t_max then t_max = t end
end
t_avg = t_total / #computation_times
print('\nAverage time (ms)', t_avg*1e3, t_max*1e3, n_over / #computation_times)
print()
--]]

-- Form the edges
local t0_edge = unix.time()
edge_t, grey_t = ImageProc2.yuyv_to_edge(yuyv_t:data(),{61, 91, 11, 111})
local t1_edge = unix.time()
local t_edge = t1_edge-t0_edge

-- New line detection
local t0_new = unix.time()
plines = ImageProc2.line_stats(edge_t,1)
local t1_new = unix.time()
local t_new = t1_new-t0_new

for i, line in ipairs(plines) do
  print('\nLine',i)
  util.ptable(line)
  -- Form the equation of the line
end

-- Save the YUYV image
str = c_yuyv:compress(yuyv_t, w, h )
f_y = io.open('../Data/edge_img.jpeg','w')
f_y:write(str)
f_y:close()

print('\nEdge Size:',edge_t:size(1),edge_t:size(2))
-- Save this edge
f_y = torch.DiskFile('../Data/edge.raw', 'w')
f_y.binary(f_y)
f_y:writeInt(edge_t:storage())
f_y:close()


-- The rest is just debug code :)

-- Threshold (Somewhat not working...)
-- TODO: Some other dynamic range compression
local edge_char_t = torch.CharTensor()
edge_char_t:resize(edge_t:size())
-- Form a char map between 0 and 255 for Radon...?
local thresh = thresh or 2000
edge_char_t:map(edge_t, function(g, l)
  if l>thresh then return 1
--    elseif l<-thresh then return -1
  else return 0 end
  end)

f_y = torch.DiskFile('../Data/edge_char.raw', 'w')
f_y.binary(f_y)
f_y:writeChar(edge_char_t:storage())
f_y:close()

-- Now let's save this to a JPEG for viewing
-- -1 is 255 since 0xFF is -1 in signed stuff :)
--local str = c_gray:compress(edge_char_t:mul(-1))
local str = c_gray:compress(edge_char_t:mul(127):add(127))
local f_y = io.open('../Data/edge_bin.jpeg','w')
f_y:write(str)
f_y:close()

-- Save the greyscale
local grey_t2 = torch.ByteTensor(grey_t:size())
grey_t2:copy(grey_t)
str = c_gray:compress(grey_t2)
f_y = io.open('../Data/edge_gray.jpeg','w')
f_y:write(str)
f_y:close()

f_y = torch.DiskFile('../Data/edge.raw', 'w')
f_y.binary(f_y)
f_y:writeInt(edge_t:storage())
f_y:close()

--util.ptable(lines)

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