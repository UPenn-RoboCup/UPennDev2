dofile'../include.lua'
local libLog = require'libLog'
local torch = require'torch'
local ImageProc2 = require'ImageProc.ffi'
local util = require'util'
local bit = require'bit'
local vector = require'vector'

local mp = require'msgpack.MessagePack'
local si = require'simple_ipc'
local edge_ch = si.new_publisher('edge')
local camera_ch = si.new_publisher('camera0')
local tou_che = si.new_subscriber('touche')

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

local meta, yuyv_t, edge_t
local computation_times, n_over = {}, 0
local kernel_t, use_horiz, use_vert = ImageProc2.dir_to_kernel(), true, true
-- Form the default bounding box (in scaled down space...)
local bbox = {51, 101, 21, 111}
for i,m,r in d do
	if i>#metadata/2 then break end
	local t0 = unix.time()
	meta = m
	yuyv_t = r

  -- Collect garbage in regular intervals :)
  local gc_kb = collectgarbage('count')
  local t0_gc = unix.time()
  collectgarbage()
  local t1_gc = unix.time()
  local t_gc = t1_gc - t0_gc

	-- Check if there is an updated bounding box
	local bbox_data = tou_che:receive(true)
	if bbox_data then
		-- Just use the first one...
		local bb = mp.unpack(bbox_data[1])
		bbox = vector.new(bb.bbox) / 2
		for i,v in ipairs(bbox) do bbox[i] = math.ceil(v) end
		local dir = bb.dir
		print('BBOX', bbox, dir)
		kernel_t = ImageProc2.dir_to_kernel(dir)
		if dir=='v' then
			use_horiz, use_vert = false, true
		elseif dir=='h' then
			use_horiz, use_vert = true, false
		else
			use_horiz, use_vert = true, true
		end
		util.ptorch(kernel_t)
	end

  -- Form the edges
  local t0_edge = unix.time()
	-- True means use PCA. False means use the Y channel
  local edge_t, grey_t = ImageProc2.yuyv_to_edge(yuyv_t:data(), bbox, true, kernel_t)
  local t1_edge = unix.time()
  local t_edge = t1_edge-t0_edge

  -- New line detection
  local t0_new = unix.time()
  local RT = ImageProc2.radon_lines(edge_t, use_horiz, use_vert)
  local pline1, pline2, line_radon = RT.get_parallel_lines()
  local t1_new = unix.time()
  local t_new = t1_new-t0_new

  local t_total = t_gc + t_edge + t_new
  -- Save the times
  table.insert(computation_times, t_total)

  -- Print if found a line
	--[[
  if pline1 then
    util.ptable(pline1)
    print()
    util.ptable(pline2)
    print()
    util.ptable(line_radon)
  end
	--]]

  -- Broadcast on ZMQ
  local metapack = mp.pack({
    l1 = pline1,
    l2 = pline2,
    lr = line_radon,
    bbox = bbox,
    NTH = RT.NTH,
    MAXR = RT.MAXR,
		kernel = ffi.string(kernel_t:data(), ffi.sizeof'double' * kernel_t:nElement()),
		kh = kernel_t:size(1),
		kw = kernel_t:size(2),
  })

  -- Send the Image, line information
  local bb_w = bbox[2] - bbox[1] + 1
  local bb_h = bbox[4] - bbox[3] + 1
	assert(bb_h==grey_t:size(1), 'bad dims height '..bb_h..' '..grey_t:size(1))
	assert(bb_w==grey_t:size(2), 'bad dims width'..bb_w..' '..grey_t:size(2))
	local jstr = c_yuyv:compress(yuyv_t, w, h)
  edge_ch:send({
    metapack,
    jstr,
    ffi.string(RT.count_d, ffi.sizeof('int')*RT.MAXR*RT.NTH),
    ffi.string(grey_t:data(), ffi.sizeof('double') * grey_t:nElement() ),
    ffi.string(edge_t:data(), ffi.sizeof('double') * edge_t:nElement() ),
    --ffi.string(RT.line_sum_d, ffi.sizeof('int')*RT.MAXR*RT.NTH),
  })

	-- Send the image to the browser
	camera_ch:send({mp.pack(meta),jstr})

	if t_total > 1/30 then
		print('\n',i)
		print('GC (ms)', t_gc*1e3, collectgarbage('count'))
		print('Over time! (ms)', i, t_total*1e3)
		print("yuyv_to_edge (ms)", t_edge*1e3)
		print("line_stats (ms)", t_new*1e3)
		--print("line_stats_old (ms)", t_old*1e3)
		n_over = n_over + 1
	end

  -- Sleep a little
  unix.usleep(1e5)

end

local t_total, t_max = 0, -math.huge
for _, t in ipairs(computation_times) do
  t_total = t_total + t
  if t>t_max then t_max = t end
end
t_avg = t_total / #computation_times
print('\nAverage time (ms)', t_avg*1e3, t_max*1e3, n_over / #computation_times)
print()
