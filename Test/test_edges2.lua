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
local line_ch = si.new_publisher('line')

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



-- Update the distance measurements!
local T = require'libTransform'
local Body = require'Body'
local K = Body.Kinematics
local last_measurement
--local focal_length = 184.75
--local focal_length = 160
--local focal_length = 200
local focal_length = 300
local sin, cos = math.sin, math.cos
local function update_dist (pline1, pline2, tr, t)
	-- Find the distance between the lines
	local p0 = vector.new{pline2.iMin, pline2.jMin}
	local p1 = vector.new{pline1.iMean, pline1.jMean}
	local p2 = vector.new{pline2.iMax, pline2.jMax}
	local a = vector.norm(p1 - p0)
	local b = vector.norm(p2 - p0)
	local c = vector.norm(p2 - p1)
	local angle = math.acos( (a^2+b^2-c^2) / (2 * a * b) )
	local px_width = math.abs(a * sin(angle))
	-- Get the angles
	local i_px1, i_px2 = pline1.iMean - (w / 2), pline2.iMean - (w / 2)
	local camera_angle1 = math.atan(i_px1 / focal_length)
	local camera_angle2 = math.atan(i_px2 / focal_length)
	--local angle_width = math.abs((camera_angle2 + camera_angle1) / 2)
	local angle_width = math.abs(camera_angle2 - camera_angle1)
	--print('angle_width',angle_width*RAD_TO_DEG)
	-- See if this is our first mesaurment
	if not last_measurement then
		print("UPDATE", T.get_pos(tr))
		last_measurement = {
			px_width = px_width,
			tr = tr,
			angle_width = angle_width,
			t = t,
		}
		return
	end

	-- If the less than a few pixels difference in width, then return
	local px_diff = math.ceil(px_width - last_measurement.px_width)

	if px_diff < 3 then
		--print('FAILED px_diff', px_diff, px_width)
		return
	end
	-- Check the distance between transforms
	local p_last, p_now = T.get_pos(last_measurement.tr), T.get_pos(tr)
	local p_diff = vector.norm(p_last - p_now)

	--util.ptorch(tr)
	--print('p_diff', p_diff)

	-- If less than an inch, discard
	if p_diff < 0.050 then
		--print('FAILED p_diff', p_diff)
		return
	else
		--print('SUCCESS p_diff', p_diff)
		--print('angle_width', RAD_TO_DEG*angle_width, RAD_TO_DEG*last_measurement.angle_width)
	end

	local angle_diff = angle_width - last_measurement.angle_width
	if math.abs(angle_diff)<1*DEG_TO_RAD then
		--print('FAILED angle_diff', angle_diff)
		return
	end

	local sin_diff = sin(angle_width - last_measurement.angle_width)
	--print('sin_diff',sin_diff)
	local s_a = sin(angle_width)
	local r = (s_a * sin(last_measurement.angle_width)) / sin_diff * p_diff
	local d = (s_a * cos(last_measurement.angle_width)) / sin_diff * p_diff
	-- Update the last_measurment
	--[[
	last_measurement = {
		px_width = px_width,
		tr = tr,
		angle_width = angle_width,
		t = t,
	}
	--]]
	print("\n\nr, d",r, d)
	print('p_diff, a_diff',p_diff, RAD_TO_DEG*angle_diff)
	print()
	-- Return the distance measurement
	return r, d
end







local meta, yuyv_t, edge_t
local computation_times, n_over = {}, 0
local kernel_t, use_horiz, use_vert = ImageProc2.dir_to_kernel('v'), true, true
util.ptorch(kernel_t)
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


	if t_total > 1/30 then
		print('\n',i)
		print('GC (ms)', t_gc*1e3, collectgarbage('count'))
		print('Over time! (ms)', i, t_total*1e3)
		print("yuyv_to_edge (ms)", t_edge*1e3)
		print("line_stats (ms)", t_new*1e3)
		--print("line_stats_old (ms)", t_old*1e3)
		n_over = n_over + 1
	end


	-- Update the distance
	--util.ptable(meta)
	local q = vector.new(meta.arm)
	local fk = K.forward_arm(q)
	local camera_roll = line_radon.ith / line_radon.NTH * math.pi
	camera_roll = camera_roll > (math.pi / 2) and (camera_roll - math.pi) or camera_roll
	local cam_roll_diff = camera_roll_last or camera_roll - camera_roll
	camera_roll_last = camera_roll
	--util.ptable(line_radon)
	--print('camera_roll',camera_roll)

	if i>185 and pline1 then
	--if i>610 and pline1 then

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



		pline1.iMin  = 2 * (pline1.iMin  + bbox[1])
		pline1.iMean = 2 * (pline1.iMean + bbox[1])
		pline1.iMax  = 2 * (pline1.iMax  + bbox[1])
		pline1.jMin  = 2 * (pline1.jMin  + bbox[3])
		pline1.jMean = 2 * (pline1.jMean + bbox[3])
		pline1.jMax  = 2 * (pline1.jMax  + bbox[3])
		pline2.iMin  = 2 * (pline2.iMin  + bbox[1])
		pline2.iMean = 2 * (pline2.iMean + bbox[1])
		pline2.iMax  = 2 * (pline2.iMax  + bbox[1])
		pline2.jMin  = 2 * (pline2.jMin  + bbox[3])
		pline2.jMean = 2 * (pline2.jMean + bbox[3])
		pline2.jMax  = 2 * (pline2.jMax  + bbox[3])




line_ch:send(mp.pack({
	name = 'pline',
	l1 = {
		{x=pline1.iMin,  y=pline1.jMin},
		{x=pline1.iMean, y=pline1.jMean},
		{x=pline1.iMax,  y=pline1.jMax},
	},
	l2 = {
		{x=pline2.iMin,  y=pline2.jMin},
		{x=pline2.iMean, y=pline2.jMean},
		{x=pline2.iMax,  y=pline2.jMax},
	},
	-- Relative placement
	bbox = bbox,
}))

update_dist (pline1, pline2, fk, meta.t)
print('i',i)
		-- Sleep a little
		unix.usleep(1e5)
	end



end

local t_total, t_max = 0, -math.huge
for _, t in ipairs(computation_times) do
  t_total = t_total + t
  if t>t_max then t_max = t end
end
t_avg = t_total / #computation_times
print('\nAverage time (ms)', t_avg*1e3, t_max*1e3, n_over / #computation_times)
print()
