local detectLine = {}
local ok, ffi = pcall(require, 'ffi')
local ImageProc = require'ImageProc'
local ImageProc2 = require'ImageProc.ffi2'
local T = require'Transform'
local vector = require'vector'
local util = require'util'
local function bboxB2A(bboxB, scaleB)
	return {
		scaleB * bboxB[1],
		scaleB * bboxB[2] + scaleB - 1,
		scaleB * bboxB[3],
		scaleB * bboxB[4] + scaleB - 1,
	}
end
local label_flag, grid_x, grid_y, th_min_area, min_black_fill_rate
local th_aspect_ratio, th_max_height, th_min_height, th_min_orientation
local min_ground_fill_rate
local colors
local config
function detectLine.entry(cfg, Image)
  config = cfg
  colors = Image.colors
end

local libLog = require'libLog'

-- TODO: Needs to know the ball centroid, so as to avoid categorizing that one
function detectLine.update(Image)
  if type(Image)~='table' then
    return false, 'Bad Image'
  end
  local lines = {}
  lines.detect = 0

	--[[
  local linePropsB = ImageProc.field_lines(
		tonumber(ffi.cast('intptr_t', ffi.cast('void *', Image.labelB_d))),
		Image.wb, Image.hb,
		config.max_width,
    config.connect_th, config.max_gap, config.min_length
	)
	--]]

	----[[
	local linePropsB, props = ImageProc2.field_lines(
		Image.labelB_d, Image.wb, Image.hb
	)
	--]]
	--[[
	local linePropsB, props = ImageProc2.field_lines(
		Image.labelA_d, Image.wa, Image.ha
	)
	--]]

	--[[
	local meta = {}
	meta.NTH = props.NTH
	meta.NR = props.NR
	meta.MAXR = props.MAXR
	meta.wb = Image.wb
	meta.hb = Image.hb
	meta.i0 = props.i0
	meta.j0 = props.j0
	meta.ijs = linePropsB

	libLog.one('labelB_d', meta, Image.labelB_d, ffi.sizeof(Image.labelB_d))
	libLog.one('count_d', meta, props.count_d, ffi.sizeof(props.count_d))
	libLog.one('line_sum_d', meta, props.line_sum_d, ffi.sizeof(props.line_sum_d))
	libLog.one('line_min_d', meta, props.line_min_d, ffi.sizeof(props.line_min_d))
	libLog.one('line_max_d', meta, props.line_max_d, ffi.sizeof(props.line_max_d))
	os.exit()
	--]]

  if type(linePropsB)~='table' or #linePropsB==0 then
    return false, 'None'
  end
	--util.ptable(linePropsB[1].endpoint)

	-- Position of the head now
	local pHead4 = T.position4(Image.tfL)
	--local pHead4 = T.position4(Image.tfG)

  lines.propsB = linePropsB
  lines.v = {}
	lines.endpoint = {}
  lines.angle = {}
	lines.length = {}

	local msgs = {}
  local num_line = 4

  for i=1, math.min(num_line, #linePropsB) do

		local passed = true
		local propsB = lines.propsB[i]
		--util.ptable(propsB)
		local bendpoint = propsB.endpoint

		if type(bendpoint)~='table' then
			msgs[i] = 'stupid1'
			passed = false
		else
			for i, bend in ipairs(bendpoint) do
				if type(bend)~='number' then
					passed = false
					msgs[i] = 'stupid2'
				end
			end
		end

		vector.new(bendpoint)
		if passed then
			if math.max(bendpoint[3], bendpoint[4]) < 4 then
				--passed = false
				msgs[i] = string.format('Too close to edge y')
			elseif math.max(bendpoint[1], bendpoint[2]) < 4 then
				--passed = false
				msgs[i] = string.format('Too close to edge x')
			end
		end

		local length
		if passed then
			length = math.sqrt(
	    	(bendpoint[1]-bendpoint[2])^2 +
	    	(bendpoint[3]-bendpoint[4])^2
			)
			local minLen = IS_WEBOTS and 38 or 50
			if length<minLen then
				passed = false
				msgs[i] = string.format('min_length: %.2f < %.2f', length, minLen)
			end
		end

		local vL_endpoint, vG_endpoint
		if passed then
			local scale = 1
			local v01 = vector.new{
		    Image.focalB,
		    -(bendpoint[1] - Image.x0B),
		    -(bendpoint[3] - Image.y0B),
		    scale,
		  }
			local vL1 = Image.tfL * (v01 / v01[4])
			local vG1 = Image.tfG * (v01 / v01[4])

			local v02 = vector.new{
		    Image.focalB,
				-(bendpoint[2] - Image.x0B),
		    -(bendpoint[4] - Image.y0B),
		    scale,
		  }
			local vL2 = Image.tfL * (v02 / v02[4])
			local vG2 = Image.tfG * (v02 / v02[4])

			-- Project to the ground
			local zHead = pHead4[3]
			vL_endpoint = {
				pHead4 + (vL1 - pHead4) * zHead / (zHead - vL1[3]),
				pHead4 + (vL2 - pHead4) * zHead / (zHead - vL2[3])
			}
			vG_endpoint = {
				pHead4 + (vG1 - pHead4) * zHead / (zHead - vG1[3]),
				pHead4 + (vG2 - pHead4) * zHead / (zHead - vG2[3])
			}

			local d1 = math.sqrt(vG_endpoint[1][1]^2+vG_endpoint[1][2]^2)
			local d2 = math.sqrt(vG_endpoint[2][1]^2+vG_endpoint[2][2]^2)
			--print('Line | d1, d2:', d1, d2)
			if d1<.2 or d2<.2 then
				passed = false
				msgs[i] = string.format('Center circle: %.2f, %.2f', d1, d2)
			end
			--[[
				local vlen = math.sqrt(
					(vendpoint[2][1]-vendpoint[1][1])^2 + (vendpoint[2][2]-vendpoint[1][2])^2
				)
			--]]
		end

    if passed then



			table.insert(lines.length, length)
			table.insert(lines.endpoint, Image.scaleB * bendpoint)
			table.insert(lines.v, vL_endpoint)
      table.insert(lines.angle,
				math.abs(math.atan2(
					vL_endpoint[1][2]-vL_endpoint[2][2],
					vL_endpoint[1][1]-vL_endpoint[2][1]
				))
			)

			--print('lines.endpoint', unpack(lines.endpoint))
			--print('lines.length', unpack(lines.length))
			--print('vL_endpoint', unpack(vL_endpoint))

			msgs[i] = 'Passed checks'
    end

  end -- end for

  lines.nLines = #lines.endpoint
	lines.detect = #lines.endpoint > 0 and 1 or 0

  return lines, table.concat(msgs, '\n')
end
function detectLine.exit()
end
return detectLine
