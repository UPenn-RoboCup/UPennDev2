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
  local linePropsB = ImageProc.field_lines(
		tonumber(ffi.cast('intptr_t', ffi.cast('void *', Image.labelB_d))),
		Image.wb, Image.hb,
		config.max_width,
    config.connect_th, config.max_gap, config.min_length
	)

	--[[
	local linePropsB, props = ImageProc2.field_lines(
		Image.labelB_d, Image.wb, Image.hb
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
	meta.ijs = ijs

	libLog.one('labelB_d', meta, Image.labelB_d, ffi.sizeof(Image.labelB_d))
	libLog.one('count_d', meta, props.count_d, ffi.sizeof(props.count_d))
	libLog.one('line_sum_d', meta, props.line_sum_d, ffi.sizeof(props.line_sum_d))
	libLog.one('line_min_d', meta, props.line_min_d, ffi.sizeof(props.line_min_d))
	libLog.one('line_max_d', meta, props.line_max_d, ffi.sizeof(props.line_max_d))
	os.exit()
	--]]

  if #linePropsB==0 then
    return false, 'None'
  end
	--util.ptable(linePropsB[1].endpoint)

	-- Position of the head now
	local pHead4 = T.position4(Image.tfL)
	--local pHead4 = T.position4(Image.tfG)

  lines.propsB = linePropsB
  lines.v, lines.endpoint = {},{}
  lines.angle, lines.length= {},{}

	local msgs = {}
  local num_line = 4
  for i = 1, num_line do
    lines.endpoint[i] = vector.zeros(4)
    lines.v[i] = {
      vector.zeros(4),
      vector.zeros(4)
    }
    lines.angle[i] = 0
  end

  local bestindex = 1
  local bestlength = 0
  local linecount = 0

  local vendpoint, vHeight = {}, 0
  for i=1, math.min(num_line, #linePropsB) do

		local passed = true

    local length = math.sqrt(
    	(lines.propsB[i].endpoint[1]-lines.propsB[i].endpoint[2])^2 +
    	(lines.propsB[i].endpoint[3]-lines.propsB[i].endpoint[4])^2
		)

--[[
		print('found line...')
		util.ptable(lines.propsB[i])
		print('length', length)
		print('endpoint',unpack(lines.propsB[i].endpoint))
		print('===')
		--print('length', length)
		--]]

		-- TODO: Why this scale?
		local scale = 1
		local v01 = vector.new{
	    Image.focalB,
	    -(lines.propsB[i].endpoint[1] - Image.x0B),
	    -(lines.propsB[i].endpoint[3] - Image.y0B),
	    scale,
	  }
		-- Put into the local and global frames
    vendpoint[1] = Image.tfL * (v01 / v01[4])
		--vendpoint[1] = Image.tfG * (v0 / v0[4])

		local v02 = vector.new{
	    Image.focalB,
			-(lines.propsB[i].endpoint[2] - Image.x0B),
	    -(lines.propsB[i].endpoint[4] - Image.y0B),
	    scale,
	  }
		-- Put into the local and global frames
    vendpoint[2] = Image.tfL * (v02 / v02[4])
		--vendpoint[2] = Image.tfG * (v02 / v02[4])

    vHeight = (vendpoint[1][3]+vendpoint[2][3]) / 2

		-- TODO: Place in the config
    local vHeightMax = 0.50

		if length<config.min_length then
			passed = false
			msgs[i] = string.format('min_length: %.2f<%.2f', length, config.min_length)
		end
		if linecount>num_line then
			passed = false
			msgs[i] = string.format('linecount: %.2f>%.2f', linecount, num_line)
		end
		if vHeight>vHeightMax then
			passed = false
			msgs[i] = string.format('vHeight: %.2f>%.2f', vHeight, vHeightMax)
		end

    if passed then
			lines.detect = 1
      linecount = linecount + 1
      lines.length[linecount] = length
      lines.endpoint[linecount] = Image.scaleB * vector.new(lines.propsB[i].endpoint)

			-- Project to the ground
			local target_height = 0
			if pHead4[3]==target_height then
				vendpoint[1] = vector.copy(vendpoint[1])
			else
				local scale = (pHead4[3] - target_height) / (pHead4[3] - vendpoint[1][3])
				vendpoint[1] = pHead4 + scale * (vendpoint[1] - pHead4)
			end
			if pHead4[3]==target_height then
				vendpoint[2] = vector.copy(vendpoint[2])
			else
				local scale = (pHead4[3] - target_height) / (pHead4[3] - vendpoint[2][3])
				vendpoint[2] = pHead4 + scale * (vendpoint[2] - pHead4)
			end

      lines.v[linecount] = { unpack(vendpoint, 1, 2) }
      lines.angle[linecount] = math.abs(
				math.atan2(vendpoint[1][2]-vendpoint[2][2], vendpoint[1][1]-vendpoint[2][1])
			)
			msgs[i] = 'Passed checks'
    end
  end -- end for

  lines.nLines = linecount

  return lines, table.concat(msgs, '\n')
end
function detectLine.exit()
end
return detectLine
