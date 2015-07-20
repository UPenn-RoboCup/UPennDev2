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

-- TODO: Needs to know the ball centroid, so as to avoid categorizing that one
function detectLine.update(Image)
  if type(Image)~='table' then
    return false, 'Bad Image'
  end

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

  if type(linePropsB)~='table' or #linePropsB==0 then
    return false, 'None'
  end

	-- Position of the head now
	local pHead4 = T.position4(Image.tfL)

	local msgs = {}
	local lines = {}
  for i=1, math.min(4, #linePropsB) do

		local passed = true
		local propsB = linePropsB[i]
		util.ptable(propsB)

		local length
		if passed then
			length = math.sqrt(
	    	(propsB.iMax - propsB.iMin)^2 +
	    	(propsB.jMax - propsB.jMin)^2
			)
			local minLen = IS_WEBOTS and 38 or 50
			if length<minLen then
				passed = false
				msgs[i] = string.format('min_length: %.2f < %.2f', length, minLen)
			end
		end

		local vL_endpoint, vG_endpoint
		if passed then
			-- Scale: nPixels / actual length
			-- 20 pixels / m; 1px for a 5cm wide line
			local scale = 20
			local vL1 = Image.tfL * {
		    Image.focalB / scale,
		    -(propsB.iMin - Image.x0B) / scale,
		    -(propsB.jMin - Image.y0B) / scale,
		    1,
		  }
			local vL2 = Image.tfL * {
		    Image.focalB / scale,
				-(propsB.iMax - Image.x0B) / scale,
		    -(propsB.jMax - Image.y0B) / scale,
		    1,
		  }
			local vL3 = Image.tfL * {
		    Image.focalB / scale,
				-(propsB.iMean - Image.x0B) / scale,
		    -(propsB.jMean - Image.y0B) / scale,
		    1,
		  }

			-- Project to the ground
			local zHead = pHead4[3]
			vL_endpoint = {
				pHead4 + (vL1 - pHead4) * zHead / (zHead - vL1[3]),
				pHead4 + (vL2 - pHead4) * zHead / (zHead - vL2[3])
			}

			-- TODO: Reject the center circle
			local p6 = T.position6D(Image.tfG)
			local pose = vector.pose{p6[1], p6[2], p6[6]}
			local ca = math.cos(pose.a)
	    local sa = math.sin(pose.a)
			vG_endpoint = {}
			vG_endpoint[1] = vector.new{
				vL_endpoint[1][1]*ca - vL_endpoint[1][2]*sa + pose.x,
				vL_endpoint[1][1]*sa + vL_endpoint[1][2]*ca + pose.y
			}
			vG_endpoint[2] = vector.new{
				vL_endpoint[2][1]*ca - vL_endpoint[2][2]*sa + pose.x,
				vL_endpoint[2][1]*sa + vL_endpoint[2][2]*ca + pose.y
			}

			-- Filter out the center circle
			local d1 = math.sqrt(vG_endpoint[1][1]^2+vG_endpoint[1][2]^2)
			local d2 = math.sqrt(vG_endpoint[2][1]^2+vG_endpoint[2][2]^2)
			if d1<1.5 and d2<1.5 then
				passed = false
				msgs[i] = string.format('Center circle: %.2f, %.2f', d1, d2)
			end

			-- TODO: filter out endpoints that are out of bounds
			if math.abs(vG_endpoint[1][1]) > 7 or math.abs(vG_endpoint[2][1]) > 7
			then
				--[[
				print('X vG_endpoint', unpack(vG_endpoint))
				print('X vL_endpoint', unpack(vL_endpoint))
				--]]
				passed = false
				msgs[i] = string.format('Outside field X!')
			end
			if math.abs(vG_endpoint[1][2]) > 7 or math.abs(vG_endpoint[2][2]) > 7
			then
				--[[
				print('Y vG_endpoint', unpack(vG_endpoint))
				print('Y vL_endpoint', unpack(vL_endpoint))
				--]]
				passed = false
				msgs[i] = string.format('Outside field Y!')
			end

			-- TODO: filter out the small penalty box line
		end

    if passed then

			local angleL = math.abs(math.atan2(
				vL_endpoint[1][2]-vL_endpoint[2][2],
				vL_endpoint[1][1]-vL_endpoint[2][1]
			))

			-- TODO: Round to 0 or 90 to length/width line categories
			local angleG = math.abs(math.atan2(
				vG_endpoint[1][2]-vG_endpoint[2][2],
				vG_endpoint[1][1]-vG_endpoint[2][1]
			))

			local epB = vector.new{
				propsB.iMin, propsB.iMax, propsB.jMin, propsB.jMax
			}

			table.insert(lines, {
				len = length,
				epA = Image.scaleB * epB,
				vL = vL_endpoint,
				vG = vG_endpoint,
				aL = angleL,
				aG = angleG
			})

			msgs[i] = 'Passed checks'
    end

  end -- end for

  return lines, table.concat(msgs, '\n')
end
function detectLine.exit()
end
return detectLine
