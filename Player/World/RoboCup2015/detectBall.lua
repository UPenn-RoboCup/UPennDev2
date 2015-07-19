local detectBall = {}
local ok, ffi = pcall(require, 'ffi')
local ImageProc = require'ImageProc'
local ImageProc2 = require'ImageProc.ffi2'
local T = require'Transform'
local vector = require'vector'
local util = require'util'
local ENABLE_COORDINATE_CHECK = true
local ENABLE_FIELD_CHECK = true
local ENABLE_GROUND_CHECK = true
local function bboxB2A(bboxB, scaleB)
	return {
		scaleB * bboxB[1],
		scaleB * bboxB[2] + scaleB - 1,
		scaleB * bboxB[3],
		scaleB * bboxB[4] + scaleB - 1,
	}
end

local lastBallG = nil
local lastBallGt = -math.huge

local b_ground_head_pitch, b_ground_boundingbox, b_ground_white, b_ground_green
local colors
local ball_color
local config
function detectBall.entry(cfg, Image)
	config = cfg
	b_ground_head_pitch = config.th_ground_head_pitch
	b_ground_boundingbox = config.th_ground_boundingbox
	b_ground_white = config.th_ground_white
	b_ground_green = config.th_ground_green
	colors = Image.colors
	ball_color = colors.orange
end

local function find_ball_off_line(Image)
	-- Connect the regions in labelB
	local ballPropsB = ImageProc.connected_regions(
	tonumber(ffi.cast('intptr_t', ffi.cast('void *', Image.labelB_d))),
	Image.wb,
	Image.hb,
	ball_color
	)

	if not ballPropsB then
		return false, 'No connected regions'
	end
	local nProps = #ballPropsB
	if nProps==0 then
		return false, '0 connected regions'
	end

	-- Position of the head now
	local pHead4 = T.position4(Image.tfL)
	--local pHead4 = T.position4(Image.tfG)

	-- Run the checks
	local msgs = {}
	local nCheck = math.min(8, nProps)
	for i=1, nCheck do
		local passed = true
		-- Check the image properties
		-- TODO: Verify the bounding box!
		local propsB = ballPropsB[i]
		local bboxA = bboxB2A(propsB.boundingBox, Image.scaleB)
		local bboxArea = (bboxA[2] - bboxA[1] + 1) * (bboxA[4] - bboxA[3] + 1)
		if bboxArea < config.th_min_bbox_area then
			passed = false
			msgs[i] = string.format('Box area: %d<%d', bboxArea, config.th_min_bbox_area)
		end

		-- labelA area check
		local propsA
		if passed then
			propsA = ImageProc2.color_stats(
			Image.labelA_d, Image.wa, Image.ha, ball_color,
			bboxA
			)
			if propsA.area < config.th_min_area then
				passed = false
				msgs[i] = string.format('Area: %d < %d', propsA.area, config.th_min_area)
			end
		end

		-- Ball width/height
		if passed then
			local axisRatio = propsA.axisMajor / propsA.axisMinor
			local axisCheck = 1.8
			if axisRatio > axisCheck or axisRatio < 1/axisCheck then
				passed = false
				msgs[i] = string.format('axisRatio: %.2f < %.2f < %.2f',
				1/axisCheck, axisRatio, axisCheck)
			end
		end

		-- Fill rate check
		if passed then
			local fill_rate = propsA.area / (propsA.axisMajor * propsA.axisMinor)
			if fill_rate < config.th_min_fill_rate then
				passed = false
				msgs[i] = string.format('Fill rate: %.2f < %.2f', fill_rate, config.th_min_fill_rate)
			end
		end

		-- Check the coordinate on the field
		local v = vector.new{0,0,0,1}
		local vL, vG
		local dArea = passed and math.sqrt((4/math.pi) * propsA.area)
		if passed and ENABLE_COORDINATE_CHECK then
			local scale = math.max(dArea, propsA.axisMajor) / config.diameter
			local v0 = vector.new{
				Image.focalA,
				-(propsA.centroid[1] - Image.x0A),
				-(propsA.centroid[2] - Image.y0A),
				scale
			}
			-- Put into the local and global frames
			vL = Image.tfL * (v0 / v0[4])
			vG = Image.tfG * (v0 / v0[4])
			-- Save the position
			v = vL
			--[[
			print('v0', v0)
			print('vG', vG)
			print('vL', vL)
			--]]

			-- Check the distance
			local dist = math.sqrt(v[1]^2 + v[2]^2)
			--require'util'.ptable(config)
			local minZ = -0.1
			local maxZ = 0.5
			--local minX = 0.02
			--local minY = 0.02
			--local maxH = (config.max_height0 and config.max_height1) and (config.max_height0 + dist * config.max_height1)
			if dist > config.max_distance then
				passed = false
				msgs[i] = string.format("Distance: %.2f > %.2f", dist, config.max_distance)
				--elseif maxH and v[3] > maxH then
			elseif v[3] > maxZ then
				passed = false
				msgs[i] = string.format("maxZ: %.2f (%.2f, %.2f, %.2f)", maxZ, unpack(v,1,3))
			elseif minZ and v[3] < minZ then
				passed = false
				msgs[i] = string.format("minZ: %.2f (%.2f, %.2f, %.2f)", minZ, unpack(v,1,3))
			elseif minX and v[1] < minX then
				passed = false
				msgs[i] = string.format("minZ: %.2f (%.2f, %.2f, %.2f)", minZ, unpack(v,1,3))
			end
		end

		-- TODO: Field bounds check
		-- Field width is 6
		----[[
		if passed and ENABLE_FIELD_CHECK then
			--print('vG', vG)
			-- only when the ball is far from us
			local distL = math.sqrt(vL[1]^2+vL[2]^2)
			--if distL>3 then
				if math.abs(vG[1])>3.5 or math.abs(vG[2])>3.5 then
					passed = false
					msgs[i] = string.format("Outside field (%.2f, %.2f, %.2f)", unpack(v,1,3))
				end
			--end
		end
		--]]

		-- Ground Check
		--			print(string.format(
		--'ball height:%.2f, thr: %.2f'
		-- v[3], config.max_height0+config.max_height1*math.sqrt(v[1]*v[1]+v[2]*v[2]
		--)))
		if passed and ENABLE_GROUND_CHECK then
			-- TODO: Only when looking down
			--if Image.qHead[2] < b_ground_head_pitch then
				local th_ground_boundingbox = b_ground_boundingbox
				local ballCentroid = propsA.centroid
				local vmargin = Image.ha - ballCentroid[2]
				-- When robot looks down, it may fail to pass the green check
				-- So increase the bottom margin threshold
				if vmargin > dArea * 2.0 then
					-- Bounding box in labelA below the ball
					local fieldBBox = {
						ballCentroid[1] + th_ground_boundingbox[1],
						ballCentroid[1] + th_ground_boundingbox[2],
						ballCentroid[2] + dArea/2 + th_ground_boundingbox[3],
						ballCentroid[2] + dArea/2 + th_ground_boundingbox[4],
					}
					-- color stats for the bbox of the field
					local fieldBBoxStats = ImageProc2.color_stats(
					Image.labelA_d, Image.wa, Image.ha, colors.field, fieldBBox
					)
					print('fieldBBoxStats.area', fieldBBoxStats.area)
					if fieldBBoxStats.area < b_ground_green then
						-- if there is no field under the ball
						-- it may be because its on a white line
						local whiteBBoxStats = ImageProc2.color_stats(
						Image.labelA_d, Image.wa, Image.ha, colors.white, fieldBBox
						)
						print('whiteBBoxStats.area', whiteBBoxStats.area)
						if whiteBBoxStats.area < b_ground_white then
							passed = false
							msgs[i] = "Green check fail"
						end
					end -- end green
				end -- end margin
			--end -- end qHead check
		end

		local ignoringBack = Image.HeadFSM=="headBackScan" or
		Image.HeadFSM=="headBackScanInit" or
		Image.HeadFSM=="headLog" or
		Image.HeadFSM=="headObstacleScan" or wcm.get_ball_backonly()==1
		if Image.HeadFSM=="headLookGoal" then
			passed = false
			msgs[i] = string.format("Looking for goal")
		end

		local projectedVL, projectedVG
		if passed then
			local target_height = config.diameter / 2
			local scale = (pHead4[3] - target_height) / (pHead4[3] - vL[3])
			projectedVL = pHead4 + scale * (vL - pHead4)

			local scale = (pHead4[3] - target_height) / (pHead4[3] - vG[3])
			projectedVG = pHead4 + scale * (vG - pHead4)

			local minBackX = -0.1
			if Config.reject_forward_balls and ignoringBack then
				if projectedVG[1] > minBackX then
					passed = false
					msgs[i] = string.format("Ball forward: %.2f", projectedVG[1])
				end
			end

		end

		if passed and lastBallG then
			local distObs = math.sqrt(
				(projectedVG[1] - lastBallG[1]) ^ 2,
				(projectedVG[2] - lastBallG[2]) ^ 2
			)
			--print('lastBallG', lastBallG, 'ballG', projectedVG, 'distObs', distObs)
			local dtBall = Image.t - lastBallGt
			if (distObs > 2) and (dtBall < 10) then
				passed = false
				msgs[i] = string.format("Big delta: %.2f", distObs)
				--print('dist big', msgs[i])
				--print('passed1', passed)
			end
		end

		-- If passed the checks
		--print('passed', passed)
		if passed==true then
			propsA.v = projectedVL
			propsA.t = Image.t
			-- For ballFilter
			propsA.r = math.sqrt(v[1]^2+v[2]^2)
			propsA.dr = 0.25 * propsA.r --TODO: tweak
			propsA.da = 10 * DEG_TO_RAD
			msgs[i] = string.format('Ball detected @ %.2f, %.2f, %.2f', unpack(projectedVL))
			--print('detectBall |', msgs[i])
			propsA.online = false

			lastBallG = projectedVG
			lastBallGt = Image.t

			return propsA, table.concat(msgs, '\n')
		end

	end  -- end of loop

	-- Assume failure
	return false, table.concat(msgs, '\n')
end

function detectBall.update(Image)
	if type(Image)~='table' then
		return false, 'Bad Image'
	end
	--local ball_color = colors.orange
	local cc = Image.ccA_d[ball_color]
	if cc<6 then return false, 'Color count' end
	--
	local propsA, msgOff = find_ball_off_line(Image)
	if propsA then return propsA, msgOff end
	--print('Check on line...')
	--local propsA, msgOn = find_ball_on_line(Image)
	--if propsA then return propsA, msgOn end
	--print('not online either...')
	return false, msgOff..'\n=On=\n'..(msgOn or '')
end
function detectBall.exit()
end
return detectBall
