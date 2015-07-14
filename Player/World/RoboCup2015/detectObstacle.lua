local detectObstacle = {}
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
function detectObstacle.entry(cfg, Image)
  config = cfg
	label_flag = config.label
	grid_x = config.grid_x
	grid_y = config.grid_y
	th_min_area = config.th_min_area
	min_black_fill_rate = config.min_black_fill_rate
	th_aspect_ratio = config.th_aspect_ratio
	th_max_height = config.th_max_height
	th_min_height = config.th_min_height
	th_min_orientation = config.th_min_orientation
	min_ground_fill_rate = config.min_ground_fill_rate
  colors = Image.colors
end
function detectObstacle.update(Image)
  if type(Image)~='table' then
    return false, 'Bad Image'
  end
	-- Obstacle table
  local obstacle, obs_count, obs_debug = {}, 0, ''
  obstacle.iv, obstacle.v, obstacle.detect = {}, {}, 0
  obstacle.axisMinor, obstacle.axisMajor, obstacle.orientation = {}, {}, {}
  obstacle.dist = {}

  -- Update horizon
  local pa = Image.qHead[2]   -- + Config.walk.bodyTilt
  local horizonB = (Image.hb/2.0) - Image.focalB*math.tan(pa - 10*DEG_TO_RAD)
  horizonB = math.min(Image.hb, math.max(math.floor(horizonB), 0))
  --TODO: plot it in monitor
 --print("HORIZON: ", horizonB, 'hb:', Image.hb)

	-- Done in b...
	local obsProps = ImageProc.obstacles(
		tonumber(ffi.cast('intptr_t', ffi.cast('void *', Image.labelB_d))),
		Image.wb, Image.hb,
		config.min_width, config.max_width, horizonB
	)

  if #obsProps == 0 then
		return false, 'None'
	end

	-- Position of the head now
	local pHead4 = T.position4(Image.tfL)
	--local pHead4 = T.position4(Image.tfG)

  --for i=1,math.min(30, #obsProps) do
	local msgs = {}
  for i=1, #obsProps do
    local passed = true
		local v
		local obstacle_dist
		msgs[i] = 'unknown'
		-- Form the bounding boxes to inspect
		local black_bboxB = {
			obsProps[i].position[1] - obsProps[i].width/2,
			obsProps[i].position[1] + obsProps[i].width/2,
			obsProps[i].position[2] - 2*obsProps[i].width,
			obsProps[i].position[2],
		}
		local blackStatsB = ImageProc2.color_stats_exact(
			Image.labelB_d, Image.wb, Image.hb, colors.black, black_bboxB
		)
		--util.ptable(blackStatsB)

		-- TODO: Use labelA?
		local black_bboxA = bboxB2A(black_bboxB, Image.scaleB)
		local blackStatsA = ImageProc2.color_stats_exact(
			Image.labelA_d, Image.wa, Image.ha, colors.black, black_bboxA
		)
		--util.ptable(blackStatsA)

		-- Black area check
		if blackStatsB.area < 1 then
			passed = false
			msgs[i] = '0 B area'
		end

		-- Black Fill rate check
		if passed then
			local blackA_fill_rate
			local blackB_fill_rate
			-- Find the area
			if blackStatsA then
				local box_areaA = (black_bboxA[2] - black_bboxA[1] + 1) *
					(black_bboxA[4] - black_bboxA[3] + 1)
				blackA_fill_rate = blackStatsA.area / box_areaA
			end
			if blackStatsB then
				local box_areaB = (black_bboxB[2] - black_bboxB[1] + 1) *
					(black_bboxB[4] - black_bboxB[3] + 1)
				blackB_fill_rate = blackStatsB.area / box_areaB
			end

			if blackA_fill_rate < min_black_fill_rate then
				passed = false
				msgs[i] = string.format(
					"black_fill_rate: A: %.2f<%.2f B: %.2f<%.2f",
					blackA_fill_rate, min_black_fill_rate, blackB_fill_rate, min_black_fill_rate
				)
			end
		end
    -- Convert to local frame
		if passed then
			-- TODO: place the diameter in entry
			-- This is in labelB coordinates
    	local scale = math.max(1, obsProps[i].width / Config.world.obsDiameter)
    	-- Instead of the width-based distance
    	-- Let's just project the bottom position to the ground
			local centroid = obsProps[i].position
			-- TODO: Why this scale?
			local scale = 0.1
			local v0 = vector.new{
		    Image.focalB,
		    -(centroid[1] - Image.x0B),
		    -(centroid[2] - Image.y0B),
		    scale,
		  }
			-- Put into the local and global frames
      local vL = Image.tfL * (v0 / v0[4])
			local vG = Image.tfG * (v0 / v0[4])
			v = vL

			-- Project to the ground
			local target_height = 0
			if pHead4[3]==target_height then
				obsProps[i].v = vector.copy(v)
			else
				local scale = (pHead4[3] - target_height) / (pHead4[3] - v[3])
				obsProps[i].v = pHead4 + scale * (v - pHead4)
			end

    	obstacle_dist = math.sqrt(obsProps[i].v[1]^2+obsProps[i].v[2]^2)
		end


    -- TODO: Field bounds check
		--[[
    if passed then
      local global_v = util.pose_global({v[1], v[2], 0}, wcm.get_robot_pose())
      -- TODO: for now ignore opponent
      if math.abs(global_v[1])>xMax-0.3 or math.abs(global_v[2])>yMax then
        passed = false
        msgs[i] = 'OUTSIDE FIELD!'
      end
    end
		--]]

    -- TODO: Distance check
    if passed and obstacle_dist > 7 then
      passed = false
      msgs[i] = string.format('TOO FAR:%.2f >%.2f\n', obstacle_dist, 7)
    end

    -- Ground check
    if passed and Image.hb-obsProps[i].position[2]>10 then

			-- Bounding box in labelB below the ball
			-- {left_x, right_x, top_y, bot_y}
			local fieldBBox = {
				obsProps[i].position[1] - obsProps[i].width/2,
				obsProps[i].position[1] + obsProps[i].width/2,
				obsProps[i].position[2],
				math.min(Image.hb, obsProps[i].position[2]+20),
			}
			-- Area of the labelB bbox
			local fieldBBoxArea = (fieldBBox[2] - fieldBBox[1] + 1)
				* (fieldBBox[4] - fieldBBox[3] + 1)
			-- Color stats for the bbox of the field
			local fieldBBoxStats = ImageProc2.color_stats(
				Image.labelB_d, Image.wb, Image.hb, colors.field, fieldBBox
			)
			if fieldBBoxStats.area/fieldBBoxArea < min_ground_fill_rate then
        passed = false
        msgs[i] = string.format(
					'Ground Check: %.2f < %.2f',
					fieldBBoxStats.area/fieldBBoxArea, min_ground_fill_rate
				)
      end
    end

    if passed then
      obs_count = obs_count + 1
      table.insert(obstacle.dist, obstacle_dist)
      obstacle.iv[obstacle_dist] = Image.scaleB * vector.new(obsProps[i].position)
      obstacle.axisMinor[obstacle_dist] = obsProps[i].width
      obstacle.axisMajor[obstacle_dist] = obsProps[i].width

      obstacle.v[obstacle_dist] = obsProps[i].v
      obstacle.detect = 1
      obstacle.count = obs_count
			msgs[i] = 'Found!'
    end

  end -- end loop

	-- Detected one or more obstacles
  if obstacle.detect == 1 then
    -- Sort to get the closest two
    local obsStats = {}
    obsStats.iv, obsStats.xs, obsStats.ys = {},{},{}
    obsStats.axisMinor, obsStats.axisMajor, obsStats.orientation = {}, {}, {}
    table.sort(obstacle.dist)
    for i=1, math.min(5, obstacle.count) do
      obsStats.iv[i] = obstacle.iv[ obstacle.dist[i] ]
	    local pos = vector.new(obstacle.v[ obstacle.dist[i] ])  -- LOCAL
      obsStats.xs[i] = pos[1]
      obsStats.ys[i] = pos[2]

      obsStats.axisMinor[i] = obstacle.axisMinor[ obstacle.dist[i] ]
      obsStats.axisMajor[i] = obstacle.axisMajor[ obstacle.dist[i] ]
      obsStats.orientation[i] = math.pi/2
    end
    return obsStats, table.concat(msgs, '\n')
  end

	return obsStats, table.concat(msgs, '\n')
end
function detectObstacle.exit()
end
return detectObstacle
