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

	-- Done in b...
	-- TODO: No tensor this time!
	local obsProps = ImageProc.obstacles(
		tonumber(ffi.cast('intptr_t', ffi.cast('void *', Image.labelB_d))),
		config.min_width, config.max_width, horizonB
	)

  -- print("HORIZON: ", horizonB, 'hb:', hb)

  if #obsProps == 0 then
		return false, 'None'
	end

  --for i=1,math.min(30, #obsProps) do
	local msgs = {}
  for i=1, #obsProps do
    local passed = true
		local v
		local obstacle_dist
		-- Black check
		local black_box = {
			obsProps[i].position[1] - obsProps[i].width/2,
			obsProps[i].position[1] + obsProps[i].width/2,
			obsProps[i].position[2] - 2*obsProps[i].width,
			obsProps[i].position[2],
		}
		-- Make we know the difference between b and a....
		local blackStats = ImageProc2.color_stats(
			Image.labelB_d, Image.wb, Image.hb, colors.black, black_box
		)
		local bboxA = bboxB2A(black_box, Image.scaleB)
		local box_area = (bboxA[2] - bboxA[1] + 1) * (bboxA[4] - bboxA[3] + 1)

    local black_fill_rate = blackStats.area / box_area
		if black_fill_rate < min_black_fill_rate then
			passed = false
			msgs[i] = string.format(
				"black fillrate: %.2f<%.2f", black_fill_rate, min_black_fill_rate)
		end

    -- Convert to local frame
		if passed then
    	local scale = math.max(1, obsProps[i].width / Config.world.obsDiameter)
    --Instead of the width-based distance
    --Let's just project the bottom position to the ground
      v = check_coordinateB(
        { obsProps[i].position[1],  obsProps[i].position[2]}, 0.1)
      v = projectGround(v,0)
    	obstacle_dist = math.sqrt(v[1]*v[1]+v[2]*v[2])
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
    if passed and hb-obsProps[i].position[2]>10 then
      local left_x = obsProps[i].position[1] - obsProps[i].width/2
      local right_x = obsProps[i].position[1] + obsProps[i].width/2
      local top_y = obsProps[i].position[2]
      local bot_y = math.min(hb, obsProps[i].position[2]+20)

      local ground_bbox = {left_x, right_x, top_y, bot_y}
      local groundStats, bbox_area = bboxStats('b', colors.field, ground_bbox)
      if groundStats.area/bbox_area < min_ground_fill_rate then  --TODO
        passed = false
        msgs[i] = string.format(
					'GROUND CHECK FAIL: %.2f < %.2f\n',
					groundStats.area/bbox_area, min_ground_fill_rate
				)
      end
    end

    if passed then
      obs_count = obs_count + 1
      table.insert(obstacle.dist, obstacle_dist)
      obstacle.iv[obstacle_dist] = vector.new(obsProps[i].position)*scaleB
      obstacle.axisMinor[obstacle_dist] = obsProps[i].width
      obstacle.axisMajor[obstacle_dist] = obsProps[i].width

      obstacle.v[obstacle_dist] = v
      obstacle.detect = 1
      obstacle.count = obs_count
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
