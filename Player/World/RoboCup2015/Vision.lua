-- Vision
-- (c) 2014 Stephen McGill
-- General Detection methods
local Vision = {}
-- Detection and HeadTransform information
local ImageProc = require'ImageProc'
local ImageProc2 = require'ImageProc.ffi2'
local ok, ffi = pcall(require, 'ffi')
local HeadImage
local T = require'Transform'
local HT = require'libHeadTransform'
local mp = require'msgpack.MessagePack'
local vector = require'vector'
local util = require'util'
local c_zlib = require'zlib.ffi'.compress
local si = require'simple_ipc'
require'wcm'
require'hcm'
require'gcm'
local ptable = require'util'.ptable

-- Important local variables
local colors
-- Detection thresholds
local b_diameter, b_dist, b_height0, b_height1, b_fill_rate, b_bbox_area, b_area
local b_ground_head_pitch, b_ground_boundingbox, b_ground_white, b_ground_green
local th_nPostB
local g_area, g_bbox_area, g_fill_rate, g_orientation, g_aspect_ratio, g_margin

-- Store information about what was detected
local detected = {
  id = 'detect',
}
-- World config
local postDiameter = Config.vision.goal.postDiameter
local postHeight = Config.vision.goal.goalHeight
local goalWidth = Config.vision.goal.goalWidth

-- Debug printing in terminal
local DEBUG = Config.debug.obstacle

local function bboxB2A(bboxB, scaleB)
	return {
		scaleB * bboxB[1],
		scaleB * bboxB[2] + scaleB - 1,
		scaleB * bboxB[3],
		scaleB * bboxB[4] + scaleB - 1,
	}
end

-- Simple bbox with no tilted color stats
-- Assume always input bboxB
local function bboxStats(label, color, bbox)
	local stats
	if label=='a' then
		stats = ImageProc2.color_stats(label, color, bbox)
	else
		bbox = bboxB2A(bbox)
		stats = ImageProc2.color_stats(label, color, bbox)
	end
	-- return stats and area
  return stats, (bbox[2] - bbox[1] + 1) * (bbox[4] - bbox[3] + 1)
end

local function check_prop(color, prop, th_bbox_area, th_area, th_fill, labelA_d)
  -- Grab the statistics in labelA
  local stats, box_area = bboxStats('a', color, prop.boundingBox)
  --TODO: bbox area check seems redundant
  if box_area < th_bbox_area then
    return string.format('Box area: %d<%d\n',box_area,th_bbox_area)
  end
  local area = stats.area
  -- If no pixels then return
  if area < th_area then
    return string.format('Area: %d < %d \n', area, th_area)
  end
  -- Get the fill rate
	-- TODO: depends on ball or goal
  --local fill_rate = area / box_area
	local fill_rate = area / (stats.axisMajor * stats.axisMinor)
  if fill_rate < th_fill then
    return string.format('Fill rate: %.2f < %.2f\n', fill_rate, th_fill)
  end
  return stats
end

-- Yield coordinates in the labelA space
-- Returns an error message if max limits are given
local function check_coordinateA(centroid, scale, maxD, maxH1, maxH2)
	local v = HT.project({
    focalA,
    -(centroid[1] - x0A),
    -(centroid[2] - y0A),
    scale,
  })

	local dist_sq = v[1]^2 + v[2]^2
  local maxH = maxH1 and maxH2 and maxH1 + math.sqrt(dist_sq) * maxH2

  -- Check the distance
  if maxD and dist_sq > maxD^2 then
    return string.format("Distance: %.2f > %.2f", math.sqrt(dist_sq), maxD)
  elseif maxH and v[3] > maxH then
    return string.format("Height: %.2f > %.2f", v[3], maxH)
  end
  return v
end

-- Yield coordinates in the labelB space
-- Returns an error message if max limits are given
local function check_coordinateB(centroid, scale, maxD, maxH)

	local v = HT.project({
    focalB,
    -(centroid[1] - x0B),
    -(centroid[2] - y0B),
    scale,
  })

  -- Check the distance
	local dist_sq = v[1]^2 + v[2]^2
  if maxD and dist_sq > maxD^2 then
    return string.format("Distance: %.2f > %.2f", math.sqrt(dist_sq), maxD)
  elseif maxH and v[3] > maxH then
    return string.format("Height: %.2f > %.2f", v[3], maxH)
  end
  return v
end

local function find_ball(Image)
  if type(Image)~='table' then
    return false, 'Bad Image'
  end
  --print('\n=========\n')
  --ptable(Image)

  local cc = Image.ccA_d[colors.orange]
  if cc<6 then
    return false, 'Color count'
  end
  -- Connect the regions in labelB
  local ballPropsB = ImageProc.connected_regions(
    tonumber(ffi.cast('intptr_t', ffi.cast('void *', Image.labelB_d))),
    Image.wb,
    Image.hb,
    colors.orange
  )

  if not ballPropsB then
    return false, 'No connected regions'
  end
  local nProps = #ballPropsB
  if nProps==0 then
    return false, '0 connected regions'
  end

  -- Position of the head now
  local pHead4 = T.position4(Image.tfG)

  -- Run the checks
  local msgs = {}
  local nCheck = math.min(5, nProps)
  for i=1, nCheck do
    local passed = true
    -- Check the image properties
    -- TODO: Verify the bounding box!
    local propsB = ballPropsB[i]
    local bboxA = bboxB2A(propsB.boundingBox, Image.scaleB)
    local bboxAarea = (bboxA[2] - bboxA[1] + 1) * (bboxA[4] - bboxA[3] + 1)
    if bboxAarea < b_bbox_area then
      passed = false
      msgs[i] = string.format('Box area: %d<%d\n', bboxAarea, b_bbox_area)
    end

    -- labelA area check
    local propsA
    if passed then
      propsA = ImageProc2.color_stats(
        Image.labelA_d, Image.wa, Image.ha, colors.orange,
        bboxA
      )
      if propsA.area < b_area then
        passed = false
        msgs[i] = string.format('Area: %d < %d \n', propsA.area, b_area)
      end
    end

    -- Fill rate check
    if passed then
      local fill_rate = propsA.area / (propsA.axisMajor * propsA.axisMinor)
      if fill_rate < b_fill_rate then
        passed = false
        msgs[i] = string.format('Fill rate: %.2f < %.2f\n', fill_rate, b_fill_rate)
      end
    end

    -- Check the coordinate on the field
    local v, dArea
    if passed then
      dArea = math.sqrt((4/math.pi) * propsA.area)
      local scale = math.max(dArea/b_diameter, propsA.axisMajor/b_diameter);
      local v0 = vector.new{
        Image.focalA,
        -(propsA.centroid[1] - Image.x0A),
        -(propsA.centroid[2] - Image.y0A),
        scale,
      }
      -- Put into the local frame
      --v = Image.tfL * (v0 / v0[4])
      -- Put into the global frame
      v = Image.tfG * (v0 / v0[4])

      -- Check the distance
      local dist_sq = v[1]^2 + v[2]^2
      local maxH = (b_height0 and b_height1) and
        (b_height0 + math.sqrt(dist_sq) * b_height1)
      if dist_sq > b_dist^2 then
        passed = false
        msgs[i] = string.format("Distance: %.2f > %.2f", math.sqrt(dist_sq), maxD)
      elseif maxH and v[3] > maxH then
        passed = false
        msgs[i] = string.format("Height: %.2f > %.2f", v[3], maxH)
      end
    end

    -- TODO: Field bounds check
    --[[
    if passed then
      if math.sqrt(v[1]*v[1]+v[2]*v[2])>3 then
        local margin = 0.85 --TODO
        local global_v = util.pose_global({v[1], v[2], 0}, wcm.get_robot_pose())
        if math.abs(global_v[1])>xMax+margin or math.abs(global_v[2])>yMax+margin then
          check_fail = true
          debug_ball('OUTSIDE FIELD!\n')
        end
      end
    end
    --]]

    -- Ground Check
    --			print(string.format(
    --'ball height:%.2f, thr: %.2f'
    -- v[3], b_height0+b_height1*math.sqrt(v[1]*v[1]+v[2]*v[2]
    --)))
    if passed then
      -- Only when looking down
      if Image.qHead[2] < b_ground_head_pitch then
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
          if fieldBBoxStats.area < b_ground_green then
            -- if there is no field under the ball
            -- it may be because its on a white line
            local whiteBBoxStats = ImageProc2.color_stats(
              Image.labelA_d, Image.wa, Image.ha, colors.white, fieldBBox
            )
            if whiteBBoxStats.area < b_ground_white then
              passed = false
              msgs[i] = "Green check fail"
            end
          end -- end green
        end -- end margin
      end -- end qHead check
    end


    -- If passed the checks
    -- Project the ball to the ground
    if passed then
      local target_height = b_diameter / 2
      if pHead4[3]==target_height then
        propsA.v = vector.copy(v)
      else
        local scale = (pHead4[3] - target_height) / (pHead4[3] - v[3])
        propsA.v = pHead4 + scale * (v - pHead4)
      end

      propsA.t = Image.t
			-- For ballFilter
		  propsA.r = math.sqrt(v[1]^2+v[2]^2)
			propsA.dr = 0.25 * propsA.r --TODO: tweak
			propsA.da = 10 * DEG_TO_RAD

      msgs[i] = string.format('Ball detected @ %.2f, %.2f, %.2f', unpack(propsA.v,1,3))
      return propsA, msgs[i]
    end
  end  -- end of loop

  -- Assume failure
  return false, table.concat(msgs, '\n')
end

-- TODO: Allow the loop to run many times
--[[
local function find_goal()
  -- Form the initial goal check
  local postB = ImageProc.goal_posts(
		tonumber(ffi.cast('intptr_t', ffi.cast('void *', HeadImage.labelB_d))),
		wb,
		hb,
		colors.magenta)
  if not postB then return'None detected' end
  -- Now process each goal post
  -- Store failures in the array
  local failures, successes = {}, {}
	local nPosts, i_validB, valid_posts = 0, {}, {}
	--for i=1, math.min(#postB, th_nPostB) do
	for i=1, #postB do
		local post = postB[i]
    local fail, has_stats = {}, true
    local postStats = check_prop(colors.magenta, post, g_bbox_area, g_area, g_fill_rate, labelA_t)
    if type(postStats)=='string' then
      table.insert(fail, postStats)
    else
			table.insert(fail, string.format('\n Post # %d ', i))
			local check_passed = true
      -- TODO: Add lower goal post bbox check
      -- Orientation check
      if check_passed and math.abs(postStats.orientation) < g_orientation then
        table.insert(fail,
					string.format('Orientation:%.1f < %.1f \n', postStats.orientation, g_orientation) )
				check_passed = false
      end
      -- Aspect Ratio check
			if check_passed then
				local aspect = postStats.axisMajor / postStats.axisMinor;
				if (aspect < g_aspect_ratio[1]) or (aspect > g_aspect_ratio[2]) then
					table.insert(fail,
						string.format('Aspect ratio:%.2f, [%.2f %.2f]\n', aspect, unpack(g_aspect_ratio)) )
					check_passed = false
				end
			end
      -- Edge Margin
			if check_passed then
				local leftPoint= postStats.centroid[1] - postStats.axisMinor / 2
				local rightPoint= postStats.centroid[1] + postStats.axisMinor / 2
				local margin = math.min(leftPoint, wa - rightPoint)
				if margin <= g_margin then
					table.insert(fail, string.format('Edge margin:%.1f < %.1f\n', margin, g_margin))
					check_passed = false
				end
			end
			-- TODO: Add ground check

			-- Height Check
			if check_passed then
				local scale = postStats.axisMinor / postDiameter
				local v = check_coordinateA(postStats.centroid, scale)
         --print('GOAL HEIGHT:', v[3])
				if v[3] < Config.vision.goal.height_min then
					table.insert(fail, 'TOO LOW\n')
					check_passed = false
        elseif v[3]>Config.vision.goal.height_max then
					table.insert(fail, 'TO HIGH\n')
					check_passed = false
				end
			end

			-- Check # of valid postB
			if check_passed then
				table.insert(fail, 'is good\n')
				nPosts = nPosts + 1
				i_validB[#i_validB + 1] = i
				valid_posts[nPosts] = postStats
			end
    end  -- End of check on this postB
		table.insert(failures, table.concat(fail, ',') )
	end -- End of checks on all postB

	-- Goal type detection
	local post_detected = true
	if nPosts>2 or nPosts<1 then
		--TODO: this might have problem when robot see goal posts on other fields
		table.insert(failures, 'Bad number of posts')
		post_detected = false
	end

	-- 0:unknown 1:left 2:right 3:double
	local goalStats = {}
	if post_detected then
		-- Convert to body coordinate
		for i=1,nPosts do
      goalStats[i] = {}
			local good_postB = postB[ i_validB[1] ]
			local good_post = valid_posts[i]

			local scale1 = good_post.axisMinor / postDiameter
			local scale2 = good_post.axisMajor / postHeight
			local scale3 = math.sqrt(good_post.area / (postDiameter*postHeight))
			local scale
			if good_postB.boundingBox[3]<2 then
				--This post is touching the top, so we can only use diameter
				scale = scale1
			else
			  scale = math.max(scale1,scale2,scale3)
			end
			if scale == scale1 then
				goalStats[i].v = check_coordinateA(good_post.centroid, scale1)
			elseif scale == scale2 then
				goalStats[i].v = check_coordinateA(good_post.centroid, scale2)
			else
				goalStats[i].v = check_coordinateA(good_post.centroid, scale3)
			end
			--TODO: distanceFactor
			goalStats[i].post = good_post
			goalStats[i].postB = good_postB
		end

		-- Check goal type
    local fail_msg = {}
		if nPosts==2 then
			goalStats[1].type = 3
      goalStats[2].type = 3

      -- Goal width check in x-y space
      local dx = goalStats[1].v[1]-goalStats[2].v[1]
      local dy = goalStats[1].v[2]-goalStats[2].v[2]
      local dist = math.sqrt(dx*dx+dy*dy)
      if dist > goalWidth * 3 then --TODO: put into Config
        local fail_str = string.format("Goal too wide: %.1f > %.1f\n", dist, goalWidth*3)
        return table.insert(fail_msg, fail_str)
      elseif dist<goalWidth * 0.2 then
        local fail_str = string.format("Goal too wide: %.1f < %.1f\n", dist, goalWidth*0.2)
        return table.insert(fail_msg, fail_str)
      end

    else  -- Only single post is detected
      -- look for crossbar stats
      local dxCrossbar, crossbar_ratio
      --If the post touches the top, it should be an unknown post
      if goalStats[1].postB.boundingBox[3]<2 then --touching the top
        dxCrossbar = 0 --Should be unknown post
        crossbar_ratio = 0
      else
        -- The crossbar should be seen
        local postWidth = goalStats[1].post.axisMinor

        local leftX = goalStats[1].post.boundingBox[1]-5*postWidth
        local rightX = goalStats[1].post.boundingBox[2]+5*postWidth
        local topY = goalStats[1].post.boundingBox[3]-5*postWidth
        local bottomY = goalStats[1].post.boundingBox[3]+5*postWidth
        local bboxA = {leftX, rightX, topY, bottomY}

        local crossbarStats = ImageProc2.color_stats('a', colors.magenta, bboxA)
        dxCrossbar = crossbarStats.centroid[1] - goalStats[1].post.centroid[1]
        crossbar_ratio = dxCrossbar / postWidth
      end
      -- Determine left/right/unknown
      if (math.abs(crossbar_ratio) > min_crossbar_ratio) then
        if crossbar_ratio>0 then goalStats[1].type = 1
        else goalStats[1].type = 2 end
      else
        -- Eliminate small post without cross bars
        if goalStats[1].post.area < th_min_area_unknown_post then
          return table.insert(fail_msg, 'single post size too small')
        end
        -- unknown post
        goalStats[1].type = 0
      end

    end  --End of goal type check

    -- Convert torch tensor to table
    for i=1,#goalStats do
      goalStats[i].v = vector.new(goalStats[i].v)
			table.insert(failures, table.concat({'\n\n Goal Position',
				string.format('%.2f %.2f', goalStats[i].v[1], goalStats[i].v[2])},'\n') )
    end

	end

	if post_detected then
		return table.concat(failures, ','), goalStats
	end

  -- Yield the failure messages and the success tables
  return table.concat(failures, ',')
end
--]]

-- Set the variables based on the config file
function Vision.entry(cfg)
  colors = cfg.vision.colors

	HeadImage = ImageProc2.new(
    cfg.w, cfg.h, cfg.vision.scaleA, cfg.vision.scaleB
  )
	HeadImage:load_lut(table.concat{HOME, "/Data/", "lut_", cfg.lut, ".raw"})

  -- Center should be calibrated and saved in Config
  local focal_length, focal_base = cfg.focal_length, cfg.focal_base
  HeadImage.x0A = (HeadImage.wa-1)/2 + cfg.cx_offset
  HeadImage.y0A = (HeadImage.ha-1)/2 + cfg.cy_offset
  HeadImage.x0B = (HeadImage.wb-1)/2 + cfg.cx_offset / HeadImage.scaleB
  HeadImage.y0B = (HeadImage.hb-1)/2 + cfg.cy_offset / HeadImage.scaleB
  HeadImage.focalA = focal_length / (focal_base / HeadImage.wa)
  HeadImage.focalB = HeadImage.focalA / HeadImage.scaleB

  -- Ball thresholds
  if cfg.vision.ball then

    b_diameter = cfg.vision.ball.diameter
    b_dist = cfg.vision.ball.max_distance
    b_height0 = cfg.vision.ball.max_height0
    b_height1 = cfg.vision.ball.max_height1
    b_fill_rate = cfg.vision.ball.th_min_fill_rate
    b_bbox_area = cfg.vision.ball.th_min_bbox_area
    b_area = cfg.vision.ball.th_min_area
    b_ground_head_pitch = cfg.vision.ball.th_ground_head_pitch
    b_ground_boundingbox = cfg.vision.ball.th_ground_boundingbox
    b_ground_white = cfg.vision.ball.th_ground_white
    b_ground_green = cfg.vision.ball.th_ground_green
  end

  -- Goal thresholds
  if cfg.vision.goal then
    g_bbox_area = cfg.vision.goal.th_min_bbox_area
    g_area = cfg.vision.goal.th_min_area
    g_fill_rate = cfg.vision.goal.th_min_fill_rate
    g_orientation = cfg.vision.goal.th_min_orientation
    g_aspect_ratio = cfg.vision.goal.th_aspect_ratio
    g_margin = cfg.vision.goal.th_edge_margin
		th_nPostB = cfg.vision.goal.th_nPostB
    min_crossbar_ratio = cfg.vision.goal.min_crossbar_ratio
    th_min_area_unknown_post = cfg.vision.goal.th_min_area_unknown_post
  end

end

function Vision.update(meta, img)

  -- Images to labels
  --HeadImage:rgb_to_labelA(img)
  HeadImage:yuyv_to_labelA(img)
  HeadImage:block_bitor()
  HeadImage.tfL = T.from_flat(meta.tfL16)
  HeadImage.tfG = T.from_flat(meta.tfG16)
  HeadImage.qHead = vector.new(meta.head)
  HeadImage.t = meta.t

  local cc_d = HeadImage:color_countA()
	if cc_d[colors.magenta]>0 or cc_d[colors.cyan]>0 then
		print('Magenta, Cyan', cc_d[colors.magenta], cc_d[colors.cyan])
	end

  -- Debug the color count
  --for i=0,255 do if cc_d[i]~=0 then print(i, cc_d[i]) end end

  local ball, b_debug = find_ball(HeadImage)
  print('Ball', ball)

  local debug = {
    ball = b_debug or 'ball',
    post = 'goal',
    obstacle = 'obstacle',
  }
  local detect = {
    id = 'detect',
    debug = debug
  }
  if ball then detect.ball = ball end
  --local post_fails, posts = find_goal()

  -- Send the detected stuff over the channel every cycle
  return HeadImage, detect

end

return Vision
