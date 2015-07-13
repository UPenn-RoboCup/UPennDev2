local detectPost = {}
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









-- TODO: World config
local postDiameter = Config.vision.goal.postDiameter
local postHeight = Config.vision.goal.goalHeight
local goalWidth = Config.vision.goal.goalWidth
--
local th_nPostB
local g_area, g_bbox_area, g_fill_rate, g_orientation, g_aspect_ratio, g_margin
local colors
local config
function detectPost.entry(cfg, Image)
  config = cfg
  g_bbox_area = config.th_min_bbox_area
  g_area = config.th_min_area
  g_fill_rate = config.th_min_fill_rate
  g_orientation = config.th_min_orientation
  g_aspect_ratio = config.th_aspect_ratio
  g_margin = config.th_edge_margin
  th_nPostB = config.th_nPostB
  min_crossbar_ratio = config.min_crossbar_ratio
  th_min_area_unknown_post = config.th_min_area_unknown_post
  colors = Image.colors
end
function detectPost.update(Image)
  if type(Image)~='table' then
    return false, 'Bad Image'
  end
  -- Form the initial goal check
  local postB = ImageProc.goal_posts(
		tonumber(ffi.cast('intptr_t', ffi.cast('void *', Image.labelB_d))),
		Image.wb,
		Image.hb,
		colors.white)
  if not postB then return'None detected' end
  -- Now process each goal post
  -- Store failures in the array
  local failures, successes = {}, {}
	local nPosts, i_validB, valid_posts = 0, {}, {}

	--for i=1, math.min(#postB, th_nPostB) do
	for i=1, #postB do
		local passed = true
		local msgs = {}

		local post = postB[i]

		-- TODO: Use labelB?
		-- Grab the statistics in labelA
		local post_bboxB = post.boundingBox
		--[[
		-- TODO: Exact and not just bit and?
		local postStatsB = ImageProc2.color_stats(
			Image.labelB_d, Image.wb, Image.hb, colors.white, post_bboxB
		)
		--]]

		local post_bboxA = bboxB2A(post_bboxB, Image.scaleB)
		local postStatsA = ImageProc2.color_stats(
			Image.labelA_d, Image.wa, Image.ha, colors.white, post_bboxA
		)

		-- If no pixels then return
		if postStatsA.area < g_area then
			passed = false
			msgs[i] = string.format('Area: %d < %d', stats.area, g_area)
		end


		--TODO: bbox area check seems redundant
		local box_areaA, box_areaB
		if passed then
			if postStatsA then
				box_areaA = (post_bboxA[2] - post_bboxA[1] + 1) *
					(post_bboxA[4] - post_bboxA[3] + 1)
			end
			if postStatsB then
				box_areaB = (post_bboxB[2] - post_bboxB[1] + 1) *
					(post_bboxB[4] - post_bboxB[3] + 1)
			end

			if box_areaA < g_bbox_area then
				passed = false
				msgs[i] = string.format('Box area: %d < %d',box_area,g_bbox_area)
			end
		end

		local postStats = postStatsA
		--local postStats = postStatsB

		-- Get the fill rate
		if passed then
			-- TODO: depends on ball or goal
			--local fill_rate = postStats.area / box_area
			local fill_rate = postStats.area / (postStats.axisMajor * postStats.axisMinor)
			if fill_rate < g_fill_rate then
				passed = true
				msgs[i] = string.format('Fill rate: %.2f < %.2f', fill_rate, g_fill_rate)
			end
		end

		-- TODO: Add lower goal post bbox check
		-- Orientation check
		if passed then
			if math.abs(postStats.orientation) < g_orientation then
				passed = false
				msgs[i] = string.format('Oriented: %.1f < %.1f',
					postStats.orientation, g_orientation)
			end
		end

		-- Aspect Ratio check
		if passed then
			local aspect = postStats.axisMajor / postStats.axisMinor
			if (aspect < g_aspect_ratio[1]) or (aspect > g_aspect_ratio[2]) then
				passed = false
				msgs[i] = string.format('Aspect: %.2f [%.2f %.2f]\n', aspect, unpack(g_aspect_ratio))
			end
		end

		-- Edge Margin
		if passed then
			local leftPoint = postStats.centroid[1] - postStats.axisMinor / 2
			local rightPoint = postStats.centroid[1] + postStats.axisMinor / 2
			local margin = math.min(leftPoint, Image.wa - rightPoint)
			if margin <= g_margin then
				passed = false
				msgs[i] = string.format('Edge margin:%.1f < %.1f\n', margin, g_margin)
			end
		end

		-- TODO: Add ground check

		-- Height Check
		local v
		if passed then
			local scale = postStats.axisMinor / postDiameter
			-- coords A
			local v0 = vector.new{
		    Image.focalA,
		    -(postStatsA.centroid[1] - Image.x0A),
		    -(postStatsA.centroid[2] - Image.y0A),
		    scale,
		  }
			-- Put into the local and global frames
      local vL = Image.tfL * (v0 / v0[4])
			local vG = Image.tfG * (v0 / v0[4])
			postStats.vL = vL


			if postStats.vL[3] < Config.vision.goal.height_min then
				passed = false
				msgs[i] = 'TOO LOW'
			elseif postStats.vL[3] > Config.vision.goal.height_max then
				passed = false
				msgs[i] = 'TO HIGH'
			end
		end

		-- Check # of valid postB
		if passed then
			msgs[i] = 'GOOD'
			nPosts = nPosts + 1
			i_validB[#i_validB + 1] = i
			valid_posts[nPosts] = postStats
		end

	end -- End of checks on all postB

	-- Goal type detection
	-- TODO: this might have problem when robot see goal posts on other fields
	local post_detected = true
	if nPosts>2 or nPosts<1 then
		post_detected = false
		msgs[#msgs+1] = 'Bad number of posts'
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
				goalStats[i].v = postStats.vL
			elseif scale == scale2 then
				goalStats[i].v = check_coordinateA(good_post.centroid, scale2)
			else
				goalStats[i].v = check_coordinateA(good_post.centroid, scale3)
			end
			-- TODO: distanceFactor
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
function detectPost.exit()
end
return detectPost
