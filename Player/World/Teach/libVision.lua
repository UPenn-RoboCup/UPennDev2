-- libVision
-- (c) 2014 Stephen McGill
-- General Detection methods
local libVision = {}
-- Detection and HeadTransform information
local ImageProc = require'ImageProc'
local ImageProc2 = require'ImageProc.ffi2'
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

local vision_ch = si.new_publisher'vision'

-- Important local variables
local colors
local w, h, wa, wb, ha, scaleA, scaleB
local labelA_d, labelB_d
-- Camera information
local focal_length, focal_base
local x0A, y0A, focalA
local x0B, y0B, focalB
-- Detection thresholds
local th_nPostB, g_area, g_bbox_area, g_fill_rate, g_orientation, g_aspect_ratio, g_margin
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

function libVision.get_metadata()
end

-- Should have a common API (meta/raw)
function libVision.send()

  local to_send = {}
  local lA_raw = c_zlib(ImageProc2.labelA_d, ImageProc2.labelA_n)
  local lA_meta = {
    w = wa,
    h = ha,
    sz = #lA_raw,
    c = 'zlib',
    id = 'labelA',
  }
  to_send[1] = {lA_meta, lA_raw}
  to_send[2] = {detected}
  return to_send
end

local function bboxB2A(bboxB)
	return {
		scaleB * bboxB[1],
		scaleB * bboxB[2] + scaleB - 1,
		scaleB * bboxB[3],
		scaleB * bboxB[4] + scaleB - 1,
	}
end

-- Simple bbox with no tilted color stats
-- Assume always input bboxB
local function bboxStats(label, color, bboxB)
	local bbox = label=='a' and bboxB2A(bboxB) or bboxB
  local area = (bbox[2] - bbox[1] + 1) * (bbox[4] - bbox[3] + 1)
	local stats = ImageProc2.color_stats(label, color, bbox)
  return stats, area
end

local function check_prop(color, prop, th_bbox_area, th_area, th_fill, labelA_t)
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
	-- TODO: depend on ball or goal
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

-- TODO: Allow the loop to run many times
local function find_goal()
  -- Form the initial goal check
  local postB = ImageProc.goal_posts(
		tonumber(ffi.cast('intptr_t', ffi.cast('void *', ImageProc2.labelB_d))),
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
		table.insert(failures, table.concat({'Bad post number'},','))
		post_detected = false
	end

	-- 0:unknown 1:left 2:right 3:double
	local goalStats = {}
	if post_detected then
		-- Convert to body coordinate
		for i=1,nPosts do
      goalStats[i] = {}
			local good_postB = postB[i_validB[1]]
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

-- Set the variables based on the config file
function libVision.entry()
	local cfg = Config.camera[1]
  -- Recompute the width and height of the images
  w, h = cfg.w, cfg.h
  -- Set up ImageProc
  ImageProc2.setup(w, h, scaleA, scaleB)
  focal_length, focal_base = cfg.focal_length, cfg.focal_base

  -- Save the scale paramter
  scaleA, scaleB = cfg.vision.scaleA, cfg.vision.scaleB
  colors = cfg.vision.colors

	-- Camera properties
  wa, ha = w / scaleA, h / scaleA
  wb, hb = wa / scaleB, ha / scaleB
  -- Center should be calibrated and saved in Config
  x0A, y0A = 0.5*(wa-1)+cfg.cx_offset, 0.5*(ha-1)+cfg.cy_offset
  x0B, y0B = 0.5*(wb-1)+cfg.cx_offset/scaleB, 0.5*(hb-1)+cfg.cy_offset/scaleB
  focalA = focal_length / (focal_base / wa)
  focalB = focalA / scaleB

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
  -- Load the lookup table
  ImageProc2.load_lut(table.concat{HOME, "/Data/", "lut_", cfg.lut, ".raw"})
end

function libVision.update(img)

  -- Images to labels
  ImageProc2.yuyv_to_labelA(img)
  ImageProc2.block_bitor()
  -- Detection System
  -- NOTE: Muse entry each time since on webots, we switch cameras
  -- In camera wizard, we do not switch cameras, so call only once
	--[[
  local cc_d = ImageProc2.color_countA()
	if cc_d[colors.magenta]>0 or cc_d[colors.cyan]>0 then
		print('Magenta, Cyan', cc_d[colors.magenta], cc_d[colors.cyan])
	end
	--]]
	
  local post_fails, posts = find_goal()

  -- Save the detection information
  detected.posts = posts

  detected.debug = {
		post = post_fails or ' ',
	}
    
  -- Send the detected stuff over the channel every cycle
  vision_ch:send(mp.pack(detected))

end

return libVision
