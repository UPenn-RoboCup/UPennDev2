-- libVision
-- (c) 2014 Stephen McGill
-- General Detection methods
local libVision = {}
-- Detection and HeadTransform information
local ImageProc = require'ImageProc'
local ImageProc2 = require'ImageProc.ffi'
local T = require'libTransform'
local mp = require'msgpack.MessagePack'
local vector = require'vector'
local util = require'util'
local zlib = require'zlib.ffi'
local si = require'simple_ipc'
require'wcm'
-- Body should be optional...
local Body
-- Important local variables
local w, h, wa, wb, ha, hab, scaleA, scaleB, lut_t
-- Hold on to these
local labelA_t, labelB_t
-- Head transform
local trHead, vHead, trNeck, trNeck0, dtrCamera
-- Camera information
local x0A, y0A, focalA, focal_length, focal_base
local x0B, y0B, focalB
-- Detection thresholds
local b_diameter, b_dist, b_height, b_fill_rate, b_bbox_area, b_area
local th_nPostB, g_area, g_bbox_area, g_fill_rate, g_orientation, g_aspect_ratio, g_margin
-- Store information about what was detected
local detected = {
  id = 'detect',
}
-- World config
local postDiameter = Config.world.postDiameter
local postHeight = Config.world.goalHeight
local goalWidth = Config.world.goalWidth
-- Field
local xMax = Config.world.xMax
local yMax = Config.world.yMax

--
local colors
local c_zlib = zlib.compress_cdata
local vision_ch = si.new_publisher'vision'


-- Debug printing in terminal
local DEBUG = Config.debug.obstacle


function libVision.get_metadata()
end

-- Should have a common API (meta/raw)
function libVision.send()
  local to_send = {}
  local lA_raw = c_zlib(labelA_t:data(), labelA_t:nElement(), true)
  local lA_meta = {
    w = labelA_t:size(2),
    h = labelA_t:size(1),
    sz = #lA_raw,
    c = 'zlib',
    id = 'labelA',
  }
  to_send[1] = {lA_meta, lA_raw}
  to_send[2] = {detected}
  return to_send
end

-- Update the Head transform
-- Input: Head angles
local function update_head()
	if not Body then return end
	-- Get from Body...
  local head = Body.get_head_position()
  -- TODO: Add any bias for each robot
  trNeck = trNeck0 * T.rotZ(head[1]) * T.rotY(head[2])
  trHead = trNeck * dtrCamera
  -- Grab the position only
  vHead = T.get_pos(trHead)
end

local function bboxB2A(bboxB)
  local bboxA = {}
  bboxA[1] = scaleB * bboxB[1];
  bboxA[2] = scaleB * bboxB[2] + scaleB - 1;
  bboxA[3] = scaleB * bboxB[3];
  bboxA[4] = scaleB * bboxB[4] + scaleB - 1;
  return bboxA
end

-- Simple bbox with no tilted color stats
-- Assume always input bboxB
local function bboxStats(label, color, bboxB)
  if label=='a' then 
    bbox = bboxB2A(bboxB) 
    local area = (bbox[2] - bbox[1] + 1) * (bbox[4] - bbox[3] + 1)
  	local stats = ImageProc.color_stats(labelA_t, color, bbox)
    return stats, area
  else
    local area = (bboxB[2] - bboxB[1] + 1) * (bboxB[4] - bboxB[3] + 1)
  	local stats = ImageProc.color_stats(labelB_t, color, bboxB)
    return stats, area
  end
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
local function check_coordinateA(centroid, scale, maxD, maxH)
  local v0 = torch.Tensor({
    focalA,
    -(centroid[1] - x0A),
    -(centroid[2] - y0A),
    scale,
  })
  local v = torch.mv(trHead, v0) / v0[4]
    
  -- Check the distance
  if maxD and v[1]*v[1] + v[2]*v[2] > maxD*maxD then
    return'TOO FAR'
  elseif maxH and v[3] > maxH then
    return'TOO HEIGH'
  end
  return v
end

-- Yield coordinates in the labelB space
-- Returns an error message if max limits are given
local function check_coordinateB(centroid, scale, maxD, maxH)
  local v0 = torch.Tensor({
    focalB,
    -(centroid[1] - x0B),
    -(centroid[2] - y0B),
    scale,
  })
  local v = torch.mv(trHead, v0) / v0[4]
  -- Check the distance
  if maxD and v[1]*v[1] + v[2]*v[2] > maxD*maxD then
    return'Distance'
  elseif maxH and v[3] > maxH then
    return'Height'
  end
  return v
end

local function projectGround(v,targetheight)
  targetheight = targetheight or 0
  local vout = vector.new(v)
  local vHead_homo = vector.new({vHead[1], vHead[2], vHead[3], 1})
  --Project to plane
  if vHead[3]>targetheight then
    vout = vHead_homo +
      (vout-vHead_homo)*( (vHead[3]-targetheight) / (vHead[3]-vout[3]) )
  end

  return vout
end

function libVision.ball(labelA_t, labelB_t, cc_t)
  -- print('Black pixels?', cc_t[colors.black])
  -- print('Red pixels?', cc_t[colors.orange])
  -- print('Yellow pixels?', cc_t[colors.yellow])

  -- The ball is color 1
  local cc = cc_t[colors.orange]
  if cc<6 then return'Color count' end
  -- Connect the regions in labelB
  local ballPropsB = ImageProc.connected_regions(labelB_t, 1)
  if not ballPropsB then return'No connected regions' end
  local nProps = #ballPropsB
  if nProps==0 then return'0 connected regions' end
  --
  local failures, successes = {}, {}
  for i=1,math.min(5, nProps) do
    local fail = {}
    -- Check the image properties
    local propsB = ballPropsB[i]
    local propsA = check_prop(colors.orange, propsB, b_bbox_area, b_area, b_fill_rate, labelA_t)
    if type(propsA)=='string' then
      table.insert(fail, propsA)
    else
      -- Check the coordinate on the field
      local dArea = math.sqrt((4/math.pi) * propsA.area)
      local scale = math.max(dArea/b_diameter, propsA.axisMajor/b_diameter);
      local v = check_coordinateA(propsA.centroid, scale, b_dist, b_height)
      if type(v)=='string' then
        table.insert(fail, v)
      else        
  			-- Height Check
  			if check_passed then
  				local scale = postStats.axisMinor / postDiameter 
  				local v = check_coordinateA(postStats.centroid, scale)
  				if v[3] < Config.vision.goal.height_min then
  					table.insert(fail,string.format("Height fail:%.2f\n",v[3]))
  					check_passed = false 
  				end
  			end
        

        -- Project the ball to the ground
        propsA.v = projectGround(v,b_diameter/2)
        propsA.t = Body and Body.get_time() or 0
				-- For ballFilter
			  propsA.r = math.sqrt(v[1]*v[1]+v[2]*v[2])
				propsA.dr = 0.25*propsA.r --TODO: tweak 
				propsA.da = 10*math.pi/180
        -- TODO: Check if outside the field
        -- TODO: Ground color check
      end
    end
    -- Did we succeed in finding a ball?
    if #fail==0 then
      return tostring(propsA.v), propsA
    else
      -- Add another failure
      table.insert(failures, table.concat(fail,', '))
    end
  end
  -- Assume failure
  return table.concat(failures,', ')
end


-- TODO: Allow the loop to run many times
function libVision.goal(labelA_t, labelB_t, cc_t)
  -- Form the initial goal check
  local postB = ImageProc.goal_posts(labelB_t, colors.yellow)
  if not postB then return'None detected' end
  -- Now process each goal post
  -- Store failures in the array
  local failures, successes = {}, {}
	local nPosts, i_validB, valid_posts = 0, {}, {}
	--for i=1, math.min(#postB, th_nPostB) do
	for i=1, #postB do
		local post = postB[i]
    local fail, has_stats = {}, true
    local postStats = check_prop(colors.yellow, post, g_bbox_area, g_area, g_fill_rate, labelA_t)
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
				if v[3] < Config.vision.goal.height_min then
					table.insert(fail,string.format("Height fail:%.2f\n",v[3]))
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
        print(fail_str)
        return table.insert(fail_msg, fail_str)
      elseif dist<goalWidth * 0.2 then
        local fail_str = string.format("Goal too wide: %.1f < %.1f\n", dist, goalWidth*0.2)
        print(fail_str)
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

        local crossbarStats = ImageProc.color_stats(labelA_t, colors.yellow, bboxA)
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
    wcm.set_goal_t(Body.get_time())

	end
    
	if post_detected then
		return table.concat(failures, ','), goalStats
	end

  -- Yield the failure messages and the success tables
  return table.concat(failures,',')
end

-- Obstacle detection
function libVision.obstacle(labelB_t, cc)
  -- If no black pixels
  if cc[colors.black]<100 then return 'Not much black pixels' end
  -- Obstacle table
  local obstacle, obs_count, obs_debug = {}, 0, ''
  obstacle.iv, obstacle.v, obstacle.bbox = {}, {}, {}
  obstacle.dist = {}
  
  -- Parameters  TODO: put into entry()
  local grid_x = Config.vision.obstacle.grid_x
  local grid_y = Config.vision.obstacle.grid_y
  local th_min_area = Config.vision.obstacle.th_min_area
  local th_green_fill_rate = Config.vision.obstacle.th_green_fill_rate
  local th_max_height = Config.vision.obstacle.th_max_height
  local th_min_height = Config.vision.obstacle.th_min_height
  local th_green_black_ratio = Config.vision.obstacle.th_green_black_ratio
  local th_min_orientation = Config.vision.obstacle.th_min_orientation
  local th_near_check = Config.vision.obstacle.th_near_check
  
  local col = wb / grid_x
  local row = hb / grid_y 
  for i=1, col do
    for j=1,row do
      local leftX = (i-1)* grid_x+1
      local topY = (j-1)* grid_y+1
      local rightX = math.min(wb, i* grid_x)
      local bottomY = math.min(hb, j* grid_y)

      local bboxB = {leftX, rightX, topY, bottomY}
      -- local blackStats = bboxStats('b', colors.black, bboxB)
      local blackStats, greenStats = bboxStats('a', colors.black, bboxB)

      ----------- Checks -----------
      local check_passed, v, obstacle_dist = true
      
      -- Check black area       
      if blackStats.area < th_min_area then
        check_passed = false
        obs_debug = obs_debug..string.format('FAIL black area:%.1f < %.1f\n',
          blackStats.area, th_min_area)
      end
      -- Check orientation
      if check_passed then
        if math.abs(blackStats.orientation)<th_min_orientation then
          check_passed = false
          obs_debug = obs_debug..string.format('Orientation: %.2f < %.2f\n',
            math.abs(blackStats.orientation), th_min_orientation)
        end
      end
      -- Check aspect ratio TODO: might be not necessary
      if check_passed and blackStats.axisMinor<grid_x then
        local aspect_ratio = blackStats.axisMajor / blackStats.axisMinor
        local th_aspect = {1.1, 10}
        if aspect_ratio < th_aspect[1] then
          check_passed = false
          obs_debug = obs_debug..string.format('Aspect ratio: %.2f < %.2f\n',
            aspect_ratio, th_aspect[1])
        elseif aspect_ratio > th_aspect[2] then
          check_passed = false
          obs_debug = obs_debug..string.format('Aspect ratio: %.2f > %.2f\n',
            aspect_ratio, th_aspect[2])
        end  
      end

      if check_passed then
        --TODO: if quite close to the obs, the axisMinor > grid_x
        local scale = math.max(1, blackStats.axisMinor / Config.world.obsDiameter)
        -- v = check_coordinateB(blackStats.centroid, scale)
      	v = check_coordinateA(blackStats.centroid, scale)
      end
      
      -- Height check
      if check_passed and v[3]>th_max_height then
        check_passed = false
        obs_debug = obs_debug..string.format('TOO High:%.2f > %.2f\n',
          v[3], th_max_height)
      end
      if check_passed and v[3]<th_min_height then
        check_passed = false
        obs_debug = obs_debug..string.format('TOO low: %.2f < %.2f\n',
          v[3], th_min_height)
      end

      if check_passed then
        v = projectGround(v, v[3]) --TODO
        obstacle_dist = math.sqrt(v[1]*v[1]+v[2]*v[2])
      end      
      
      -- Field bounds check
      if check_passed then
        local global_v = util.pose_global({v[1], v[2], 0}, wcm.get_robot_pose())
        if math.abs(global_v[1])>xMax or math.abs(global_v[2])>yMax then
          check_passed = false
          obs_debug = obs_debug..'OUTSIDE FIELD!'
        end
      end
      -- Distance check
      if check_passed and obstacle_dist>7 then  --TODO
        check_passed = false
        obs_debug = obs_debug..string.format('TOO FAR:%.2f >%.2f\n', obstacle_dist, 5)
      end
      
      --TODO: due to the way we split image into blocks, this has some issue 
      -- AND we are not actually looking for obstacles when head pitch < 30 deg
      -- --Ground check: might be slow?
      -- if check_passed and ha-blackStats.boundingBox[4]>10 then
      --   -- local left_x = blackStats.boundingBox[1]
      --   -- local right_x = blackStats.boundingBox[2]
      --   -- local top_y = blackStats.boundingBox[4]
      --   -- local bot_y = blackStats.boundingBox[4]+10
      --
      --   local left_x = leftX
      --   local right_x = rightX
      --   local top_y = topY
      --   local bot_y = math.min(ha, bottomY+20)
      --   local ground_bbox = {left_x, right_x, top_y, bot_y}
      --
      --   local groundStats, bbox_area = bboxStats('b', colors.field, ground_bbox)
      --   if groundStats.area/bbox_area < 0.5 then  --TODO
      --     check_passed = false
      --     obs_debug = obs_debug..string.format('GROUND CHECK FAIL: %.2f < %.2f\n',
      --       groundStats.area/bbox_area, 0.5)
      --   end
      -- end
      
      
      -- Green fill rate
      if check_passed then
        greenStats, bbox_area = bboxStats('a', colors.field, bboxB)
        local fill_rate = greenStats.area / bbox_area
        if fill_rate > th_green_fill_rate then
          check_passed = false
          obs_debug = obs_debug..string.format('green fill rate: %.2f > %.2f\n',
            fill_rate, th_green_fill_rate)
        end
      end
      -- Green/black ratio check
      -- To screen out false positives when big black region is close
      if check_passed then
        local dist_th = grid_x/blackStats.axisMinor
        local green_black_ratio =  greenStats.area / (blackStats.axisMinor*blackStats.axisMajor)
        if dist_th<th_near_check and green_black_ratio > th_green_black_ratio*dist_th then
          check_passed = false
          obs_debug = obs_debug..string.format('Too much green: %.2f > %.2f\n',
            green_black_ratio, th_green_black_ratio*dist_th)
        else
          if DEBUG then 
            print('\ngrid_x/axisMinor:', dist_th)
            print('green black ratio', green_black_ratio,th_green_black_ratio*dist_th)
            print('black pixels',cc[colors.black], 'black area', blackStats.area)
            print('green fill:', greenStats.area / bbox_area)
            print('obs v no proj:', vector.new(v)) 
            print('axisminor:', blackStats.axisMinor, 'grid_x', grid_x*scaleB)
          end
          
          obs_count = obs_count + 1
          table.insert(obstacle.dist, obstacle_dist)
          obstacle.iv[obstacle_dist] = vector.new(blackStats.centroid)
          obstacle.bbox[obstacle_dist] = vector.new(blackStats.boundingBox)
          obstacle.v[obstacle_dist] = v
          obstacle.detect = 1
          obstacle.count = obs_count
        end
      end
      
    end -- end row
  end -- end col
    
  if obstacle.detect == 1 then
    -- Might be not necessary?
    -- Sort to get the closest three 
    local obsStats = {}
    obsStats.iv, obsStats.bbox, obsStats.v = {},{},{}
    table.sort(obstacle.dist)
    for i=1, math.min(3, obstacle.count) do
      obsStats.iv[i] = obstacle.iv[obstacle.dist[i]]
      obsStats.v[i] = obstacle.v[obstacle.dist[i]]
      obsStats.bbox[i] = obstacle.bbox[obstacle.dist[i]]
    end    
    
    --return 'Detected', obsStats
    return obs_debug, obsStats
  else
    return obs_debug
  end
end


-- Set the variables based on the config file
function libVision.entry(cfg, body)
  -- Dynamically load the body
  Body = body
  -- Recompute the width and height of the images
  w, h = cfg.w, cfg.h
  -- Set up ImageProc
  ImageProc2.setup(w, h, scaleA, scaleB)
  focal_length, focal_base = cfg.focal_length, cfg.focal_base

  -- Save the scale paramter
  scaleA, scaleB = cfg.vision.scaleA, cfg.vision.scaleB
  colors = cfg.vision.colors

  wa, ha = w / scaleA, h / scaleA
  wb, hb = wa / scaleB, ha / scaleB
  -- Center should be calibrated and saved in Config
  x0A, y0A = 0.5*(wa-1)+cfg.cx_offset, 0.5*(ha-1)+cfg.cy_offset
  x0B, y0B = 0.5*(wb-1)+cfg.cx_offset/scaleB, 0.5*(hb-1)+cfg.cy_offset/scaleB
  -- Delta transform from neck to camera
  dtrCamera = T.trans(unpack(cfg.head.cameraPos or {0,0,0}))
  * T.rotY(cfg.head.pitchCamera or 0)
  focalA = focal_length / (focal_base / wa)
  focalB = focalA / scaleB
  -- TODO: get from shm maybe?
  trNeck0 = T.trans(-Config.walk.footX, 0, Config.walk.bodyHeight)
  * T.rotY(Config.walk.bodyTilt)
  * T.trans(cfg.head.neckX, 0, cfg.head.neckZ)
	-- Initial guess
  trNeck = trNeck0 * T.rotZ(0) * T.rotY(0)
  trHead = trNeck * dtrCamera
  -- Load ball
  if cfg.vision.ball then
    b_diameter = cfg.vision.ball.diameter
    b_dist = cfg.vision.ball.max_distance
    b_height = cfg.vision.ball.max_height
    b_fill_rate = cfg.vision.ball.th_min_fill_rate
    b_bbox_area = cfg.vision.ball.th_min_bbox_area
    b_area = cfg.vision.ball.th_min_area
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
  -- Load the lookup table
  local lut_fname = {HOME, "/Data/", "lut_", cfg.lut, ".raw"}
  lut_t = ImageProc2.load_lut (table.concat(lut_fname))
end

function libVision.update(img)

  -- Update the motion elements
  update_head()

  -- Images to labels
  labelA_t = ImageProc2.yuyv_to_label(img, lut_t:data())
  labelB_t = ImageProc2.block_bitor(labelA_t)
  -- Detection System
  -- NOTE: Muse entry each time since on webots, we switch cameras
  -- In camera wizard, we do not switch cameras, so call only once
  local cc = ImageProc2.color_count(labelA_t)
  local ball_fails, ball = libVision.ball(labelA_t, labelB_t, cc)
  local post_fails, posts = libVision.goal(labelA_t, labelB_t, cc)
	local obstacle_fails, obstacles
  
	local head_angle = Body.get_head_position()
	-- If looking down, then do not detect obstacles
	if head_angle[2]>50*DEG_TO_RAD then
		obstacle_fails = 'looking down'
	else
		obstacle_fails, obstacles = libVision.obstacle(labelB_t, cc)
	end

  -- Save the detection information
  detected.ball = ball
  detected.posts = posts
  detected.obstacles = obstacles
    
  -- Debug messages
  -- detected.debug = table.concat({'Ball',ball_fails,'Posts',post_fails},'\n')
  -- detected.debug = table.concat({'Posts',post_fails},'\n')
  detected.debug = table.concat({'Obstacle', obstacle_fails},'\n')

  -- Send the detected stuff over the channel every cycle
  vision_ch:send(mp.pack(detected))

end

return libVision
