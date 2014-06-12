local detectGoal = {}

local ImageProc = require'ImageProc'
local Body = require'Body'
local libVision = require'libVision'
local vector = require'vector'

--TODO: Use tilted boundingbox? (robots with nonzero bodytilt)
use_tilted_bbox = Config.vision.use_tilted_bbox or 0 -- this is for camera roll
--Use center post to determine post type (disabled for OP)
use_centerpost = Config.vision.goal.use_centerpost or 0
--Check the bottom of the post for green
check_for_ground = Config.vision.goal.check_for_ground or 0
--Min height of goalpost (to reject false positives at the ground)
goal_height_min = Config.vision.goal.height_min or -0.8

---Detects a goal of a given color.
--@return Table containing whether a goal was detected
--If a goal is detected, also contains additional stats about the goal

distanceFactorYellow = Config.vision.goal.distanceFactorYellow or 1.0

	
--Post dimension
postDiameter = Config.world.postDiameter or 0.10
postHeight = Config.world.goalHeight or 1.80 --TODO
goalWidth = Config.world.goalWidth or 3  --TODO

use_white_wall = Config.vision.use_white_wall or 0
use_nonwhite_wall = Config.vision.use_nonwhite_wall or 0
white_wall_is_blue = Config.vision.white_wall_is_blue or 0
white_wall_min_count = Config.vision.white_wall_min_count or 0
white_wall_min_rate = Config.vision.white_wall_min_rate or 0

nonwhite_wall_min_area = Config.vision.nonwhite_wall_min_area or 0
nonwhite_wall_max_rate = Config.vision.nonwhite_wall_max_rate or 0


--------------------------------------------------------------
--Vision threshold values (to support different resolutions)
--------------------------------------------------------------
th_min_color_count=Config.vision.goal.th_min_color
th_min_area = Config.vision.goal.th_min_area
th_nPostB = Config.vision.goal.th_nPostB
th_min_orientation = Config.vision.goal.th_min_orientation
th_min_fill_rate = Config.vision.goal.th_min_fill_rate
th_aspect_ratio = Config.vision.goal.th_aspect_ratio
min_crossbar_ratio = Config.vision.goal.min_crossbar_ratio or 0.6
th_edge_margin = Config.vision.goal.th_edge_margin
th_bottom_boundingbox = Config.vision.goal.th_bottom_boundingbox
th_ground_boundingbox = Config.vision.goal.th_ground_boundingbox
th_min_green_ratio = Config.vision.goal.th_min_green_ratio
th_goal_separation = Config.vision.goal.th_goal_separation
th_min_area_unknown_post = Config.vision.goal.th_min_area_unknown_post


--TODO
local rollAngle = 0
local colorWhite = Config.vision.colorWhite

local debug_msg
local function add_debug_message(str)
  debug_msg = debug_msg..str
end

function detectGoal.detect(goalColor, colorCount, labelA, labelB, HeadTransform, t)
  debug_msg = ''
  
  local goal = {}
  goal.detect = 0
  
  -- TODO: check pixel count?

  local postB

  if use_tilted_bbox>0 then
    --[[----------------------------------------------------------------------
    
    --where shoud we update the roll angle? HeadTransform?
    rollAngle = HeadTransform.getCameraRoll()
    vcm.set_camera_rollAngle(rollAngle)

    --Tilted labelB test for OP

    scaleBGoal = 4

    Vision.labelBtilted={}
    Vision.labelBtilted.moffset = Vision.labelA.m/scaleBGoal/2
    Vision.labelBtilted.m = Vision.labelA.m/scaleBGoal*2
    Vision.labelBtilted.n = Vision.labelA.n/scaleBGoal
    Vision.labelBtilted.npixel = Vision.labelBtilted.m*Vision.labelBtilted.n

    Vision.labelBtilted.data = 
  	ImageProc.tilted_block_bitor(Vision.labelA.data, 
  	Vision.labelA.m, Vision.labelA.n, scaleBGoal, 
  	scaleBGoal, rollAngle)
    postB = ImageProc.goal_posts(Vision.labelBtilted.data, 
	    Vision.labelBtilted.m, Vision.labelBtilted.n, goalColor, th_nPostB)
    --discount tilt offset
    if postB then
      for i = 1,#postB do
        postB[i].boundingBox[1] = 
    	  postB[i].boundingBox[1]-Vision.labelBtilted.moffset
        postB[i].boundingBox[2] = 
	      postB[i].boundingBox[2]-Vision.labelBtilted.moffset
      end
    end
------------------------------------------------------------------------]]

  else
    rollAngle=0
    -- vcm.set_camera_rollAngle(rollAngle)
    postB = ImageProc.goal_posts(labelB.data,labelB.m,labelB.n,goalColor,th_nPostB)
  end

  if (not postB) then 	
    add_debug_message("No post detected\n")
    goal.debug_msg = debug_msg
    return goal 
  end

  local npost = 0
  local ivalidB = {}
  local postA = {}
  add_debug_message(string.format("Checking %d posts\n",#postB))

  lower_factor = 0.3

  for i = 1, #postB do
    local valid = true

    --Check lower part of the goalpost for thickness
    
    if use_tilted_bbox>0 then
      --[[----------------------------------------------------------------------
      
      add_debug_message("Use Tilted goalStats\n")
      goalStats = Vision.bboxStats(goalColor,postB[i].boundingBox,rollAngle,scaleBGoal)
------------------------------------------------------------------------]]
      
    else
      goalStats = libVision.bboxStats(goalColor,postB[i].boundingBox,nil,nil,labelA)
    end

    -- size and orientation check
    add_debug_message(string.format("Area check: %d\n", goalStats.area))
    
    if (goalStats.area < th_min_area) then
      add_debug_message("Area check fail\n")
      -- print(string.format("Area check fail\n Area: %d, 
        -- threshold: %d",goalStats.area, th_min_area))
      valid = false
    end

    if valid then
      local orientation = goalStats.orientation - rollAngle
      add_debug_message(string.format("Orientation check: %f\n", 
      	 180*orientation/math.pi))
      if (math.abs(orientation) < th_min_orientation) then
        add_debug_message("orientation check fail\n")
        valid = false
      end
    end
      
    --fill_rate check
    if valid then
      local fill_rate = goalStats.area / (goalStats.axisMajor * goalStats.axisMinor)
      add_debug_message(string.format("Fill rate check: %.2f\n", fill_rate))
      if (fill_rate < th_min_fill_rate) then 
        add_debug_message("Fill rate check fail\n")
        valid = false 
      end
    end

    --aspect ratio check
    if valid then
      local aspect = goalStats.axisMajor/goalStats.axisMinor
      add_debug_message(string.format("Major axis: %d, Minor axis %d\n", 
        goalStats.axisMajor, goalStats.axisMinor))
      add_debug_message(string.format("Aspect check: %d\n",aspect))
      if (aspect < th_aspect_ratio[1]) or 
          	(aspect > th_aspect_ratio[2]) then 
        add_debug_message("Aspect check fail\n")
        valid = false 
      end
    end

    --[[check edge margin
    if valid then

      local leftPoint= goalStats.centroid[1] - 
	      goalStats.axisMinor/2 * math.abs(math.cos(rollAngle))
      local rightPoint= goalStats.centroid[1] + 
	      goalStats.axisMinor/2 * math.abs(math.cos(rollAngle))
  
      add_debug_message(string.format("Left and right point: %d / %d\n", 
          leftPoint, rightPoint))

      local margin = math.min(leftPoint,labelA.m-rightPoint)

      add_debug_message(string.format("Edge margin check: %d\n",margin))

      if margin<=th_edge_margin then
        add_debug_message("Edge margin check fail\n")
        valid = false
      end
    end
    --]]

    --[[ ground check at the bottom of the post
    if valid and check_for_ground>0 then 
      local bboxA = Vision.bboxB2A(postB[i].boundingBox)
      if (bboxA[4] < th_bottom_boundingbox * Vision.labelA.n) then

        -- field bounding box 
        local fieldBBox = {}
        fieldBBox[1] = bboxA[1] + th_ground_boundingbox[1]
        fieldBBox[2] = bboxA[2] + th_ground_boundingbox[2]
        fieldBBox[3] = bboxA[4] + th_ground_boundingbox[3]
        fieldBBox[4] = bboxA[4] + th_ground_boundingbox[4]

        local fieldBBoxStats
      	if use_tilted_bbox>0 then
              -- color stats for the bbox
               fieldBBoxStats = ImageProc.tilted_color_stats(Vision.labelA.data, 
      		Vision.labelA.m,Vision.labelA.n,colorField,fieldBBox,rollAngle)
      	else
          fieldBBoxStats = ImageProc.color_stats(Vision.labelA.data, 
      		Vision.labelA.m,Vision.labelA.n,colorField,fieldBBox,rollAngle)
      	end
 
        local fieldBBoxArea = Vision.bboxArea(fieldBBox)
      	green_ratio=fieldBBoxStats.area/fieldBBoxArea
        add_debug_message(string.format(
      		"Green ratio check: %.2f\n",green_ratio))

        -- is there green under the ball?
        if (green_ratio<th_min_green_ratio) then
          add_debug_message("Green check fail")
          valid = false
        end
      end
    end
    --]]
    
    
    --Height Check
    if valid then
      scale = math.sqrt(goalStats.area / (postDiameter*postHeight) )
      v = HeadTransform.coordinatesA(goalStats.centroid, scale)
      if v[3] < goal_height_min then
        add_debug_message(string.format("Height check fail:%.2f\n",v[3]))
        valid = false 
      end
    end

    if valid then
      ivalidB[#ivalidB + 1] = i
      npost = npost + 1
      postA[npost] = goalStats
    end
  end

  add_debug_message(string.format("Total %d valid posts\n", npost ))

  if ((npost < 1) or (npost > 2)) then 
    add_debug_message("Post number failure\n")
    goal.debug_msg = debug_msg
    return goal 
  end

  goal.propsB = {}
  goal.propsA = {}
  goal.v = {}

  for i = 1,npost do
    goal.propsB[i] = postB[ivalidB[i]]
    goal.propsA[i] = postA[i]

    local scale1 = postA[i].axisMinor / postDiameter
    local scale2 = postA[i].axisMajor / postHeight
    local scale3 = math.sqrt(postA[i].area / (postDiameter*postHeight))

    if goal.propsB[i].boundingBox[3]<2 then 
      --This post is touching the top, so we can only use diameter
      add_debug_message("Post touching the top\n")
      scale = scale1
    else
      scale = math.max(scale1,scale2,scale3)
    end

    --TODO: goal x distance is smaller than actual distance
    -- seems by area is better?
    local v1 = HeadTransform.coordinatesA(postA[i].centroid, scale1)
    local v2 = HeadTransform.coordinatesA(postA[i].centroid, scale2)
    local v3 = HeadTransform.coordinatesA(postA[i].centroid, scale3)
    add_debug_message(string.format("Distance by diameter: %.1f\n",
	math.sqrt(v1[1]^2+v1[2]^2) ))
    add_debug_message(string.format("Distance by height: %.1f\n",
	math.sqrt(v2[1]^2+v2[2]^2) ))
    add_debug_message(string.format("Distance by area: %.1f\n",
	math.sqrt(v3[1]^2+v3[2]^2) ))

    if scale==scale1 then
      add_debug_message("Post distance measured by diameter\n")
      goal.v[i] = v1
    elseif scale==scale2 then
      add_debug_message("Post distance measured by height\n")
      goal.v[i] = v2
    else
      add_debug_message("Post distance measured by area\n")
      goal.v[i] = v3
    end

    goal.v[i][1]=goal.v[i][1]*distanceFactorYellow
    goal.v[i][2]=goal.v[i][2]*distanceFactorYellow

    add_debug_message(string.format("post[%d] = %.2f %.2f %.2f\n",
    	 i, goal.v[i][1], goal.v[i][2], goal.v[i][3]))
  end

  if (npost == 2) then
    goal.type = 3 --Two posts

    --SJ: goal width check in x-y space
    local dx = goal.v[1][1]-goal.v[2][1]
    local dy = goal.v[1][2]-goal.v[2][2]
    local dist = math.sqrt(dx*dx+dy*dy)
    add_debug_message(string.format('CHECK WIDTH: %.1f\n', dist))
    if dist > goalWidth * 2 then
      add_debug_message("Goal width check fail\n")
      goal.debug_msg = debug_msg
      return goal
    end

    ---- TRY ANOTHER SCALE ----
    local di = postA[1].centroid[1] - postA[2].centroid[1]
    local dj = postA[1].centroid[2] - postA[2].centroid[2]
    --local dist_ij = math.sqrt(di*di+dj*dj)
    local dist_ij = math.abs(di)
    local scale4 = dist_ij / goalWidth
    local centroid = {}
    for i=1,#postA[1].centroid do
    	centroid[i] = 0.5*(postA[1].centroid[i] + postA[2].centroid[i])
    end
    local v = HeadTransform.coordinatesA(centroid, scale4)
    add_debug_message(string.format("Distance by WIDTH : %.2f\n",v[1]))
    ---------------------------
    
    --White check around the goalpost
    local bboxAL =  libVision.bboxB2A(goal.propsB[1].boundingBox)
    local bboxAR =  libVision.bboxB2A(goal.propsB[2].boundingBox)

    local goalBBox={}
    goalBBox[1] = bboxAL[1]
    goalBBox[2] = bboxAR[2]
    goalBBox[3] = math.max(bboxAL[3],bboxAR[3]) --TODO
    goalBBox[4] = math.min(bboxAL[4],bboxAR[4])
    -- For monitoring
    goal.goalBBox = goalBBox
    
    local goalBBoxStats
    if use_tilted_bbox>0 then
        -- color stats for the bbox
      goalBBoxStats = ImageProc.tilted_color_stats(labelA.data, 
     labelA.m,labelA.n,colorWhite,goalBBox,rollAngle)
    else
      goalBBoxStats = ImageProc.color_stats(labelA.data, 
    	labelA.m,labelA.n,colorWhite,goalBBox,rollAngle)
    end
    
    local goalBBoxArea = libVision.bboxArea(goalBBox)
    local white_ratio = goalBBoxStats.area / goalBBoxArea
    add_debug_message(string.format("White area: %d/%d  rate: %.2f\n",
		  goalBBoxStats.area,goalBBoxArea, white_ratio))
      
    if use_white_wall>0 then
      if goalBBoxStats.area > white_wall_min_count and
        white_ratio > white_wall_min_rate then

        --TODO: White wall behind the goal
        if white_wall_is_blue>0 then

          add_debug_message("Blue goal detected\n")
          wcm.set_goal_color(1)-- Blue team goal
        else
          add_debug_message("Red goal detected\n")
          wcm.set_goal_color(2)-- Red team goal
        end  
      end
    end

    if use_nonwhite_wall >0 then  
       if goalBBoxArea > nonwhite_wall_min_area and
         white_ratio < nonwhite_wall_max_rate then

        if white_wall_is_blue>0 then
          add_debug_message("Red goal detected\n")
          wcm.set_goal_color(2)-- Red team goal
        else
          add_debug_message("Blue goal detected\n")
          wcm.set_goal_color(1)-- Red team goal
        end
      end    
    end
    
  else -- only one goalpost detected    
    
    goal.v[2] = vector.new({0,0,0,0})
    goal.goalBBox = vector.new({0,0,0,0})

    -- look for crossbar:
    local dxCrossbar, cross_ratio
    --If the post touches the top, it should be a unknown post
    if goal.propsB[1].boundingBox[3]<2 then --touching the top
      dxCrossbar = 0 --Should be unknown post
      crossbar_ratio = 0
      add_debug_message(string.format("Post top: %d, touching the top\n",
        goal.propsB[1].boundingBox[3]))
    else
      -- The crossbar should be seen
      local postWidth = postA[1].axisMinor

      local leftX = postA[1].boundingBox[1]-5*postWidth
      local rightX = postA[1].boundingBox[2]+5*postWidth
      local topY = postA[1].boundingBox[3]-5*postWidth
      local bottomY = postA[1].boundingBox[3]+5*postWidth    
      local bboxA = {leftX, rightX, topY, bottomY}

      local crossbarStats = ImageProc.color_stats(labelA.data,labelA.m,labelA.n, goalColor,bboxA,rollAngle)
      dxCrossbar = crossbarStats.centroid[1] - postA[1].centroid[1]
      crossbar_ratio = dxCrossbar / postWidth

    end
    
    add_debug_message(string.format(
    	"Crossbar stat: %.2f\n",crossbar_ratio))

    if (math.abs(crossbar_ratio) > min_crossbar_ratio) then
      if (dxCrossbar > 0) then
        if use_centerpost>0 then goal.type = 1 -- left post
        else goal.type = 0 end  -- unknown post
      else
        if use_centerpost>0 then goal.type = 2  -- right post
        else goal.type = 0 end  -- unknown post
      end
    else
      -- unknown post
      goal.type = 0
      -- eliminate small posts without cross bars
      add_debug_message(string.format(
      	"Unknown single post size check:%d\n",postA[1].area))
      
      if (postA[1].area < th_min_area_unknown_post) then
        add_debug_message("Post size too small")
        goal.debug_msg = debug_msg
        return goal
      end
    end
    
  end-- end of check goal type  

  if goal.type==0 then
    add_debug_message(string.format("Unknown single post detected\n"))
  elseif goal.type==1 then
    add_debug_message(string.format("Left post detected\n"))
  elseif goal.type==2 then
    add_debug_message(string.format("Right post detected\n"))
  elseif goal.type==3 then
    add_debug_message(string.format("Two posts detected\n"))
  end

  goal.detect = 1
  goal.t = t
  goal.debug_msg = debug_msg
  
  vcm.set_goal_v1(goal.v[1])
  vcm.set_goal_v2(goal.v[2])
  vcm.set_goal_detect(1)
  vcm.set_goal_type(goal.type)
  vcm.set_goal_t(t)
  
  return goal
end

return detectGoal
