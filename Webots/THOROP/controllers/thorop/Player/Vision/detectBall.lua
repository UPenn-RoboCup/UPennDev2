--simplified detectball
detectBall={}

local Config = require'Config'      -- For Ball and Goal Size
local ImageProc = require'ImageProc'
local Body = require'Body'
local libVision = require'libVision' --not sure whether this is appropriate

require'vcm'

-- Debugging flag
local debugging = false


-- Filtering parameters
diameter = Config.world.ballDiameter 
th_min_color = Config.vision.ball.th_min_color 
th_min_color2 = Config.vision.ball.th_min_color2 
th_min_fill_rate = Config.vision.ball.th_min_fill_rate 
th_height_max = Config.vision.ball.th_height_max 
th_ground_boundingbox = Config.vision.ball.th_ground_boundingbox 
th_min_green1 = Config.vision.ball.th_min_green1 
th_min_green2 = Config.vision.ball.th_min_green2 

check_for_ground = Config.vision.ball.check_for_ground 
check_for_field = Config.vision.ball.check_for_field or 0 

--[[
field_margin = Config.vision.ball.field_margin or 0 
th_headAngle = Config.vision.ball.th_headAngle or -10*math.pi/180 
--]]

local debug_msg
local function add_debug_message(str)
  debug_msg = debug_msg..str
end



--Detects a ball of a given color.
--@return Table containing whether a ball was detected
--If a ball is detected, also contains additional stats about the ball
function detectBall.detect(ballColor, colorCount, labelA, labelB, HeadTransform, t)
  debug_msg = ''

  headAngle = Body.get_head_position()
  --print("headPitch:",headAngle[2]*180/math.pi) 

  local ball = {}
  ball.detect = 0

  add_debug_message(string.format("\nBall: pixel count: %d\n",
    colorCount[ballColor] )) 

  -- threshold check on the total number of ball pixels in the image
  -- TODO: a more intellengent way of setting th_min_color?
  if (colorCount[ballColor] < th_min_color) then  	
    add_debug_message("pixel count fail")
    ball.debug_msg = debug_msg
    return ball 	
  end

  -- find connected components of ball pixels
  ballPropsB = ImageProc.connected_regions(
    labelB.data,
    labelB.m,
    labelB.n,
    ballColor)

  if (#ballPropsB == 0) then 
    ball.debug_msg = debug_msg
    return ball 
  end

  -- Check max 5 largest blobs 
  for i=1,math.min(5,#ballPropsB) do
    add_debug_message(string.format("Ball: checking blob %d/%d\n",i,#ballPropsB)) 

    check_passed = true
    ball.propsB = ballPropsB[i]
    ball.propsA = libVision.bboxStats(color, ballPropsB[i].boundingBox,nil,nil,labelA) 
    ball.bboxA = libVision.bboxB2A(ballPropsB[i].boundingBox) 


    local fill_rate = ball.propsA.area / 
	    libVision.bboxArea(ball.propsA.boundingBox) 
      add_debug_message(string.format("Area:%d\nFill rate:%2f\n",
         ball.propsA.area,fill_rate)) 

    if ball.propsA.area < th_min_color2 then
      --Area check
      add_debug_message("Area check fail\n") 
      check_passed = false
    elseif fill_rate < th_min_fill_rate then
      --Fill rate check
      add_debug_message("Fillrate check fail\n") 
      check_passed = false
    else
      -- diameter of the area
      dArea = math.sqrt((4/math.pi)*ball.propsA.area) 
     -- Find the  of the ball
      ballCentroid = ball.propsA.centroid 
      -- Coordinates of ball
      scale = math.max(dArea/diameter, ball.propsA.axisMajor/diameter) 


      add_debug_message(string.format(
        "Ball detected, centroid: %.1f %.1f\n",
        ballCentroid[1],ballCentroid[2]))

      v = HeadTransform.coordinatesA(ballCentroid, scale) 
      v_inf = HeadTransform.coordinatesA(ballCentroid,0.1) 

      add_debug_message(string.format(
	     "Ball v0: %.2f %.2f %.2f\n",v[1],v[2],v[3])) 




--[[        
      if v[3] > th_height_max then
        --Ball height check
        vcm.add_debug_message("Height check fail\n") 
        check_passed = false 

      elseif check_for_ground>0 and

        headAngle[2] < th_headAngle then
        -- ground check
        -- is ball cut off at the bottom of the image?
        local vmargin=labelA.n-ballCentroid[2] 
        vcm.add_debug_message("Bottom margin check\n") 
        vcm.add_debug_message(string.format(
    	  "lableA height: %d, centroid Y: %d diameter: %.1f\n",
  	     labelA.n, ballCentroid[2], dArea )) 
        --When robot looks down they may fail to pass the green check
        --So increase the bottom margin threshold
        if vmargin > dArea * 2.0 then
          -- bounding box below the ball
          fieldBBox = {} 
          fieldBBox[1] = ballCentroid[1] + th_ground_boundingbox[1] 
          fieldBBox[2] = ballCentroid[1] + th_ground_boundingbox[2] 
          fieldBBox[3] = ballCentroid[2] + .5*dArea 
				     + th_ground_boundingbox[3] 
          fieldBBox[4] = ballCentroid[2] + .5*dArea 
 				     + th_ground_boundingbox[4] 
          -- color stats for the bbox
          fieldBBoxStats = ImageProc.color_stats(Vision.labelA.data, 
  	    Vision.labelA.m, Vision.labelA.n, colorField, fieldBBox) 
          -- is there green under the ball?
          vcm.add_debug_message(string.format("Green check:%d\n",
	   	   fieldBBoxStats.area)) 
          if (fieldBBoxStats.area < th_min_green1) then
            -- if there is no field under the ball 
      	    -- it may be because its on a white line
            whiteBBoxStats = ImageProc.color_stats(Vision.labelA.data,
 	      Vision.labelA.m, Vision.labelA.n, colorWhite, fieldBBox) 
            if (whiteBBoxStats.area < th_min_green2) then
              vcm.add_debug_message("Green check fail\n") 
              check_passed = false 
            end
          end --end white line check
        end --end bottom margin check
      end --End ball height, ground check
--]]      
    end --End all check

--[[
    if check_passed then    
      ballv = {v[1],v[2],0} 
      pose=wcm.get_pose() 
      posexya=vector.new( {pose.x, pose.y, pose.a} ) 
      ballGlobal = util.pose_global(ballv,posexya)  
      if check_for_field>0 then
        if math.abs(ballGlobal[1]) > 
   	  Config.world.xLineBoundary + field_margin or
          math.abs(ballGlobal[2]) > 
	  Config.world.yLineBoundary + field_margin then

          vcm.add_debug_message("Field check fail\n") 
          check_passed = false 
        end
      end
    end
--]]    
    if check_passed then
      break 
    end
  end --End loop


  ball.debug_msg = debug_msg
  if not check_passed then
    ball.debug_msg = debug_msg
    return ball 
  end

  --TODO: this projection introduces HUGE error
  v_proj = HeadTransform.projectGround(v,diameter/2)
  --v_proj = HeadTransform.projectGround(v,0);
  add_debug_message(string.format(
	"Ball FLAT v_proj: %.2f %.2f %.2f\n",v_proj[1],v_proj[2],v_proj[3])) 


  --Projecting ball to flat ground makes large distance error
  --We are using declined plane for projection
  local vMag =math.max(0,math.sqrt(v[1]^2+v[2]^2)-0.50) 
  local bodyTilt = mcm.get_stance_bodyTilt() 
  local projHeight = vMag * math.tan(bodyTilt)

  local v_proj2=HeadTransform.projectGround(v,diameter/2-projHeight) 

  --Subtract foot offset 
  --TODO: 
  -- v[1]=v[1]-mcm.get_footX()
  v_proj2[1] = v_proj2[1] - 0.05  --hack
  
  local ball_shift = Config.ball_shift or {0,0} 
  --Compensate for camera tilt
  v_proj2[1]=v_proj2[1] + ball_shift[1] 
  v_proj2[2]=v_proj2[2] + ball_shift[2] 

  add_debug_message(string.format(
	"Ball TILT v_proj: %.2f %.2f %.2f\n",v_proj2[1],v_proj2[2],v_proj2[3])) 

  --[[  

  --Ball position ignoring ball size (for distant ball observation)
  v_inf=HeadTransform.projectGround(v_inf,diameter/2) 
  v_inf[1]=v_inf[1]-mcm.get_footX()
  wcm.set_ball_v_inf({v_inf[1],v_inf[2]})   

--]]

  ball.v = v
  ball.detect = 1
  ball.r = math.sqrt(ball.v[1]^2 + ball.v[2]^2) 

  -- How much to update the particle filter
  ball.dr = 0.25*ball.r 
  ball.da = 10*math.pi/180 
  ball.t = t

  ball.debug_msg = debug_msg

  --Write info on shm
  vcm.set_ball_detect(ball.detect)
  vcm.set_ball_centroid(ballCentroid)
  vcm.set_ball_diameter(dArea)
  vcm.set_ball_v(v)
  vcm.set_ball_r(r)
  vcm.set_ball_t(t)
  
  ball.debug_msg = debug_msg
  return ball
  
end

return detectBall
