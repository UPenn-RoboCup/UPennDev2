-- Temp vision library for DRC site visit 

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
require'hcm'
require'gcm'


-- Body should be optional...
local Body
-- Important local variables
local w, h, wa, wb, ha, hab, scaleA, scaleB, lut_t
-- Hold on to these
local labelA_t, labelB_t
-- Head transform
local trHead, vHead, trNeck, trNeck0
local dtrCamera, cameraPitch, cameraRoll, cameraPos
-- Camera information
local x0A, y0A, focalA, focal_length, focal_base
local x0B, y0B, focalB
-- Detection thresholds
local b_diameter, b_dist, b_height0, b_height1, b_fill_rate, b_bbox_area, b_area
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
local xMax = 4.5 --Config.world.xMax
local yMax = 3   --Config.world.yMax

--
local colors
local c_zlib = zlib.compress
local vision_ch = si.new_publisher'vision'


local ball_debug=''
local function debug_ball_clear() ball_debug='' end
local function debug_ball(str)  ball_debug = ball_debug..str end


function libVision.get_metadata()
end

t_last_sent = unix.time()

-- Should have a common API (meta/raw)
function libVision.send()

--  print("label sent delay: ms",(unix.time()-t_last_sent ) *1000 )
  t_last_sent = unix.time()

  local to_send = {}
  local lA_raw = c_zlib(labelA_t:data(), labelA_t:nElement())
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
  local headBias = hcm.get_camera_bias()
  head[1] = head[1] - headBias[1]  
	
  local rpy = Body.get_rpy()
  --SJ: let's use imu value to recalculate camera transform every frame
  trNeck0 = T.trans(-Config.walk.footX, 0, Config.walk.bodyHeight)
  * T.rotY(rpy[2])
  * T.trans(Config.head.neckX, 0, Config.head.neckZ)
  trNeck = trNeck0 * T.rotZ(head[1]) * T.rotY(head[2])
	
	cameraPitch = headBias[2]
	cameraRoll = headBias[3]
	cameraYaw = headBias[4]

	dtrCamera = T.trans(unpack(cameraPos))
  * T.rotY(cameraPitch or 0)
  * T.rotX(cameraRoll or 0)
  * T.rotZ(cameraYaw or 0)

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
local function check_coordinateA(centroid, scale, maxD, maxH1, maxH2,balldebug)
  local v0 = torch.Tensor({
    focalA,
    -(centroid[1] - x0A),
    -(centroid[2] - y0A),
    scale,
  })

 local v = torch.mv(trHead, v0) / v0[4]

--  if balldebug then debug_ball(string.format("Ball pre-v0:%.2f %.2f %.2f\n",v0[1]/scale,v0[2]/scale,v0[3]/scale)) end
  if balldebug then debug_ball(string.format("Ball v0:%.2f %.2f %.2f\n",v[1],v[2],v[3])) end

  local maxH
  if maxH1 and maxH2 then maxH = maxH1 + math.sqrt(v[1]^2+v[2]^2)*maxH2 end

  -- Check the distance
  if maxD and v[1]*v[1] + v[2]*v[2] > maxD*maxD then
    return string.format("Distance:%.2f>%.2f",math.sqrt(v[1]*v[1] + v[2]*v[2]),maxD) 
  elseif maxH and v[3] > maxH then
    return string.format("Height:%.2f>%.2f",v[3],maxH)
  end
  return v
end

-- Yield coordinates in the labelB space
-- Returns an error message if max limits are given
local function check_coordinateB(centroid, scale, maxD, maxH, balldebug)
  local v0 = torch.Tensor({
    focalB,
    -(centroid[1] - x0B),
    -(centroid[2] - y0B),
    scale,
  })
  local v = torch.mv(trHead, v0) / v0[4]
  if balldebug then debug_ball(string.format("Ball v0:%.2f %.2f %.2f\n",v[1],v[2],v[3])) end
  -- Check the distance
  if maxD and v[1]*v[1] + v[2]*v[2] > maxD*maxD then
    return debug_ball(string.format("Distance:%.2f>%.2f",math.sqrt(v[1]*v[1] + v[2]*v[2]),maxD) )
  elseif maxH and v[3] > maxH then    
    return string.format("Height:%.2f>%.2f",v[3],maxH) 
  end
  return v
end

local function projectGround(v,targetheight)
  targetheight = targetheight or 0
  local vout = vector.new(v)
  local vHead_homo = vector.new({vHead[1], vHead[2], vHead[3], 1})
  --Project to plane
  if vHead[3]>targetheight and v[3]<targetheight then
    vout = vHead_homo +
      (vout-vHead_homo)*( (vHead[3]-targetheight) / (vHead[3]-vout[3]) )
  end

  return vout
end




function libVision.ball(labelA_t, labelB_t, cc_t)
  
  debug_ball_clear() 

  local balls = {}
  
  balls.v, balls.r, balls.dr, balls.da = {},{},{},{}
  
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
    debug_ball("Checking "..i.." / "..nProps.."\n")
    local check_fail = false
    -- Check the image properties
    local propsB = ballPropsB[i]
    local propsA = check_prop(colors.orange, propsB, b_bbox_area, b_area, b_fill_rate, labelA_t)
    if type(propsA)=='string' then 
      debug_ball(propsA)
      check_fail = true
    else
      -- Check the coordinate on the field

      local dArea = math.sqrt((4/math.pi) * propsA.area)
      local scale = math.max(dArea/b_diameter, propsA.axisMajor/b_diameter)

      local v = check_coordinateA(propsA.centroid, scale, b_dist, b_height0,b_height1,true)

      if type(v)=='string' then 
        check_fail = true
        debug_ball(v)
      else        
--			print(string.format('ball height:%.2f, thr: %.2f', v[3], b_height0+b_height1*math.sqrt(v[1]*v[1]+v[2]*v[2])))
       
        --[[ Field bounds check
        if not check_fail and math.sqrt(v[1]*v[1]+v[2]*v[2])>3 then
					local margin = 0.85 --TODO
          local global_v = util.pose_global({v[1], v[2], 0}, wcm.get_robot_pose())
          if math.abs(global_v[1])>xMax+margin or math.abs(global_v[2])>yMax+margin then
            check_fail = true
            debug_ball('OUTSIDE FIELD!\n')
          end
        end
				--]]

        --[[ Ground check
        if not check_fail and Body.get_head_position()[2] < Config.vision.ball.th_ground_head_pitch then
          local th_ground_boundingbox = Config.vision.ball.th_ground_boundingbox
          local ballCentroid = propsA.centroid
          local vmargin = ha-ballCentroid[2]
          --When robot looks down they may fail to pass the green check
          --So increase the bottom margin threshold
          if vmargin > dArea * 2.0 then
            -- Bounding box in labelA below the ball
            local fieldBBox = {}
            fieldBBox[1] = ballCentroid[1] + th_ground_boundingbox[1]
            fieldBBox[2] = ballCentroid[1] + th_ground_boundingbox[2]
            fieldBBox[3] = ballCentroid[2] + .5*dArea
                       + th_ground_boundingbox[3]
            fieldBBox[4] = ballCentroid[2] + .5*dArea
                        + th_ground_boundingbox[4]
            -- color stats for the bbox
            local fieldBBoxStats = ImageProc.color_stats(labelA_t, colors.field, fieldBBox)
            if (fieldBBoxStats.area < Config.vision.ball.th_ground_green) then
              -- if there is no field under the ball
              -- it may be because its on a white line
              local whiteBBoxStats = ImageProc.color_stats(labelA_t, colors.white,fieldBBox)
              if (whiteBBoxStats.area < Config.vision.ball.th_ground_white) then
                debug_ball("Green check fail\n")
                check_fail = true
              end
            end --end white line check
          end
        end --end bottom margin check
        --]]
        
        
        if not check_fail then
          table.insert(balls.v, projectGround(v,b_diameter/2))
  				-- For ballFilter
          local ball_r = math.sqrt(v[1]*v[1]+v[2]*v[2])
          table.insert(balls.r, ball_r)
          table.insert(balls.dr, ball_r*0.25)
          table.insert(balls.da, 10*math.pi/180)
          
          
          --[[ Project the ball to the ground
          propsA.v = projectGround(v,b_diameter/2)
          propsA.t = Body and Body.get_time() or 0
  				-- For ballFilter
  			  propsA.r = math.sqrt(v[1]*v[1]+v[2]*v[2])
  				propsA.dr = 0.25*propsA.r --TODO: tweak 
  				propsA.da = 10*math.pi/180
          --]]
          
        end
      end

    end -- end of the check on a single propsA
    
    -- Did we succeed in finding two balls?
    if #balls.v==2 then
      local db_str = 'TWO BALLS DETECTED!'
      debug_ball(db_str)
      return db_str, balls
    end
  end  -- end of loop

  -- Assume failure 
  return ball_debug
end



-- Set the variables based on the config file
function libVision.entry(cfg, body)
	-- Set up bias params	
  hcm.set_camera_bias(Config.walk.headBias or {0,0,0})
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
    

	cameraPos = cfg.head.cameraPos or {0,0,0}
  dtrCamera = T.trans(unpack(cameraPos))
  * T.rotY(cameraPitch or 0)
  focalA = focal_length / (focal_base / wa)
  focalB = focalA / scaleB

  local rpy = Body.get_rpy()
  trNeck0 = T.trans(-Config.walk.footX, 0, Config.walk.bodyHeight)
  * T.rotY(Config.vision.bodyTilt)
  * T.trans(cfg.head.neckX, 0, cfg.head.neckZ)
	-- Initial guess
  trNeck = trNeck0 * T.rotZ(0) * T.rotY(0)
  trHead = trNeck * dtrCamera
  -- Load ball
  if cfg.vision.ball then
    b_diameter = cfg.vision.ball.diameter
    b_dist = cfg.vision.ball.max_distance
    b_height0 = cfg.vision.ball.max_height0
    b_height1 = cfg.vision.ball.max_height1
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

t_last = unix.time()

function libVision.update(img)

  t=unix.time()
--  print("vision fps:",1/(t-t_last))
  t_last = t

  -- Update the motion elements
  update_head()
  
  -- Images to labels
  labelA_t = ImageProc2.yuyv_to_label(img, lut_t:data())
  labelB_t = ImageProc2.block_bitor(labelA_t)

  -- Detection System
  -- NOTE: Muse entry each time since on webots, we switch cameras
  -- In camera wizard, we do not switch cameras, so call only once
  local cc_t = ImageProc2.color_count(labelA_t)
  local ball_fails, balls = libVision.ball(labelA_t, labelB_t, cc_t)

  -- Save the detection information
  detected.balls = balls
  
  -- DEBUGGING
  if balls then
    print(unpack(balls.v[1]))
    print(unpack(balls.v[2]))
  end
  
  detected.debug={}
  detected.debug.ball = ball_debug or ' '
  detected.debug.post = post_fails or ' '
  detected.debug.obstacle = obstacle_fails or ' '
  detected.debug.line = line_fails or ' '
  

  -- Send the detected stuff over the channel every cycle
  vision_ch:send(mp.pack(detected))

end


return libVision
