-- libVision
-- (c) 2014 Stephen McGill
-- General Detection methods
local libVision = {}
-- Detection and HeadTransform information
local ImageProc = require'ImageProc'
local ImageProc2 = require'ImageProc.ffi'
local T = require'libTransform'
local vector = require'vector'
local util = require'util'
local zlib = require'zlib.ffi'
local si = require'simple_ipc'
-- Body should be optional...
local Body
-- Important local variables
local w, h, wa, wb, ha, hab, scaleA, scaleB, lut_t
-- Hold on to these
local labelA_t, labelB_t
-- Head transform
local trHead, trNeck, trNeck0, dtrCamera
-- Camera information
local x0A, y0A, focalA, focal_length, focal_base
-- Object properties
local b_diameter, b_dist, b_height, b_fill_rate, b_area
local g_ared, g_fill_rate, g_orientation
-- Store information about what was detected
local detected = {
  id = 'detect',
}
--
local colors
local c_zlib = zlib.compress_cdata
local vision_ch = si.new_publisher'vision'

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
	-- Get from Body...
  local head = Body and Body.get_head_position() or vector.zeros(2)
  -- TODO: Smarter memory allocation
  -- TODO: Add any bias for each robot
  trNeck = trNeck0 * T.rotZ(head[1]) * T.rotY(head[2])
  trHead = trNeck * dtrCamera
  -- Grab the position only
  --vHead = T.get_pos(trHead)
  -- Horizon
  --[[
  local pa = headAngles[2] + cameraAngle[2] -- + bodyTilt   --TODO
  horizonA = (labelA.n/2.0) - focalA*math.tan(pa) - 2
  horizonA = math.min(labelA.n, math.max(math.floor(horizonA), 0))
  horizonB = (labelB.n/2.0) - focalB*math.tan(pa) - 1
  horizonB = math.min(labelB.n, math.max(math.floor(horizonB), 0))
  --]]
end

-- Simple bbox with no tilted color stats
-- TODO: Use the FFI for color stats, should be super fast
local function bboxStats(color, bboxB, labelA_t)
  local bboxA = {
    scaleB * bboxB[1],
    scaleB * bboxB[2] + scaleB - 1,
    scaleB * bboxB[3],
    scaleB * bboxB[4] + scaleB - 1
  }
  local area = (bboxA[2] - bboxA[1] + 1) * (bboxA[4] - bboxA[3] + 1)
	local stats = ImageProc.color_stats(labelA_t, color, bboxA)
  return stats, area
end

local function check_prop(color, prop, th_area, th_fill, labelA_t)
  -- Grab the statistics in labelA
  local stats, box_area = bboxStats(color, prop.boundingBox, labelA_t)
  if box_area < th_area then return'Box Area' end
  local area = stats.area
  -- If no pixels then return
  if area < th_area then return'Area' end
  -- Get the fill rate
  local fill_rate = area / box_area
  if fill_rate < th_fill then return'Fill rate' end
  return stats
end

-- Yield coordinates in the labelA space
-- Returns an error message if max limits are given
local function check_coordinateA(centroid, scale, maxD, maxH)
  local v = torch.Tensor({
    focalA,
    -(centroid[1] - x0A),
    -(centroid[2] - y0A),
    scale,
  })
  v = trHead * v / v[4]
  -- Check the distance
  if maxD and v[1]*v[1] + v[2]*v[2] > maxD*maxD then
    return'Distance'
  elseif maxH and v[3] > maxH then
    return'Height'
  end
  return v
end

function libVision.ball(labelA_t, labelB_t, cc_t)
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
    local propsA = check_prop(colors.orange, propsB, b_area, b_fill_rate, labelA_t)
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
        -- Found the ball position
        propsA.v = vector.new(v)
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
  local postB = ImageProc.goal_posts(labelB_t, colors.yellow, th_nPostB)
  if not postB then return'None detected' end
  -- Now process each goal post
  -- Store failures in the array
  local failures, successes = {}, {}
  for i, post in ipairs(postB) do
    local fail, has_stats = {}, true
    local postStats = check_prop(colors.yellow, post, g_area, g_fill_rate, labelA_t)
    if type(postStats)=='string' then
      table.insert(fail, postStats)
    else
      -- TODO: Add lower goal post bbox check
      -- Orientation check
      if math.abs(postStats.orientation) < g_orientation then
        table.insert(fail, 'Orientation '..postStats.orientation)
      end
      -- Fill rate check
      local fill_rate = postStats.area / (postStats.axisMajor * postStats.axisMinor)
      if fill_rate < g_fill_rate then
        table.insert(fail, 'Fill Extent '..fill_rate)
      end
      -- Aspect Ratio check
      local aspect = postStats.axisMajor / postStats.axisMinor;
      if (aspect < .5) or (aspect > 15) then
        table.insert(fail, 'Aspect Ratio '..aspect)
      end
      -- Edge Margin
      local leftPoint= postStats.centroid[1] - postStats.axisMinor / 2
      local rightPoint= postStats.centroid[1] + postStats.axisMinor / 2
      local margin = math.min(leftPoint, wa - rightPoint)
      if margin <= 5 then
        table.insert(fail, 'Edge Margin '..margin)
      end
    end
    -- TODO: Add ground check
    -- TODO: Add height check
    if #fail==0 then
      table.insert(successes, postStats)
      table.insert(failures, 'SUCCESS')
			--util.ptable(postStats)
    else
      table.insert(failures, table.concat(fail,', '))
    end
  end
  -- Yield the failure messages and the success tables
  if #successes<1 then successes = nil end
  return table.concat(failures,','), successes
end

-- Set the variables based on the config file
function libVision.entry(cfg, body)
  -- Dynamically load the body
  Body = body
  -- Recompute the width and height of the images
  w, h = cfg.w, cfg.h
  -- Set up ImageProc
  ImageProc2.setup(w, h)
  focal_length, focal_base = cfg.focal_length, cfg.focal_base

  -- Save the scale paramter
  scaleA, scaleB = cfg.vision.scaleA, cfg.vision.scaleB
  colors = cfg.vision.colors

  wa, ha = w / scaleA, h / scaleA
  wb, hb = wa / scaleB, ha / scaleB
  -- Information for the HeadTransform
  x0A, y0A = 0.5 * (wa - 1), 0.5 * (ha - 1)
  -- Delta transform from neck to camera
  dtrCamera = T.trans(unpack(cfg.head.cameraPos or {0,0,0}))
  * T.rotY(cfg.head.pitchCamera or 0)
  focalA = focal_length / (focal_base / wa)
  -- TODO: get from shm maybe?
  trNeck0 = T.trans(-Config.walk.footX, 0, Config.walk.bodyHeight)
  * T.rotY(Config.walk.bodyTilt)
  * T.trans(cfg.head.neckX, 0, cfg.head.neckZ)
  -- Load ball
  if cfg.vision.ball then
    b_diameter = cfg.vision.ball.diameter
    b_dist = cfg.vision.ball.max_distance
    b_height = cfg.vision.ball.max_height
    b_fill_rate = cfg.vision.ball.th_min_fill_rate
    b_area = cfg.vision.ball.th_min_ared
  end
  -- Goal thresholds
  if cfg.vision.goal then
    g_area = cfg.vision.goal.th_min_area
    g_fill_rate = cfg.vision.goal.th_min_fill_rate
    g_orientation = cfg.vision.goal.th_min_orientation
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

  -- Save the detection information
  detected.ball = ball
  detected.posts = posts
  detected.debug = table.concat({'Ball',ball_fails,'Posts',post_fails},'\n')
  --if detected.posts then util.ptable(detected.posts) end

  -- Send the detected stuff over the channel every cycle
  vision_ch:send(mp.pack(detected))

end

return libVision
