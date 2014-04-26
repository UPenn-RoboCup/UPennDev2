-- libVision
-- (c) 2014 Stephen McGill
-- General Detection methods
local libVision = {}
-- Detection and HeadTransform information
local ImageProc = require'ImageProc'
local T = require'libTransform'
local trHead, trNeck, trNeck0, dtrCamera
-- Camera information
local x0A, y0A, focalA, focal_length, focal_base
-- Object properties
local b_diameter = 0.065

-- Load robot parameters from a Config table
function libVision.load_robot (c_vision)
  -- Placeholder for now
  focal_length, focal_base = 545.6, 640
  b_diameter = 0.065
  -- Delta transform from neck to camera
  dtrCamera = Transform.trans(unpack(c_vision.pCamera))
  * Transform.rotY(c_vision.pitchCamera)
end

-- TODO: Combine entry with load_robot. Take in the config,
-- The config should have the w, h, and scales, or just ma, na, mb, nb...
function libVision.entry (w0, h0, sA, sB)
  -- Save the scale paramter
  scaleA = sA or 2
  scaleB = sB or 2
  -- Recompute the width and height of the images
  w, h = w0, h0
  wa, ha = w/scaleA, h/scaleA
  wb, hb = wa/scaleB, ha/scaleB
  -- Information for the HeadTransform
  x0A, y0A = 0.5 * (wa - 1), 0.5 * (ha - 1)
  focalA = focal_length / (focal_base / wa)
  -- TODO: Do not assume a constant height/tilt/etc.
  -- TODO: Use :mul so no new matrices allocated
  trNeck0 = T.trans(-footX, 0, bodyHeight) 
  * T.rotY(bodyTilt)
  * T.trans(neckX, 0, neckZ)
end

-- Update the Head transform
-- Input: Head angles
function libVision.update (head)
  -- TODO: Smarter memory allocation
  -- NOTE: pitch0 is Robot specific head angle bias (for OP)
  tNeck = tNeck0 * T.rotZ(head[1]) * T.rotY(head[2] + pitch0)
  tHead = tNeck * dtrCamera
  --[[
  --update camera position
  local vHead=vector.new({0,0,0,1});
  vHead=tHead*vHead;
  vHead=vHead/vHead[4];
  --]]
end

-- Simple bbox with no tilted color stats
-- TODO: Use the FFI for color stats, should be super fast
local function bboxStats (color, bboxB)
  local bboxA = {
    scaleB * bboxB[1],
    scaleB * bboxB[2] + scaleB - 1,
    scaleB * bboxB[3],
    scaleB * bboxB[4] + scaleB - 1
  }
  local area = (bboxA[2] - bboxA[1] + 1) * (bboxA[4] - bboxA[3] + 1)
  return ImageProc.color_stats(labelA_t, color, bboxA), area
end

local function check_prop (prop, th_area, th_fill)
  -- Grab the statistics in labelA
  local stats, box_area = bboxStats(1, prop.boundingBox);
  local area = stats.area
  -- If no pixels then return
  if area < th_area then return'Area' end
  -- Get the fill rate
  local fill_rate = area / box_area
  if fill_rate < th_fill then return'Fill rate' end
  return stats
end

local function check_coordinate(centroid, scale, maxD, maxH)
  local v = torch.Tensor({
    focalA,
    -(centroid[1] - x0A),
    -(centroid[2] - y0A),
    scale,
  })
  v = trHead * v / v[4]
  -- Check the distance
  if v[1]*v[1] + v[2]*v[2] > maxD*maxD then
    return"Distance"
  elseif v[3] > maxH then
    return"Height"
  end
  return v
end

function libVision.ball()
  -- The ball is color 1
  local cc = cc_d[1]
  if cc<6 then return'Color count' end
  -- Connect the regions in labelB
  local ballPropsB = ImageProc.connected_regions(labelB_t, 1)
  local nProps = #ballPropsB
  if nProps==0 then return'Connected regions' end
  --
  for i=1,math.min(5,nProps) do
    -- Check the image properties
    local propsB = ballPropsB[i]
    local propsA = check_prop(propsB, 4, 0.35)
    if type(propsA)=='string' then return string.format("Failed %s", propsA) end
    -- Check the coordinate on the field
    local dArea = math.sqrt((4/math.pi) * propsA.area)
    local scale = math.max(dArea/b_diameter, propsA.axisMajor/b_diameter);
    local v = check_coordinate(propsA.centroid, scale, 5.0, 0.20)
    if type(propsA)=='string' then return string.format("Failed %s", v) end
    -- TODO: Check if outside the field
    -- TODO: Ground color check
    return propsA.centroid, v
  end
  --
end

return libVision
