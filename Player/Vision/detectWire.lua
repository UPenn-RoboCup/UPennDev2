local torch = require'torch'
local ImageProc2 = require'ImageProc.ffi'
local util = require'util'
local vector = require'vector'
local Body = require'Body'
local K = Body.Kinematics
local T = require'libTransform'
local sin, cos = math.sin, math.cos
local w, h, focal_length

local edge_ch = si.new_publisher('edge')
local tou_che = si.new_subscriber('touche')
local tou_che2 = si.new_subscriber(55588)
local line_ch = si.new_publisher('line')
local line_ch_remote = si.new_publisher(55589,'25.25.1.109')

-- Variables
local kernel_t, use_horiz, use_vert = ImageProc2.dir_to_kernel('v'), false, true
util.ptorch(kernel_t)
-- Form the default bounding box (in scaled down space...)
local bbox = {51, 101, 21, 111}
--local bbox = {101, 201, 41, 221}

local last_measurement
local function update_dist (pline1, pline2, tr, t, line_radon)
  -- Scale up
  local px_width = 2 * math.abs(line_radon.ir1 - line_radon.ir2)

  -- Assume in the center
  local angle_width = math.atan(px_width / 2 / focal_length)

  --print('angle_width',angle_width*RAD_TO_DEG)
  -- See if this is our first mesaurment
  if not last_measurement then
    print("UPDATE", T.get_pos(tr))
    last_measurement = {
      px_width = px_width,
      tr = tr,
      angle_width = angle_width,
      t = t,
    }
    return
  end

  -- If the less than a few pixels difference in width, then return
  local px_diff = px_width - last_measurement.px_width

  if px_diff < 2.5 then
    --print('FAILED px_diff', px_diff, px_width)
    return
  end
  -- Check the distance between transforms
  local p_last, p_now = T.get_pos(last_measurement.tr), T.get_pos(tr)
  local p_diff = vector.norm(p_last - p_now)

  --util.ptorch(tr)
  --print('p_diff', p_diff)

  -- If less than an inch, discard
  if p_diff < 0.050 then
    --print('FAILED p_diff', p_diff)
    return
  else
    --print('SUCCESS p_diff', p_diff)
    --print('angle_width', RAD_TO_DEG*angle_width, RAD_TO_DEG*last_measurement.angle_width)
  end

  local angle_diff = angle_width - last_measurement.angle_width
  if angle_diff<0 then return end
  --if math.abs(angle_diff)<1*DEG_TO_RAD then print('FAILED angle_diff', angle_diff); return; end

  local sin_diff = sin(angle_width - last_measurement.angle_width)
  --print('sin_diff',sin_diff)
  local s_a = sin(math.pi - angle_width)
  local r = (s_a * sin(last_measurement.angle_width)) / sin_diff * p_diff
  local d = (s_a * cos(last_measurement.angle_width)) / sin_diff * p_diff
  -- Update the last_measurment
  print("\nr, d", r, d)
  print('p_diff', p_diff)
  --print('a_diff', RAD_TO_DEG*angle_width, RAD_TO_DEG*last_measurement.angle_width)
  --print('p_diff', p_diff - p_diff0, d - d0)
  print()
  -- Return the distance measurement
  return r, d
end

-- Updating stuff
local function update_bbox ()
  --local bbox_data = tou_che:receive(true)
  local bbox_data = tou_che2:receive(true)
  if not bbox_data then return end
  -- Evaluate all bbox change requests
  for _, bbox_request in ipairs(bbox_data) do
    local bb = mp.unpack(bbox_request)
    --
    local dir = bb.dir
    kernel_t = ImageProc2.dir_to_kernel(dir)
    --util.ptorch(kernel_t)
    if dir=='v' then
      use_horiz, use_vert = false, true
    elseif dir=='h' then
        use_horiz, use_vert = true, false
    else
        use_horiz, use_vert = true, true
    end
    --
    bbox = vector.new(bb.bbox) / 2
    for i,v in ipairs(bbox) do bbox[i] = math.ceil(v) end
    --print('BBOX', bbox, dir)
  end
end


local function send ()
  local mmm = mp.pack({
    name = 'pline',
    l1 = {
      {x=pline1.iMin,  y=pline1.jMin},
      {x=pline1.iMean, y=pline1.jMean},
      {x=pline1.iMax,  y=pline1.jMax},
      },
    l2 = {
      {x=pline2.iMin,  y=pline2.jMin},
      {x=pline2.iMean, y=pline2.jMean},
      {x=pline2.iMax,  y=pline2.jMax},
      },
    -- Relative placement
    bbox = bbox,
    })


  line_ch:send(mmm)
  line_ch_remote:send(mmm)

  -- Send to the human user
  --[[
  local c_img = c_grey:compress(grey_bt)
  local udp_ret, err = udp_ch:send( mp.pack(meta)..c_img )
  --print('udp img',img,sz,cnt,t,udp_ret)
  if err then print(name,'udp error',err) end
  --]]

end


local detectWire = {}

function detectWire.entry (metadata)
  w, h = metadata.w, metadata.h
  focal_length = metadata.focal_length
  ImageProc2.setup(w, h, 2, 2)
end

function detectWire.update (img)

  -- Check if their is a new bounding box to use
  update_bbox()

  -- Process line stuff
  local edge_t, grey_t, grey_bt = ImageProc2.yuyv_to_edge(img, bbox, false, kernel_t)
  local RT = ImageProc2.radon_lines(edge_t, use_horiz, use_vert)
  local pline1, pline2, line_radon = RT.get_parallel_lines()
  if not pline1 then return end

  -- massage from bbox region to image region
  pline1.iMin  = 2 * (pline1.iMin  + bbox[1])
  pline1.iMean = 2 * (pline1.iMean + bbox[1])
  pline1.iMax  = 2 * (pline1.iMax  + bbox[1])
  pline1.jMin  = 2 * (pline1.jMin  + bbox[3])
  pline1.jMean = 2 * (pline1.jMean + bbox[3])
  pline1.jMax  = 2 * (pline1.jMax  + bbox[3])
  --
  pline2.iMin  = 2 * (pline2.iMin  + bbox[1])
  pline2.iMean = 2 * (pline2.iMean + bbox[1])
  pline2.iMax  = 2 * (pline2.iMax  + bbox[1])
  pline2.jMin  = 2 * (pline2.jMin  + bbox[3])
  pline2.jMean = 2 * (pline2.jMean + bbox[3])
  pline2.jMax  = 2 * (pline2.jMax  + bbox[3])

  -- Find the angles for servoing
  local camera_roll = line_radon.ith / line_radon.NTH * math.pi
  camera_roll = camera_roll > (math.pi / 2) and (camera_roll - math.pi) or camera_roll
  -- Place iMean in the center of the frame horizontally
  -- Remember, we massaged plines to be in the original resolution
  local i_px = (pline1.iMean + pline2.iMean) / 2 - (w / 2)
  local j_px = (pline1.jMean + pline2.jMean) / 2 - (h / 2)
  local camera_yaw = math.atan(i_px / focal_length)
  local camera_pitch = math.atan(j_px / focal_length)
  -- Set in sahred memory
  vcm.set_wire_cam_rpy{camera_roll,camera_pitch,camera_yaw}
  -- Update the distance to the wire and the wire's radius
  update_dist(pline1, pline2, fk, t, line_radon)
end

function detectWire.exit ()

end

return detectWire
