local torch = require'torch'
local ImageProc2 = require'ImageProc.ffi'
local util = require'util'
local vector = require'vector'
local Body = require'Body'
local K = Body.Kinematics
local T = require'libTransform'
local si = require'simple_ipc'
local mp = require'msgpack.MessagePack'
local sin, cos = math.sin, math.cos
local w, h, focal_length, bbox
local bbox_ch, wire_ch
local kernel_t, use_horiz, use_vert
local bb_angle = nil -- no prior
require"vcm"
require"gcm"

local DEBUG, radon_ch = true
if DEBUG then
  radon_ch = si.new_publisher('radon')
end

local last_measurement
local function update_dist(pline1, pline2, line_radon)

  if gcm.get_fsm_Arm()~='armWireApproach' then return end

  -- Scale up
  local px_width = 2 * math.abs(line_radon.ir1 - line_radon.ir2)

  -- Assume in the center
  local angle_width = math.atan(px_width / 2 / focal_length)

  -- Get the arm position
  local tr = K.forward_arm(Body.get_larm_position())

  --print('angle_width',angle_width*RAD_TO_DEG)
  -- See if this is our first mesaurment
  if not last_measurement then
    --print("UPDATE", T.get_pos(tr))
    last_measurement = {
      px_width = px_width,
      tr = tr,
      angle_width = angle_width,
      t = Body.get_time(),
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
  --print("\nr, d", r, d, d-p_diff)
  --print('p_diff', p_diff)
  --print('a_diff', RAD_TO_DEG*angle_width, RAD_TO_DEG*last_measurement.angle_width)
  --print('p_diff', p_diff - p_diff0, d - d0)
  --print()

  -- Set the distance and radius of the object
  vcm.set_wire_model{r, d - p_diff, Body.get_time() - last_measurement.t}

  -- Return the distance measurement
  return r, d
end

-- Updating stuff
local function update_bbox()
  local bbox_data = bbox_ch:receive(true)
  if not bbox_data then
    -- Use the SHM value
    --bbox = vcm.get_wire_bbox()
  else
    local bb = mp.unpack(bbox_data[#bbox_data])
    bb_angle = bb.a > 0 and bb.a - math.pi/2 or bb.a + math.pi/2
    if DEBUG then
      --print('NEW BB')
      --util.ptable(bb)
      -- Format for the RT coordinates
      print('BB ANGLE', bb_angle * RAD_TO_DEG)
    end
    if bb.id=='bbox' then
      local dir = bb.dir
      kernel_t = ImageProc2.dir_to_kernel(dir)
      use_horiz, use_vert = true, true
      if dir=='v' then use_horiz=false elseif dir=='h' then use_vert=false end
      bbox = vector.new(bb.bbox) / 2
    end
  end
  bbox[1] = math.max(1, math.ceil(bbox[1]))
  bbox[2] = math.min(w/2, math.ceil(bbox[2]))
  bbox[3] = math.max(1, math.ceil(bbox[3]))
  bbox[4] = math.min(h/2, math.ceil(bbox[4]))

  if DEBUG then print('update_bbox', bbox) end

  -- Set into shm
  vcm.set_wire_bbox(bbox)
end


local function send (pline1, pline2, bbox)
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
  wire_ch:send(mmm)
end


local detectWire = {}

function detectWire.entry(metadata)
  w, h = metadata.width, metadata.height
  focal_length = metadata.focal_length
  ImageProc2.setup(w, h, 2, 2)
  bbox_ch = si.new_subscriber(Config.human.touch.bbox_ch)
  wire_ch = si.new_publisher(Config.vision.wire.ch)
  -- Default is the whole image (scale down of 2)
  bbox = vector.new{1, w/2, 1, h/2}
  vcm.set_wire_bbox(bbox)
  --
  kernel_t, use_horiz, use_vert = ImageProc2.dir_to_kernel(), true, true
end

function detectWire.update(img)
	if not img then print("NO IMAGE"); return end

  local arm_state = gcm.get_fsm_Arm()
  if not (arm_state=='armWireLook' or arm_state=='armWireApproach') then
    last_measurement = nil
    vcm.set_wire_model{0,0,0}
    bbox = vector.new{1, w/2, 1, h/2}
    vcm.set_wire_bbox(bbox)
    -- Reset prior
    bb_angle = nil
    return
  end

  if DEBUG then
    print('\nUpdate')
    print('KERNEL', kernel_t:size(1), kernel_t:size(2))
  end

  -- Check if their is a new bounding box to use
  update_bbox()

  -- Process line stuff
  local edge_t, grey_t, grey_bt = ImageProc2.yuyv_to_edge(img, bbox, true, kernel_t)
  -- Refine due to the convolution with the kernel
  local reduced_x, reduced_y = (kernel_t:size(1)-1) / 2, (kernel_t:size(2)-1)/2
  local bbox2 = vector.new(bbox) - vector.new{reduced_x, 0, reduced_y, 0}
  bbox2[1] = math.max(bbox2[1], 1)
  bbox2[3] = math.max(bbox2[3], 1)
  bbox2[2] = math.min(bbox2[2], w / 2 - 2 * reduced_x)
  bbox2[4] = math.min(bbox2[4], h / 2 - 2 * reduced_y)
  -- Give the angle prior
  local rt_props, pline1, pline2, line_radon =
    ImageProc2.parallel_lines(edge_t, use_horiz, use_vert, bbox2, nil, bb_angle)
  -- Send to MATLAB
  if DEBUG then
    print('BBOX', bbox)
    local counts_str = ffi.string(rt_props.count_d, rt_props.MAXR * rt_props.NTH * ffi.sizeof'int32_t')
    local meta = {}
    for k,v in pairs(rt_props) do
      if type(v)~='cdata' and type(v)~='userdata' then meta[k] = v end
    end
    local ret = radon_ch:send({mp.pack(meta), counts_str})
  end

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
  local camera_roll = line_radon.ith_true / line_radon.NTH * math.pi
  camera_roll = camera_roll > (math.pi / 2) and (camera_roll - math.pi) or camera_roll
  --camera_roll = -camera_roll
  -- Place iMean in the center of the frame horizontally
  -- Remember, we massaged plines to be in the original resolution
  local i_px = (pline1.iMean + pline2.iMean) / 2 - (w / 2)
  local j_px = (pline1.jMean + pline2.jMean) / 2 - (h / 2)
  local camera_yaw = -math.atan(i_px / focal_length)
  local camera_pitch = math.atan(j_px / focal_length)
  -- Set in shared memory
  vcm.set_wire_t(Body.get_time())
  vcm.set_wire_cam_rpy{camera_roll, camera_pitch, camera_yaw}

  -- Only update the distance measurements in the approach state
  update_dist(pline1, pline2, line_radon)

  -- Update the bounding box on the line found?

  -- TODO: Only if vertical line, else a horiz should change the j
  if DEBUG then
    print('pline1')
    util.ptable(pline1)
    print('pline2')
    util.ptable(pline2)
  end
  ----[[
  local wbuf = 12
  bbox[1] = math.min(pline1.iMin/2-wbuf, pline2.iMin/2-wbuf, pline1.iMax/2-wbuf, pline2.iMax/2-wbuf)
  bbox[2] = math.max(pline1.iMax/2+wbuf, pline2.iMax/2+wbuf, pline1.iMin/2+wbuf, pline2.iMin/2+wbuf)
  --]]

  --[[
  print('line_radon', line_radon)
  --util.ptable(line_radon)
  util.ptable(pline1)
  print()
  util.ptable(pline2)
  print()
  --]]
  send(pline1, pline2, bbox)
end

function detectWire.exit ()

end

return detectWire
