local Body = require'Body'
require'vcm'

local min_pan, max_pan, mid_pan, mag_pan
local t_entry, t_update, ph_speed, ph, forward

local lidarPan = {}
lidarPan._NAME = 'lidarPan'

-- Update the parameters
local function update_pan_params()
  -- Set up the pan boundaries
  local e = vcm.get_chest_lidar_scanlines()
  min_pan = e[1]
  max_pan = e[2]
  mid_pan = (max_pan + min_pan) / 2
  mag_pan = max_pan - min_pan
  -- Grab the desired resolution (number of columns)
  local res = math.abs(e[3]*(max_pan-min_pan))
  -- Complete the scan at this rate
  ph_speed = 40 / math.ceil(res) -- could be a little faster than 40 Hz
end

-- Take a given radian and back convert to find the current phase
-- Direction is the side of the mid point, as a boolean (forward is true)
-- Dir: forward is true, backward is false
local function radians_to_ph( rad )
  rad = math.max( math.min(rad, max_pan), min_pan )
  return ( rad - min_pan ) / mag_pan, rad>mid_pan
end

function lidarPan.entry()
  print(lidarPan._NAME..' Entry' ) 

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  -- Grab the updated pan paramters
  update_pan_params()
  
  -- Ascertain the phase, from the current position of the lidar
  local cur_angle = Body.get_lidar_position(1)
  ph, forward = radians_to_ph( cur_angle )
end

function lidarPan.update()
  --print(lidarPan._NAME..' Update' )

  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t
  
  -- Grab the updated pan paramters
  update_pan_params()
  
  -- Update the direction
  forward = (forward and ph<1) or ph<=0
  -- Update the phase of the pan
  if forward then
    ph = ph + t_diff * ph_speed
  else
    ph = ph - t_diff * ph_speed
  end
  -- Clamp the phase to avoid crazy behavior
  ph = math.max( math.min(ph, 1), 0 )
  -- Set the desired angle of the lidar tilt
  local rad = min_pan + ph*mag_pan
  -- Request the position of the motor
  --Body.request_lidar_position()
  Body.set_lidar_command_position( {rad} )
end

function lidarPan.exit()
  print(lidarPan._NAME..' Exit' )
end

return lidarPan
