local Body = require'Body'
-- TODO: Get the shm desired endpoints
-- TODO: Should these endpoints be in vcm?  Probably...
require'vcm'

local lidarPan = {}
lidarPan._NAME = 'lidarPan'

-- Timings
local t_entry = 0 -- When the state was last entered
local t_update = 0 -- When the state was last updated

-- Set up the default min and max pan angles
local min_pan = Body.servo.min_rad[Body.indexLidar]
local max_pan = Body.servo.max_rad[Body.indexLidar]
-- Save for shared memory to use
vcm.set_chest_lidar_endpoints({min_pan,max_pan})
local mid_pan = 0
local mag_pan = max_pan - min_pan

-- Set up the pan timing (can reaange order based on needs)
local ph_duration = 10 -- Number of seconds per pan
local rad_speed = mag_pan / ph_duration -- radians per second
local ph_speed = 1/ph_duration -- radians per second

-- Assume a starting phase of mid-way
local ph = .5
-- Assume that we are going forward to start
-- Dir: forward is true, backward is false
local forward = true

-- Take a phase of the panning scan and return the angle to set the lidar
-- ph is between 0 and 1
local function ph_to_radians( ph, forward )
  -- Clamp the phase to avoid crazy behavior
  ph = math.max( math.min(ph, 1), 0 )
  
  if forward then
    -- Forward direction
    return min_pan + ph*mag_pan
  else
    -- Backwards direction
    return max_pan - ph*mag_pan
  end
end

-- Take a given radian and back convert to find the durrent phase
-- Direction is the side of the mid point, as a boolean (forward is true)
local function radians_to_ph( rad, forward )
  rad = math.max( math.min(rad, max_pan), min_pan )
  return ( rad - min_pan ) / mag_pan, (forward or rad>mid_pan)
end

function lidarPan.entry()
  print(lidarPan._NAME..' Entry' ) 

  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  
  -- Ascertain the phase, from the current position of the lidar
  local cur_angle = Body.get_lidar_position(1)
  ph, forward = radians_to_ph( cur_angle )
end

function lidarPan.update()
  --print(lidarPan._NAME..' Update' ) 

  -- Get the time of entry
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update timeå
  t_update = t
  
  -- Update the direction
  forward = (forward and ph<1) or ph<0
  -- Update the phase of the pan
  if forward then
    ph = ph + t_diff * ph_speed
  else
    ph = ph - t_diff * ph_speed
  end

  -- Set the desired angle of the lidar pan
  local rad = ph_to_radians(ph)
  Body.set_lidar_command_position( {rad} )
end

function lidarPan.exit()
  print(lidarPan._NAME..' Exit' )
end

return lidarPan