require('dcm')
require('pcm')
require('Config')

--------------------------------------------------------------------------------
-- cop_sensor : estimates CoP from foot poses and wrenches
--------------------------------------------------------------------------------

cop_sensor = {}

local function sign(x)
  return x < 0 and -1 or 1
end

local function compute_foot_cop(foot_wrench)
  -- compute local CoP relative to foot frame
  local w = foot_wrench
  local fx, fy, fz = w[1], w[2], w[3]
  local tx, ty, tz = w[4], w[5], w[6]

  local pressure
  local cop = {}
  if (math.abs(fz) < 1e-10) then
    cop[1] = 0
    cop[2] = 0
    cop[3] = 0
    pressure = sign(fz)*1e-10
  else
    cop[1] = -ty/fz
    cop[2] = tx/fz
    cop[3] = 0
    pressure = fz
  end
  return cop, pressure
end

function cop_sensor.entry()
end

function cop_sensor.update()
  -- compute net CoP relative to base frame
  local l_foot_pose = pcm:get_l_foot_pose()
  local r_foot_pose = pcm:get_r_foot_pose()
  local l_foot_cop, l_foot_pressure = compute_foot_cop(pcm:get_l_foot_wrench())
  local r_foot_cop, r_foot_pressure = compute_foot_cop(pcm:get_r_foot_wrench())
  local pressure = l_foot_pressure + r_foot_pressure

  if (math.abs(pressure) < 1e-10) then
    pressure = sign(pressure)*1e-10
  end

  local cop = {}
  cop[1] = (l_foot_pose[1] + l_foot_cop[1])*l_foot_pressure/pressure
         + (r_foot_pose[1] + r_foot_cop[1])*r_foot_pressure/pressure
  cop[2] = (l_foot_pose[2] + l_foot_cop[2])*l_foot_pressure/pressure
         + (r_foot_pose[2] + r_foot_cop[2])*r_foot_pressure/pressure
  cop[3] = 0

  -- update pcm
  pcm:set_cop(cop)
  pcm:set_cop_pressure(pressure)
  pcm:set_l_foot_cop(l_foot_cop)
  pcm:set_l_foot_cop_pressure(l_foot_cop_pressure)
  pcm:set_r_foot_cop(r_foot_cop)
  pcm:set_r_foot_cop_pressure(r_foot_cop_pressure)
end

function cop_sensor.exit()
end

return cop_sensor
