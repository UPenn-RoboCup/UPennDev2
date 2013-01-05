require('dcm')
require('Config')

---------------------------------------------------------------------------
-- cop_zmp : utilities for CoP / ZMP estimation
---------------------------------------------------------------------------

cop_zmp = {}

local l_foot_cop_transform = Config.mechanics.l_foot.force_torque_transform
local r_foot_cop_transform = Config.mechanics.r_foot.force_torque_transform
local l_foot_cop_offset = l_foot_cop_transform:get_pose6D()
local r_foot_cop_offset = r_foot_cop_transform:get_pose6D()

local function sign(x)
  return x < 0 and -1 or 1
end

function cop_zmp.estimate_l_foot_cop()
  -- returns CoP relative to left foot frame
  -- (considering ground reaction forces acting on left foot only)
  local ft = dcm:get_force_torque('l_foot')
  local pressure = ft[3]

  if (math.abs(pressure) < 1e-10) then
    pressure = sign(pressure)*1e-10
  end

  local cop = {}
  cop[1] = -ft[5]/pressure + l_foot_cop_offset[1]
  cop[2] = ft[4]/pressure + l_foot_cop_offset[2]
  cop[3] = 0
  return cop, pressure
end

function cop_zmp.estimate_r_foot_cop()
  -- returns CoP relative to right foot frame
  -- (considering ground reaction forces acting on right foot only)
  local ft = dcm:get_force_torque('r_foot')
  local pressure = ft[3]

  if (math.abs(pressure) < 1e-10) then
    pressure = sign(pressure)*1e-10
  end

  local cop = {}
  cop[1] = -ft[5]/pressure + r_foot_cop_offset[1]
  cop[2] = ft[4]/pressure + r_foot_cop_offset[2]
  cop[3] = 0
  return cop, pressure
end

function cop_zmp.estimate_cop(l_foot_position, r_foot_position)
  -- returns CoP relative to torso frame
  -- (considering ground reaction forces acting on left and right foot)
  local l_foot_cop, l_foot_pressure = cop_zmp.estimate_l_foot_cop()
  local r_foot_cop, r_foot_pressure = cop_zmp.estimate_r_foot_cop()
  local pressure = l_foot_pressure + r_foot_pressure

  if (math.abs(pressure) < 1e-10) then
    pressure = sign(pressure)*1e-10
  end

  local cop = {}
  cop[1] = (l_foot_position[1] + l_foot_cop[1])*l_foot_pressure/pressure
         + (r_foot_position[1] + r_foot_cop[1])*r_foot_pressure/pressure
  cop[2] = (l_foot_position[2] + l_foot_cop[2])*l_foot_pressure/pressure
         + (r_foot_position[2] + r_foot_cop[2])*r_foot_pressure/pressure
  cop[3] = 0
  return cop, pressure
end

return cop_zmp
