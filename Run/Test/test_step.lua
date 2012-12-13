dofile('../include.lua')

require('Body')
require('trajectory')
require('Transform')
require('Kinematics')
require('vector')
require('dcm')

Body.entry()

-- initialize step parameters (coordinates relative to l_foot frame)
----------------------------------------------------------------------

local step_duration = 1.5
local step_size = 0.4
local step_height = 0.02

local x_offset = 0 
local y_offset = -0.097
local z_offset = 0.79

local x_torso_swing = 0
local y_torso_swing = 0
local z_torso_swing = 0
local x_torso_swing_momentum = 0
local y_torso_swing_momentum = 0
local z_torso_swing_momentum = 0

local torso_midpoint_ratio = 0.5
local swing_midpoint_ratio = 0.5

-- start conditions
local swing_start_position = {
  -step_size/2,
  2*y_offset,
  0,
}
local torso_start_position = {
  x_offset - step_size/4,
  y_offset,
  z_offset,
}
local swing_start_velocity = {
  0,
  0,
  0,
}
local torso_start_velocity = {
  0,
  0,
  0,
}

-- midpoint conditions
local torso_midpoint_t = torso_midpoint_ratio*step_duration
local swing_midpoint_t = swing_midpoint_ratio*step_duration
local torso_midpoint_position = {
  x_offset + x_torso_swing,
  y_offset + y_torso_swing,
  z_offset + z_torso_swing,
}
local swing_midpoint_position = {
  0,
  2*y_offset,
  step_height,
} 
local torso_midpoint_velocity = {
  x_torso_swing_momentum,
  y_torso_swing_momentum,
  z_torso_swing_momentum,
}
local swing_midpoint_velocity = {
  step_size/step_duration,
  0,
  0,
}

-- goal conditions
local swing_goal_t = step_duration
local torso_goal_t = step_duration
local swing_goal_position = {
  step_size/2,
  2*y_offset,
  0,
}
local torso_goal_position = {
  x_offset + step_size/4,
  y_offset,
  z_offset,
}
local swing_goal_velocity = {
  0, 
  0, 
  0, 
}
local torso_goal_velocity = {
  0,
  0,
  0,
}

-- initialize controller parameters
----------------------------------------------------------------------

-- transform left and right foot coordinates into torso base frame 
local p_l_foot = Transform.new({0, 0, 0})
local p_r_foot = Transform.new(swing_start_position)
local p_torso = Transform.new(torso_start_position)
p_l_foot = p_torso:inv()
p_r_foot = p_l_foot*p_r_foot

local t0 = Body.get_time()
local q0 = dcm:get_joint_position('legs');
local qstance = Kinematics.inverse_pos_legs(p_l_foot, p_r_foot)

-- define step states
----------------------------------------------------------------------

function stance_update()
  -- caclutate stance configuration
  local t = Body.get_time()
  if (t - t0 < 3) then
    local d = util.min{(t - t0)/3, 1}
    local q = q0 + d*(vector.new(qstance) - vector.new(q0))
    dcm:set_joint_position(q, 'legs')
    Body.update()
    return true
  end
end

function step_update()
  -- calculate step configuration
end

while (true) do
  stance_update()
  Body.update()
end
