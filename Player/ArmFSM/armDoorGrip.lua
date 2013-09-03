local state = {}
state._NAME = ...
local Config = require'Config'
local Body   = require'Body'
local T      = require'Transform'
local util   = require'util'
require'hcm'

-- Angular velocity limit
local dqArmMax = vector.new({10,10,10,15,45,45})*Body.DEG_TO_RAD

local SHOULDER_Y = .259 -- From the shoulder offset in Kinematics
local GRIP_ROLL  = -90*Body.DEG_TO_RAD
local GRIP_YAW   = 0*Body.DEG_TO_RAD

-- Tune our entry point for gripping
local GRIP_PITCH = -60*Body.DEG_TO_RAD

local turnAngle = 0
local body_pos = {0,0,0}
local body_rpy = {0,0,0}

local t_init = 5.0
local t_grip = 5.0

local handle_pos,handle_pitch,handle_yaw
local handle_radius1,handle_radius0,handle_radius

local update_human = function()
  local handle = hcm.get_door_handle()
  handle_x = handle[1]
  handle_z = handle[3]
end

local function calculate_arm_position()

    local trLArm = vector.new({
    handle_x, -- Free param
    SHOULDER_Y, -- 6 DOF arm cannot go certain places
    handle_z, -- Free param
    GRIP_ROLL, -- Assume a certain orientation
    GRIP_PITCH, -- Tune this, or update based on the handle position
    GRIP_YAW -- Assume a certain orientation
    })

   return trLArm
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  -- Get the human estimate
  update_human()

end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  
  -- Where are we now?
  local qLArm = Body.get_larm_command_position()

  -- Get the human estimate
  update_human()
  
  -- Calculate where we need to go  
  local trLArm = calculate_arm_position()

  -- Get desired angles from current angles and target transform
  local qL_desired = Body.get_inverse_larm(qLArm,trLArm)
  if not qL_desired then
    print('Left not possible',trLArm)
    return'reset'
  end

  -- Go to the allowable position
  local qL_approach, doneL
  qL_approach, doneL = util.approachTol( qLArm, qL_desired, dqArmMax, dt )
  Body.set_larm_command_position( qL_approach )

  -- TODO: Begin to grip by approaching the inner radius
  --[[
    ph = (t-t0)/t_grip
    handle_radius = handle_radius0*(1-ph) + ph*handle_radius1
  --]]

  if doneL then
    -- Close the fingers
    Body.set_lgrip_percent(1)
    return'done'
  end
  
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state