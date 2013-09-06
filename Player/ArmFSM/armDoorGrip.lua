local state = {}
state._NAME = ...
local Body  = require'Body'
local K     = Body.Kinematics
local T     = require'Transform'
local util  = require'util'
local vector = require'vector'
require'hcm'

-- Angular velocity limit
local dqArmMax = Config.arm.slow_limit

--local SHOULDER_Y = .259 -- From the shoulder offset in Kinematics
local GRIP_ROLL  = -90*Body.DEG_TO_RAD
local GRIP_YAW   = 0*Body.DEG_TO_RAD

-- Tune our entry point for gripping
local GRIP_PITCH = 0 -- -20*Body.DEG_TO_RAD -- 0 -- -60*Body.DEG_TO_RAD

local turnAngle = 0
local body_pos = {0,0,0}
local body_rpy = {0,0,0}

local t_init = 5.0
local t_grip = 5.0

local function undercut()
end

local grip_mode = {
  [1] = undercut,
  [2] = sj
}

local handle, handle_x, handle_z, door_arm
local update_human = function()
  handle = hcm.get_door_handle()
  handle_x = handle[1]
  handle_z = handle[3]
  -- Decide which arm to use
  if handle[2]>0 then
    door_arm = 'left'
  else
    door_arm = 'right'
  end
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  -- Get the human estimate
  update_human()

  print('Grab door with', door_arm)
  if door_arm=='right' then
    Body.set_rgrip_percent(.5)
  else
    Body.set_lgrip_percent(.5)
  end

end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local  t = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  
  -- Where are we now?
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  -- Update the human estimate
  update_human()

  -- Sanity checks
  if handle_x<.3 then return'reset' end
  if math.abs(handle_z)>.3 then return'reset' end

  -- Calculate where we need to go  
  local trArm = vector.new({
    handle_x, -- Free param
    K.shoulderOffsetY, -- 6 DOF arm cannot go certain places
    handle_z, -- Free param
    GRIP_ROLL, -- Assume a certain orientation
    GRIP_PITCH, -- Tune this, or update based on the handle position
    GRIP_YAW -- Assume a certain orientation
  })
  -- Side adjustment
  if door_arm=='right' then
    -- y direction swapped
    trArm[2] = -trArm[2]
    -- roll the other way
    trArm[4] = -trArm[4]
  end

  -- Get desired angles from current angles and target transform
  local q_desired
  if door_arm=='right' then
    q_desired = Body.get_inverse_rarm(qRArm,trArm)
  else
    q_desired = Body.get_inverse_larm(qLArm,trArm)
  end
  
  -- Double check reachability
  if not q_desired then
    print('Door grip not possible',trArm)
    return'reset'
  end
  
  -- Safety check for the joints
  q_desired[2] = 0
  q_desired[3] = 0
  
  print('q_desired',vector.new(q_desired)*180/math.pi)

  -- Go to the allowable position
  local qL_approach, qR_approach, done
  if door_arm=='right' then
    -- more safety
    q_desired[5] = 90*Body.DEG_TO_RAD
    qR_approach, done = util.approachTol( qRArm, q_desired, dqArmMax, dt )
    Body.set_rarm_command_position( qR_approach )
  else
    q_desired[5] = -90*Body.DEG_TO_RAD
    qL_approach, done = util.approachTol( qLArm, q_desired, dqArmMax, dt )
    Body.set_larm_command_position( qL_approach )
  end

  -- TODO: Begin to grip by approaching the inner radius
  --[[
    ph = (t-t0)/t_grip
    handle_radius = handle_radius0*(1-ph) + ph*handle_radius1
  --]]

  if done then
    -- Close the fingers
    if door_arm=='right' then
      Body.set_rgrip_percent(1)
    else
      Body.set_lgrip_percent(1)
    end
    return'done'
  end
  
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state