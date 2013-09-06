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

local dt, qLArm, qRArm

-- Human data
local handle, handle_x, handle_z, door_arm

-- Stages
local stage = 1

local function scrunch()
    -- Calculate where we need to go  
  local trArm = vector.new({
    0, -- Free param
    K.shoulderOffsetY, -- 6 DOF arm cannot go certain places
    math.min(-0.15,handle_z), -- Free param
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
  
  -- Safety check for the joints
  -- We must stay in the saggital plane
  q_desired[2] = 0
  q_desired[3] = 0

  -- Go to the allowable position
  local qL_approach, qR_approach, done
  if door_arm=='right' then
    qR_approach, done = util.approachTol( qRArm, q_desired, dqArmMax, dt )
  else
    qL_approach, done = util.approachTol( qLArm, q_desired, dqArmMax, dt )
  end

  -- Increment stage if done
  if done then stage=stage+1 end
  return qL_approach, qR_approach
end

local function punch_out()
    -- Calculate where we need to go  
  local trArm = vector.new({
    .9*handle_x, -- Free param
    K.shoulderOffsetY, -- 6 DOF arm cannot go certain places
    math.min(-0.1,handle_z), -- Free param
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
  
  -- Safety check for the joints
  -- We must stay in the saggital plane
  q_desired[2] = 0
  q_desired[3] = 0

  -- Go to the allowable position
  local qL_approach, qR_approach, done
  if door_arm=='right' then
    qR_approach, done = util.approachTol( qRArm, q_desired, dqArmMax, dt )
  else
    qL_approach, done = util.approachTol( qLArm, q_desired, dqArmMax, dt )
  end

  -- Increment stage if done
  if done then stage=stage+1 end
  return qL_approach, qR_approach
end

local function uppercut()

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
  
  -- Safety check for the joints
  -- We must stay in the saggital plane
  q_desired[2] = 0
  q_desired[3] = 0

  -- Go to the allowable position
  local qL_approach, qR_approach, done
  print('stage 3',done)
  if door_arm=='right' then
    qR_approach, done = util.approachTol( qRArm, q_desired, dqArmMax, dt )
  else
    qL_approach, done = util.approachTol( qLArm, q_desired, dqArmMax, dt )
  end

  -- Increment stage if done
  if done then stage=stage+1 end
  return qL_approach, qR_approach

end


local door_stages = {
  [1] = scrunch,
  [2] = punch_out,
  [3] = uppercut,
}

local possible
local update_human = function()
  handle = hcm.get_door_handle()

  -- Clamp between minimum and maximum values
  handle_x = math.min( math.max( handle[1], 0.3), .5 )
  handle_z = math.min( math.max( handle[3], -.1), .2 )

  -- Decide which arm to use
  if handle[2]>0 then
    door_arm = 'left'
  else
    door_arm = 'right'
  end

  -- Check if this is possible
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
  
  if q_desired then
    return true
  else
    print('Door grip not possible',trArm)
    return false
  end

end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  -- Where are we now?
  qLArm = Body.get_larm_command_position()
  qRArm = Body.get_rarm_command_position()
  -- Get the human estimate
  possible = update_human()

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
  dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  
  -- Update the human estimate
  if not possible then return 'reset' end

  -- Where are we now?
  qLArm = Body.get_larm_command_position()
  qRArm = Body.get_rarm_command_position()

  -- Get the position to go to, based on the current stage
  local qL_approach, qR_approach = door_stages[stage]()

  Body.set_larm_command_position( qL_approach or qLArm )
  Body.set_rarm_command_position( qR_approach or qRArm )

  -- TODO: Begin to grip by approaching the inner radius
  --[[
    ph = (t-t0)/t_grip
    handle_radius = handle_radius0*(1-ph) + ph*handle_radius1
  --]]

  --print('stage',stage,#door_stages)

  if stage>#door_stages then
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