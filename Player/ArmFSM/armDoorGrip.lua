local state = {}
state._NAME = ...
local Config = require'Config'
local Body   = require'Body'
local T      = require'Transform'
local util   = require'util'
require'hcm'

-- Angular velocity limit
local dqArmMax = vector.new({10,10,10,15,45,45})*Body.DEG_TO_RAD

local shoulder_y = .259

local turnAngle = 0
local body_pos = {0,0,0}
local body_rpy = {0,0,0}

local t_init = 5.0
local t_grip = 5.0

local handle_pos,handle_pitch,handle_yaw
local handle_radius1,handle_radius0,handle_radius
local trHandle, trGripL, trGripR, trBody, trLArm, trRArm
local function calculate_arm_position(turnAngle)

   local trHandle = T.eye()
       * T.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * T.rotZ(handle_yaw)
       * T.rotY(handle_pitch)
       
   local trBody = T.eye()
       * T.trans(body_pos[1],body_pos[2],body_pos[3])
       * T.rotZ(body_rpy[3])
       * T.rotY(body_rpy[2])
       
   local trLArm = T.position6D(T.inv(trBody)*trHandle)

   return trLArm

end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  
  -- Let's store handle data here
  local handle   = hcm.get_door_handle()
  handle_pos    = vector.slice(handle,1,3)
  -- ALways use this shoulder y
  handle_pos[2] = shoulder_y
  handle_yaw    = 0--handle[4]
  handle_pitch  = 0--wheel[5]
  handle_radius = handle[6] / 2

end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  
  local qLArm = Body.get_larm_command_position()
  
  -- Calculate where we need to go  
  --local trLArm = calculate_arm_position()
  -- TODO: Find the constraint in the plane
  local trLArm = vector.new({
    .43, shoulder_y, .22,
    -90*Body.DEG_TO_RAD, -60*Body.DEG_TO_RAD, 0, 
    })

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