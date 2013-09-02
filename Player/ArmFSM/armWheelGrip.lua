local state = {}
state._NAME = 'armWheelGrip'
local Config = require'Config'
local Body   = require'Body'
local T      = require'Transform'
local util   = require'util'
require'hcm'

-- Angular velocity limit
local dqArmMax = vector.new({10,10,10,15,45,45})*Body.DEG_TO_RAD

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

   local trGripL = trHandle
       * T.rotX(turnAngle)
       * T.trans(0,handle_radius,0)
       * T.rotZ(-math.pi/4)
   local trGripR = trHandle
       * T.rotX(turnAngle)
       * T.trans(0,-handle_radius,0)
       * T.rotZ(math.pi/4)
       
   local trBody = T.eye()
       * T.trans(body_pos[1],body_pos[2],body_pos[3])
       * T.rotZ(body_rpy[3])
       * T.rotY(body_rpy[2])
       
   local trLArm = T.position6D(T.inv(trBody)*trGripL)
   local trRArm = T.position6D(T.inv(trBody)*trGripR)
   return trLArm, trRArm
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  
  -- Let's store wheel data here
  local wheel   = hcm.get_wheel_model()
  handle_pos    = vector.slice(wheel,1,3)
  handle_yaw    = wheel[4]
  handle_pitch  = wheel[5]
  handle_radius = wheel[6]
  -- Inner and outer radius
  handle_radius0 = handle_radius - 0.02
  handle_radius1 = handle_radius + 0.02

  print('Grabbing wheel:',wheel)

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
  local qRArm = Body.get_rarm_command_position()
  
  -- Calculate where we need to go  
  local trLArm, trRArm = calculate_arm_position(0)
  -- Get desired angles from current angles and target transform
  local qL_desired = Body.get_inverse_larm(qLArm,trLArm)
  local qR_desired = Body.get_inverse_rarm(qLArm,trRArm)
  if not qL_desired then
    print('Left not possible')
    return'reset'
  end
  if not qR_desired then
    print('Right not possible')
    return'reset'
  end

  -- Go there
  if qL_desired then
    qL_desired = util.approachTol( qLArm, qL_desired, dqArmMax, dt )
    if qL_desired~=true then Body.set_larm_command_position( qL_desired ) end
  end
  if qR_desired then
    qR_desired = util.approachTol( qRArm, qR_desired, dqArmMax, dt )
    if qR_desired~=true then Body.set_rarm_command_position( qR_desired ) end
  end

  -- TODO: Begin to grip by approaching the inner radius
  --[[
    ph = (t-t0)/t_grip
    handle_radius = handle_radius0*(1-ph) + ph*handle_radius1
  --]]

  if qL_desired==true and qR_desired==true then
    Body.set_lgrip_percent(.5)
    Body.set_rgrip_percent(.5)
    return'done'
  end
  
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state