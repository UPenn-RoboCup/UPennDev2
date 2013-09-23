local state = {}
state._NAME = ...
local Body   = require'Body'
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'
require'hcm'

-- Angular velocity limit
local dqArmMax = Config.arm.slow_limit

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
       * T.trans(0,handle_radius1,0)
       * T.rotZ(-math.pi/4)
   local trGripR = trHandle
       * T.rotX(turnAngle)
       * T.trans(0,-handle_radius1,0)
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
  print("Handle model:",wheel)
--SJ: Just in case
  if handle_pos[1]==0 then
    handle_pos={0.40,0,0.10}
    handle_yaw=0
    handle_pitch=0
    handle_radius=0.10

    hcm.set_wheel_model({handle_pos[1],handle_pos[2],handle_pos[3],
                        handle_yaw,handle_pitch,handle_radius})
  end

-- Inner and outer radius
  handle_radius0 = handle_radius - 0.02
  handle_radius1 = handle_radius + 0.02

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

--Hack here to initialize wrists correctly
--[[
if math.abs(qL_desired[5])>90*math.pi/180 then
  qL_desired[5] = util.mod_angle(qL_desired[5]+math.pi)
  qL_desired[6] = - qL_desired[6]
  qL_desired[7] = util.mod_angle(qL_desired[7]+math.pi)
end

if math.abs(qR_desired[5])>90*math.pi/180 then
  qR_desired[5] = util.mod_angle(qR_desired[5]+math.pi)
  qR_desired[6] = - qR_desired[6]
  qR_desired[7] = util.mod_angle(qR_desired[7]+math.pi)
end
--]]

--[[
    print("Larm:",
      qLArm[1]*180/math.pi,
      qLArm[2]*180/math.pi,
      qLArm[3]*180/math.pi,
      qLArm[4]*180/math.pi,
      qLArm[5]*180/math.pi,
      qLArm[6]*180/math.pi,
      qLArm[7]*180/math.pi)
--]]
--[[
    print("RWrist:",qRArm[5]*180/math.pi,
      qRArm[6]*180/math.pi,
      qRArm[7]*180/math.pi)
--]]

  if not qL_desired then
    print('Left not possible')
    return'reset'
  end
  if not qR_desired then
    print('Right not possible')
    return'reset'
  end

  -- Go to the allowable position
  local qL_approach, doneL
  qL_approach, doneL = util.approachTol( qLArm, qL_desired, dqArmMax, dt )
  Body.set_larm_command_position( qL_approach )
  
  local qR_approach, doneR
  qR_approach, doneR = util.approachTol( qRArm, qR_desired, dqArmMax, dt )
  Body.set_rarm_command_position( qR_approach )

  -- TODO: Begin to grip by approaching the inner radius
  --[[
    ph = (t-t0)/t_grip
    handle_radius = handle_radius0*(1-ph) + ph*handle_radius1
  --]]

  if doneL and doneR then
    -- Close the fingers
    Body.set_lgrip_percent(1)
    Body.set_rgrip_percent(1)
    return'done'
  end
  
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state