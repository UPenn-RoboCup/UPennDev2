local state = {}
state._NAME = 'armWheelGrip'
local Config = require'Config'
local Body   = require'Body'
local T      = require'Transform'
local util   = require'util'

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
local function calculate_arm_position()
   trHandle = T.eye()
       * T.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * T.rotZ(handle_yaw)
       * T.rotY(handle_pitch)

   trGripL = trHandle
       * T.rotX(turnAngle)
       * T.trans(0,handle_radius,0)
       * T.rotZ(-math.pi/4)
   trGripR = trHandle
       * T.rotX(turnAngle)
       * T.trans(0,-handle_radius,0)
       * T.rotZ(math.pi/4)
       
   trBody = T.eye()
       * T.trans(body_pos[1],body_pos[2],body_pos[3])
       * T.rotZ(body_rpy[3])
		   * T.rotY(body_rpy[2])
		   
   trLArm = T.position6D(T.inv(trBody)*trGripL)
   trRArm = T.position6D(T.inv(trBody)*trGripR)
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  
  -- Let's store wheel data here
  handle_pos     = hcm:get_wheel_pos()
  handle_pitch   = hcm:get_wheel_pitchangle()
  handle_yaw     = hcm:get_wheel_yawangle()
  handle_radius1 = hcm:get_wheel_radius()
  handle_radius0 = handle_radius1 + 0.08
  handle_radius  = handle_radius0
  
  --[[
  print("hpose:",unpack(handle_pos))
  print("hpitch:",handle_pitch)
  print("hradius",handle_radius)
  print("tAngle",turnAngle)
  --]]

end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  
  -- Calculate where we need to go  
  calculate_arm_position()
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local qLInv = Kinematics.inverse_l_arm(trLArm, qLArm)
  local qRInv = Kinematics.inverse_r_arm(trRArm, qRArm)

  -- Go to the target arm position
  qLArm = util.approachTol( qLArm, qLInv, dqArmMax, dt )
  if qLArm~=true then Body.set_larm_command_position( qLArm ) end
  qRArm = util.approachTol( qRArm, qRInv, dqArmMax, dt )
  if qRArm~=true then Body.set_rarm_command_position( qRArm ) end

  -- TODO: Begin to grip by approaching the inner radius
  --[[
    ph = (t-t0)/t_grip
    handle_radius = handle_radius0*(1-ph) + ph*handle_radius1
  --]]

  if qLArm==true and qRArm==true then
    print'done grabbing'
    Body.set_lgrip_percent(.5)
    Body.set_rgrip_percent(.5)
    return'done'
  end
  
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state