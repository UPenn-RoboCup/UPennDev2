local state = {}
state._NAME = 'armReady'
local Config = require'Config'
local Body   = require'Body'
local T      = require'Transform'
local util   = require'util'

Body = require(Config.Body)
local turnAngle = 0
local body_pos = {0,0,0}
local body_rpy = {0,0,0}

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
  
  handle_pos    = hcm:get_wheel_pos()
  handle_pitch  = hcm:get_wheel_pitchangle()
  handle_yaw    = hcm:get_wheel_yawangle()
  handle_radius = hcm:get_wheel_radius()
  turnAngle     = hcm:get_wheel_turnangle() 
  
  Body.set_arm_movement_velocity(
     vector.new({30,30,30,45,60,60})*math.pi/180)  
     
  t_last = unix.time()
end

turnAngleMag = 3*math.pi/180 --3 deg per sec


function state.update()
  --print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  
  -- TODO: hcm should have turn angle, not "control"
  left_trigger = hcm:get_control_left_gripper()[1]
  right_trigger = hcm:get_control_right_gripper()[1]  
  turnAngleTarget = -10* (left_trigger-right_trigger)*math.pi/180

  angleDiff = turnAngleTarget - turnAngle
  angleDiff = math.min(  turnAngleMag*dt, 
        math.max(-turnAngleMag*dt,  angleDiff))
  
  turnAngle = turnAngle + angleDiff  
  turnAngle = math.min(10*math.pi/180,math.max(-10*math.pi/180,turnAngle))
  
  qLArm = Body.get_larm_command_position()
  qRArm = Body.get_rarm_command_position()

  calculate_arm_position()
  checkLeft = Body.check_larm_ik(trLArm)
  checkRight = Body.check_rarm_ik(trRArm)
  
  if checkLeft<0.01 and checkRight<0.01 then
    qLInv = Kinematics.inverse_l_arm(trLArm, qLArm)
    qRInv = Kinematics.inverse_r_arm(trRArm, qRArm)
    Body.set_larm_target_position(qLInv) 
    Body.set_rarm_target_position(qRInv)   
    hcm:set_wheel_turnangle(turnAngle) --Store current turnangle
  end
  
end

function state.exit()
  print(state._NAME..' Exit' )
  Body.set_arm_movement_velocity(
     vector.new({30,30,30,45,60,60})*math.pi/180)  
end

return state
