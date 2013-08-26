local state = {}
state._NAME = 'armReady'
local Config = require'Config'
local Body   = require'Body'
local T      = require'Transform'

rotating_direction = 1
body_pos = {0,0,0}
body_rpy = {0,0,0}

t_init = 5.0
t_grip = 5.0


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
       
   trBody=T.eye()
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
  
  phase = 1
  t0 = unix.time()
  		  --Let's store wheel data here
  handle_pos =  hcm:get_wheel_pos()
  handle_pitch = hcm:get_wheel_pitchangle()[1]
  handle_yaw = hcm:get_wheel_yawangle()[1]
  handle_radius0 = hcm:get_wheel_radius()[1] + 0.08
  handle_radius1 = hcm:get_wheel_radius()[1]
  turnAngle = 0

  print("hpose:",unpack(handle_pos))
  print("hpitch:",handle_pitch)
  print("hradius",handle_radius)
  print("tAngle",turnAngle)
  
  handle_radius = handle_radius0
end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  
  calculate_arm_position()
  qLArm = Body.get_larm_command_position()
  qRArm = Body.get_rarm_command_position()
  qLInv = Kinematics.inverse_l_arm(trLArm, qLArm)
  qRInv = Kinematics.inverse_r_arm(trRArm, qRArm)

  if phase==1 then
    Body.set_larm_target_position(qLInv) 
    Body.set_rarm_target_position(qRInv) 
    if Body.larm_joint_movement_done() and Body.rarm_joint_movement_done() then
      phase = phase + 1
      t0 = unix.time()
      return
    end
  elseif phase==2 then
    Body.set_larm_target_position(qLInv) 
    Body.set_rarm_target_position(qRInv) 
    t = unix.time()
    ph = (t-t0)/t_grip
    if ph>1 then
      handle_radius = handle_radius1
      phase = 3
      Body.set_lhand_position(Config.arm.FingerClosed)
      Body.set_rhand_position(Config.arm.FingerClosed)
      return "done"  
    end
    handle_radius = handle_radius0*(1-ph) + ph*handle_radius1
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state