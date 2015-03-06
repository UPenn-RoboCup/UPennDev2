local state = {}
state._NAME = ...
local vector = require'vector'

local Body = require'Body'
local t_entry, t_update, t_finish

require'mcm'

require'dcm'

local qLArm, qRArm


local larm_pos_old,rarm_pos_old,larm_vel_old,rarm_vel_old
local lleg_pos_old,rleg_pos_old
local l_comp_torque,r_comp_torque




local r_cmd_pos, r_cmd_vel 

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
  t_finish = t

--
  
  
  Body.set_larm_torque_enable({1,1,1, 1,1,1,1}) --enable force control
  Body.set_rarm_torque_enable({1,1,1, 1,1,1,1}) --enable force control



  Body.set_larm_torque_enable({2,2,2, 2,2,2,2}) --enable force control
--[[
  Body.set_larm_torque_enable({2,2,2, 2,2,2,2}) --enable force control
  Body.set_larm_torque_enable({2,1,1, 1,1,1,1}) --enable force control
  Body.set_larm_torque_enable({2,2,2, 2,2,1,2}) --enable force control
    
--  Body.set_larm_torque_enable({2,2,1, 2,2,1,2}) --enable force control
--  Body.set_larm_torque_enable({1,1,2, 1,1,1,1}) --enable force control    
--  Body.set_larm_torque_enable({2,1,1, 2,1,1,1}) --enable force control    


  Body.set_lleg_torque_enable({1,1,1, 1,1,1}) --enable force control
  Body.set_rleg_torque_enable({1,1,1, 1,1,1}) --enable force control

--  Body.set_lleg_torque_enable({1,1,2,1,1,1}) --enable force control
--  Body.set_rleg_torque_enable({1,1,2,1,1,1}) --enable force control
--]]

--  Body.set_larm_torque_enable({1,1,2, 1,1,1,1}) --enable force control


  larm_pos_old = Body.get_larm_position()  
  rarm_pos_old = Body.get_rarm_position()
  lleg_pos_old = Body.get_lleg_position()
  rleg_pos_old = Body.get_rleg_position()

  larm_vel_old = vector.zeros(7)
  rarm_vel_old = vector.zeros(7)

  l_comp_torque = vector.zeros(7)
  r_comp_torque = vector.zeros(7)

  r_cmd_pos = Body.get_rarm_command_position()
  r_cmd_vel=vector.zeros(7)

end

local count=0





function forcecontrol(dt)


count=count+1

    local rpy_angle = Body.get_rpy()

    local lleg_cmdpos = Body.get_lleg_command_position()
    local rleg_cmdpos = Body.get_rleg_command_position()
    local lleg_pos = Body.get_lleg_position()
    local rleg_pos = Body.get_rleg_position()
    local lleg_pos_err = (lleg_cmdpos-lleg_pos)
    local rleg_pos_err = (rleg_cmdpos-rleg_pos)
    local lleg_vel = (lleg_pos-lleg_pos_old)/dt;
    local rleg_vel = (rleg_pos-rleg_pos_old)/dt;

    lleg_pos_old,rleg_pos_old = lleg_pos,rleg_pos

    local lleg_actual_torque = Body.get_lleg_current()
    local rleg_actual_torque = Body.get_rleg_current()
    local lft = mcm.get_status_LFT()
    local rft = mcm.get_status_RFT()

    local qWaist = Body.get_waist_command_position()
    local qLArm = Body.get_larm_command_position()
    local qRArm = Body.get_rarm_command_position()

    local uTorso = mcm.get_status_uTorso()
    local uLeft = mcm.get_status_uLeft()
    local uRight = mcm.get_status_uRight()

    --TODO: find the shortest distance point from uTorso
    local uTorsoLeft= util.pose_relative(uLeft,uTorso)
    local uTorsoRight= util.pose_relative(uRight,uTorso)
    local leftDist,rightDist =  uTorsoLeft[2],-uTorsoRight[2]




--[[
    
    local com_legless=Body.Kinematics.calculate_com_pos2(qWaist,qLArm,qRArm,lleg_pos,rleg_pos,0,0,0,  0,0)
    print("upper body com offset:",com_legless[1]/com_legless[4])
    local com_leg=Body.Kinematics.calculate_com_pos2(qWaist,qLArm,qRArm,lleg_pos,rleg_pos,0,0,0,  1,1)
    print("whole body com offset:",com_leg[1]/com_leg[4])
--]]

  local com_whole_body=Body.Kinematics.calculate_com_pos2(qWaist,qLArm,qRArm,lleg_pos,rleg_pos,0,0,0,  1,1)
  local force_left = (rightDist/(leftDist+rightDist))*com_whole_body[4]*9.81
  local force_right = (leftDist/(leftDist+rightDist))*com_whole_body[4]*9.81
  if force_left<0 then force_left,force_right=0,com_whole_body[4]*9.81 end
  if force_right<0 then force_left,force_right=com_whole_body[4]*9.81,0 end

  local leg_acc_gain = 2
  local leg_damping_factor=vector.ones(6)*0 --internal damping of servo
  local leg_static_friction = vector.new({0,0,0,0, 0,0});

  --local lleg_comp_acc = util.pid_feedback(lleg_pos_err, lleg_vel, dt, leg_acc_gain)
  --local rleg_comp_acc = util.pid_feedback(rleg_pos_err, rleg_vel, dt, leg_acc_gain)

  local leg_p_gain,leg_d_gain = 10,-1

  local lleg_comp_acc = lleg_pos_err*leg_p_gain+lleg_vel*leg_d_gain
  local rleg_comp_acc = rleg_pos_err*leg_p_gain+rleg_vel*leg_d_gain

  local lsupport={0.037,0,0}
  local rsupport={0.037,0,0}


  local lleg_torques = Body.Kinematics.calculate_leg_torque(rpy_angle,lleg_pos, 1,lft[1] , lsupport  )

  local com_legless=Body.Kinematics.calculate_com_pos2(qWaist,qLArm,qRArm,lleg_pos,rleg_pos,0,0,0,  0,0)

--  local com_legless=Body.Kinematics.calculate_com_pos2(qWaist,qLArm,qRArm,lleg_pos,rleg_pos,0,0,0,  0,1)  --right leg only
  local com_offset_upperbody={com_legless[1]/com_legless[4],com_legless[2]/com_legless[4],com_legless[3]/com_legless[4]}


  local lleg_torques2 = Body.Kinematics.calculate_support_leg_torque(rpy_angle,lleg_pos,1,lft[1] , 
    com_offset_upperbody)

  local rleg_torques = Body.Kinematics.calculate_leg_torque(
      --rpy_angle,rleg_pos,rleg_comp_acc, 0,0 , rsupport   )
    rpy_angle,rleg_pos, 0,rft[1] , rsupport   )





  local lleg_stall_torque = vector.new(lleg_torques.stall);
  local rleg_stall_torque = vector.new(rleg_torques.stall);
  lleg_torques.acc = vector.zeros(6)
  rleg_torques.acc = vector.zeros(6)


  local lleg_acc_torque = util.linearize(lleg_torques.acc,lleg_vel,leg_damping_factor,leg_static_friction);
  local rleg_acc_torque = util.linearize(rleg_torques.acc,rleg_vel,leg_damping_factor,leg_static_friction);

--  Body.set_lleg_command_torque(lleg_stall_torque+lleg_acc_torque)
--  Body.set_rleg_command_torque(rleg_stall_torque+rleg_acc_torque)

  Body.set_lleg_command_torque(lleg_stall_torque)
  Body.set_rleg_command_torque(rleg_stall_torque)




  

--
----------------------------------------------------------------------------
-- Arm force-control code #1

  local larm_cmdpos = Body.get_larm_command_position()
  local rarm_cmdpos = Body.get_rarm_command_position()
  local larm_pos = Body.get_larm_position()
  local rarm_pos = Body.get_rarm_position()
  local larm_pos_err = (larm_cmdpos-larm_pos)
  local rarm_pos_err = (rarm_cmdpos-rarm_pos)
  local larm_vel = (larm_pos-larm_pos_old)/dt;
  local rarm_vel = (rarm_pos-rarm_pos_old)/dt;

  local larm_acc = (larm_vel-larm_vel_old)/dt

  larm_pos_old,rarm_pos_old = larm_pos,rarm_pos
  larm_vel_old,rarm_vel_old = larm_vel,rarm_vel

  local larm_actual_torque = Body.get_larm_current()
  local rarm_actual_torque = Body.get_rarm_current()

  local damping_factor=vector.ones(7)*0 --internal damping of servo
  local static_friction = vector.new({0,0,0,0, 0,0,0});

--[[  
  local arm_acc_gain = 20 --very stiff
  local larm_comp_acc,larm_vel_target = util.pid_feedback(larm_pos_err, larm_vel, dt, arm_acc_gain)
  local rarm_comp_acc,rarm_vel_target = util.pid_feedback(rarm_pos_err, rarm_vel, dt, arm_acc_gain)
--]]  

  local arm_p_gain,arm_d_gain = 4,-4
  local larm_comp_acc = larm_pos_err*arm_p_gain+larm_vel*arm_d_gain
  local rarm_comp_acc = rarm_pos_err*arm_p_gain+rarm_vel*arm_d_gain


  local l_torques,r_torques

  t0=unix.time()   

  l_torques = Body.Kinematics.calculate_arm_torque_adv(rpy_angle,larm_pos,larm_vel,larm_comp_acc);

  r_torques = Body.Kinematics.calculate_arm_torque(rpy_angle,rarm_pos)
  r_torques.acc=vector.zeros(7)

  t1=unix.time()

  local l_stall_torque = vector.new(l_torques.stall);
  local r_stall_torque = vector.new(r_torques.stall);
  local l_acc_torque = vector.new(l_torques.acc);
  local r_acc_torque = vector.new(r_torques.acc);
  local l_acc2_torque = vector.new(l_torques.acc2);

  damping_factor=vector.ones(7)*(0)


--  local l_acc_torque = util.linearize(l_torques.acc,larm_vel,damping_factor,static_friction);
  --local r_acc_torque = util.linearize(r_torques.acc,rarm_vel,damping_factor,static_friction);

  Body.set_larm_command_torque(l_stall_torque+l_acc_torque)
  Body.set_rarm_command_torque(r_stall_torque+r_acc_torque)






--[[
  local r_cmd_acc = (r_stall_torque-rarm_actual_torque)*0.5 - r_cmd_vel*1

  --r_cmd_acc[7],r_cmd_acc[2],r_cmd_acc[3],r_cmd_acc[5],r_cmd_acc[6]=0,0,0,0,0
  r_cmd_vel = r_cmd_vel + r_cmd_acc*dt;
  r_cmd_pos = r_cmd_pos + r_cmd_vel*dt;
  Body.set_rarm_command_position(r_cmd_pos)
--]]







----------------------------------------------------------------------------

  if count%300==0 then
--  if count%5==0 then
--    if true then
--  if false  then
    
  
--[[
print((t1-t0)*1000, "ms spent for dynamics calc")
    print(string.format("RArm actual torque: %.3f %.3f %.3f/ %.3f %.3f %.3f / %.3f",
        unpack(rarm_actual_torque)))
    print(string.format("RArm calced torque: %.3f %.3f %.3f/ %.3f %.3f %.3f / %.3f",
        unpack(r_stall_torque)))
--]]

--

print("----------------------")
    print(string.format("LArm   cur  vel     : %.3f %.3f %.3f/ %.3f %.3f %.3f / %.3f",
          unpack( larm_vel*RAD_TO_DEG) ))

    print(string.format("LArm   cur  acc     : %.3f %.3f %.3f/ %.3f %.3f %.3f / %.3f",
          unpack( larm_acc*RAD_TO_DEG) ))
    
    print(string.format("LArm calcu  acc   : %.3f %.3f %.5f/ %.3f %.3f %.3f / %.3f",
          unpack( larm_comp_acc*RAD_TO_DEG) ))

    print(string.format("LArm calcu  torque: %.3f %.3f %.5f/ %.3f %.3f %.3f / %.3f",
          unpack( l_stall_torque ) ))

    print(string.format("LArm   acc  torque: %.3f %.3f %.5f/ %.5f %.3f %.3f / %.3f",
          unpack( l_acc_torque ) ))

    print(string.format("LArm  acc2  torque: %.3f %.3f %.5f/ %.5f %.5f %.3f / %.5f",
          unpack( l_acc2_torque) ))
--    
--[[
print(string.format("%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f",

  Body.get_time(),
  larm_pos_err[1],
  larm_vel[1],
  larm_acc[1],
  larm_comp_acc[1],
  l_stall_torque[1],
  l_acc_torque[1],
  l_acc2_torque[1] ));          
--]]




--[[
    print("Support:",-uTorsoLeft[1])
    print(string.format("Roll: %.1f Pitch:%.1f",rpy_angle[1],rpy_angle[2]))

    print("calculated forces: ",force_left,force_right)
    print("measured forces: ",lft[1],rft[1])


    print(string.format("LLeg position error: %.3f %.3f/ %.3f %.3f %.3f / %.3f",
        unpack(lleg_pos_err*180/math.pi)))

    print(string.format("LLeg actual torque: %.3f %.3f/ %.3f %.3f %.3f / %.3f",
       unpack(lleg_actual_torque)))
    
    print(string.format("LLeg calced torque: %.3f %.3f/ %.2f %.2f %.2f / %.3f",
        unpack(lleg_stall_torque)))

    print(string.format("LLeg calced2 torque: %.3f %.3f/ %.2f %.2f %.2f / %.3f",
        unpack(lleg_torques2.stall)))



    print(string.format("LLeg   comp torque: %.3f %.3f/ %.2f %.2f %.2f / %.3f",
        unpack(lleg_acc_torque)))



    print(string.format("RLeg actual torque: %.3f %.3f / %.3f %.3f %.3f / %.3f",
        unpack(rleg_actual_torque)))
    print(string.format("RLeg calced torque: %.3f %.3f / %.3f %.3f %.3f / %.3f",
        unpack(rleg_stall_torque)))
--]]


  end
end





function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  forcecontrol(dt)
end





function state.exit()
  print(state._NAME..' Exit' )
end

return state
