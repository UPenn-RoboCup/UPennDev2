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




local enable_force_control = false


local torch = require'torch'
torch.Tensor = torch.DoubleTensor


local trLArmGoal,trRArmGoal --temporary variable for jacobian testing






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

  if enable_force_control then
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


  end

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

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local trLArm = Body.get_forward_larm(qLArm)
  local trRArm = Body.get_forward_larm(qRArm)
  trLArmGoal = Body.get_forward_larm(qLArm)
  trRArmGoal = Body.get_forward_rarm(qRArm)
  hcm.set_state_override({0,0,0,0,0,0,0}) 
end

local count=0


function check_override()
  local override = hcm.get_state_override()
  for i=1,7 do if override[i]~=0 then return true end end
  return false
end

jcount=0

function jacobian_control(dt)
jcount=jcount+1
t0=unix.time()   

  local qWaist = Body.get_waist_command_position()
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  
  if check_override() then
    local trRArm = Body.get_forward_rarm(qRArm)
    trRArmGoal = trRArm + hcm.get_state_override()    
    hcm.set_state_override({0,0,0,0,0,0,0}) 
    print("Goal set:",util.print_transform(trRArmGoal))
  end

--[[
  LArm_Jac = Body.Kinematics.calculate_arm_jacobian(
    qLArm,
    qWaist,
    {0,0,0}, --rpy angle
    1,       --is_left: not being used
    Config.arm.handoffset.gripper3[1],
    -Config.arm.handoffset.gripper3[2],  --positive Y value is inside
    Config.arm.handoffset.gripper3[3]
    );  --tool xyz

  local trLArm = Body.get_forward_larm(qLArm)
  local trLArmNext= util.approachTolTransform(
    trLArm, trLArmGoal,{0.3,0.3,0.3,0.3,0.3,0.3},dt)
  local trLArmDiff = util.diff_transform(trLArmNext,trLArm)  
--]]

--
  local RArm_Jac = Body.Kinematics.calculate_arm_jacobian(
    qRArm,
    qWaist,
    {0,0,0}, --rpy angle
    1,       --is_left: not being used
    0,0,0
--    Config.arm.handoffset.gripper3[1],
--    Config.arm.handoffset.gripper3[2],  
--    Config.arm.handoffset.gripper3[3]
    );  --tool xyz



  local trRArm = Body.get_forward_rarm(qRArm)  
  local trRArmDiff = util.diff_transform(trRArmGoal,trRArm)

  --Calculate target velocity
  local linear_dist = math.sqrt(
    (trRArm[1]-trRArmGoal[1])^2+
    (trRArm[2]-trRArmGoal[2])^2+
    (trRArm[3]-trRArmGoal[3])^2)

  local linear_vel = math.min(0.04, (linear_dist/0.02)*0.02 + 0.02 )


  local trRArmVelTarget={
    0,0,0,
    util.procFunc(-trRArmDiff[4],0,10*math.pi/180),
    util.procFunc(-trRArmDiff[5],0,10*math.pi/180),
    util.procFunc(-trRArmDiff[6],0,10*math.pi/180),
  }  

  if linear_dist>0 then
    trRArmVelTarget[1],trRArmVelTarget[2],trRArmVelTarget[3]=
    trRArmDiff[1]/linear_dist *linear_vel,
    trRArmDiff[2]/linear_dist *linear_vel,
    trRArmDiff[3]/linear_dist *linear_vel    
  end

  local angular_vel = 
     math.abs(trRArmVelTarget[4])
    +math.abs(trRArmVelTarget[5])
    +math.abs(trRArmVelTarget[6])
 
  if linear_dist<0.001 and angular_vel<1*math.pi/180 then
    return
  end

--  print("vel target:",util.print_transform(trRArmVelTarget))
  
--]]

--[[
  --qVel = inv(J'J + lambda^2* I) * J' * trMovement
  local J= torch.Tensor(LArm_Jac):resize(6,7)  
  local JT = torch.Tensor(J):transpose(1,2)
  local e = torch.Tensor(trLArmDiff)
  local I = torch.Tensor():resize(7,7):zero()
  local I2 = torch.Tensor():resize(7,6):zero()
  local qLArmVel = torch.Tensor(7):fill(0)

  I:addmm(JT,J):add(lambda*lambda,torch.eye(7))
  local Iinv=torch.inverse(I)  
  I2:addmm(Iinv,JT)   
  qLArmVel:addmv(I2,e)
  qLArmTarget = vector.new(qLArm)+vector.new(qLArmVel)
  Body.set_larm_command_position(qLArmTarget)
--]]

  --qVel = inv(J'J + lambda^2* I) * J' * trMovement
  local J= torch.Tensor(RArm_Jac):resize(6,7)  
  local JT = torch.Tensor(J):transpose(1,2)
  local e = torch.Tensor(trRArmVelTarget)
  local I = torch.Tensor():resize(7,7):zero()
  local I2 = torch.Tensor():resize(7,6):zero()
  local qRArmVel = torch.Tensor(7):fill(0)

  --todo: variable lambda to prevent self collision
  -- lambda_i = c*((2*q-qmin-qmax)/(qmax-qmin))^p + (1/w_i)

  local lambda=torch.eye(7)
  local c = 2 
  local p = 10

  local joint_limits={
    {-math.pi/2, math.pi},
    {0,math.pi/2},
    {-math.pi/2, math.pi/2},
    {-math.pi, -0.2}, --temp value
    {-math.pi, math.pi},
    {-math.pi/2, math.pi/2},
    {-math.pi, math.pi}
  }

  for i=1,7 do
    lambda[i][i]=0.1*0.1 + c*
      ((2*qLArm[i]-joint_limits[i][1]-joint_limits[i][2])/
       (joint_limits[i][2]-joint_limits[i][1]))^p
  end

  I:addmm(JT,J):add(1,lambda)
  local Iinv=torch.inverse(I)  
  I2:addmm(Iinv,JT)   
  qRArmVel:addmv(I2,e)
  qRArmTarget = vector.new(qRArm)+vector.new(qRArmVel)*dt
  Body.set_rarm_command_position(qRArmTarget)
  local trRArmTarget = Body.get_forward_rarm(qRArmTarget)
  local trRArmDiffActual = util.diff_transform(trRArmTarget,trRArm)

  
  
  if jcount%50==0 then
    print("trVelTarget:",util.norm(trRArmVelTarget,3),"trVelActual:",util.norm(trRArmDiffActual,3)/dt)      
  end 


--[[
  print("----")
  local count=1
  for i=1,6 do
    str=""
    for j=1,7 do
      str=str..string.format(" %.3f",LArm_Jac[count])
      count=count+1
    end
    print(str)
  end
  print("I matrix")
  for i=1,7 do
    str=""
    for j=1,7 do
      str=str..string.format(" %.3f",I[i][j])
    end
    print(str)
  end
--]]

end




function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t


  if Config.enable_jacobian_test then jacobian_control(dt) end

  
end





function state.exit()
  print(state._NAME..' Exit' )
end

return state
