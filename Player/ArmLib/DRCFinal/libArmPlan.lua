-- libArmPlan
-- (c) 2013 Seung-Joon Yi
-- Arm movement planner

local vector = require'vector'
local util = require'util'
require'mcm'

local movearm = require'movearm'
local sformat = string.format
local K = Body.Kinematics
--debug_on = true
debug_on = false
debug_on_2 = false
debugmsg = true







--print(unpack(Config.arm.iklookup.x))









local function tr_dist(trA,trB)
  return math.sqrt(  (trA[1]-trB[1])^2+(trA[2]-trB[2])^2+(trA[3]-trB[3])^2)
end

local function movfunc(cdist,dist)
  if dist==0 then return 0 end  

  local acc_factor = 1 /0.02 --accellerate 2X over 2cm
  local dcc_factor = 1 /0.03 --accellerate 2X over 3cm
  local max_vel_factor = 2 --max 3X speed
  

  local vel = math.min(
    cdist*acc_factor,
    (dist-cdist)*dcc_factor,
    max_vel_factor
    )+1
  return vel
end


local function print_arm_plan(arm_plan)
  print("Arm plan: total ", #arm_plan.LAP)  
  --  local arm_plan = {LAP = LAPs, RAP = RAPs,  uTP =uTPs, WP=WPs}
  
end



local function calculate_margin(qArm,isLeft,trArm)
  local jointangle_margin, trCheck
  if not qArm then return -math.huge 
  elseif isLeft==1 then --Left arm
    trCheck = Body.get_forward_larm(qArm,mcm.get_stance_bodyTilt(),qWaist)
  
    jointangle_margin = math.min(
      math.abs(qArm[2]-math.pi/2),       --Shoulder Roll 
      math.abs(qArm[2]),      
      math.abs(qArm[6]-math.pi/2),        --Wrist Roll
      math.abs(qArm[6]+math.pi/2)            
      )
--[[
    local sy_margin = math.min(
      math.abs(qArm[3]-math.pi/2),       --Shoulder Yaw
      math.abs(qArm[3]))      

    if sy_margin<10*math.pi/180 then
      jointangle_margin = math.min(jointangle_margin,sy_margin) 
    end
--]]
    if math.abs(qArm[6]) < 10*math.pi/180 then
      jointangle_margin = math.min(jointangle_margin,math.abs(qArm[6])) 
    end
  else --Right arm

    trCheck = Body.get_forward_rarm(qArm,mcm.get_stance_bodyTilt(),qWaist)
    jointangle_margin = math.min(
      math.abs(qArm[2]+math.pi/2),       --Shoulder Roll
      math.abs(qArm[2]),      
      math.abs(qArm[6]-math.pi/2), --Wrist Roll
      math.abs(qArm[6]+math.pi/2)      
      )

--[[
    local sy_margin = math.min(
      math.abs(qArm[3]+math.pi/2),       --Shoulder Yaw
      math.abs(qArm[3]))
    if sy_margin<10*math.pi/180 then
      jointangle_margin = math.min(jointangle_margin,sy_margin) 
    end
--]]
    if math.abs(qArm[6]) < 10*math.pi/180 then
      jointangle_margin = math.min(jointangle_margin,math.abs(qArm[6])) 
    end
  end

--[[
  if isLeft==1 then --Left arm
    if qArm[2]>math.pi/2 then 
--      print("L Roll out of bounds error", qArm[2]*RAD_TO_DEG) 
      jointangle_margin = 0 
    end
    if qArm[3]>math.pi/2 or qArm[3]<0 then 
--      print("L yaw OUT OF BOUNDS EROR!!!",qArm[3]*RAD_TO_DEG) 
      jointangle_margin = 0 
    end

  else
    if qArm[2]<-math.pi/2 then 
--      print("R Roll out of bounds error",qArm[2]*RAD_TO_DEG) 
      jointangle_margin = 0 
    end
    if qArm[3]<-math.pi/2 or qArm[3]>0 then 
--      print("R yaw OUT OF BOUNDS EROR!!!",qArm[3]*RAD_TO_DEG) 
      jointangle_margin = 0 
    end
  end
--]]


--[[
  local tr_err = math.abs(trArm[1]-trCheck[1])+math.abs(trArm[2]-trCheck[2])+math.abs(trArm[3]-trCheck[3])
  if jointangle_margin>0 and tr_err>0.005 then 
    print("IK ERROR"    )
    jointangle_margin = 0 
  end
--]]


  --Clamp the margin
  jointangle_margin = math.min(Config.arm.plan.max_margin,jointangle_margin)


  if K.collision_check_single(qArm,isLeft)>0 then jointangle_margin=-math.huge end

--  print("yaw, margin:",qArm[3]*Body.RAD_TO_DEG,jointangle_margin*Body.RAD_TO_DEG)
  return jointangle_margin
end




local function get_shoulder_yaw_angle_lookup(trArmNext,qArm)
  isLeft=0

  local xmin,xmax,div = Config.arm.iklookup.x[1],Config.arm.iklookup.x[2],Config.arm.iklookup.x[3]
  local ymin,ymax = Config.arm.iklookup.y[1],Config.arm.iklookup.y[2]
  local zmin,zmax = Config.arm.iklookup.z[1],Config.arm.iklookup.z[2]
  local xmag,ymag,zmag = (xmax-xmin)/div+1,(ymax-ymin)/div+1,(zmax-zmin)/div+1

--  print("current xyz:",util.print_transform(trArmNext))

  x = (math.max(xmin,math.min(xmax,trArmNext[1])) - xmin)/div
  y = (math.max(ymin,math.min(ymax,trArmNext[2])) - ymin)/div
  z = (math.max(zmin,math.min(zmax,trArmNext[3])) - zmin)/div
  
  local xi,yi,zi = math.floor(x+0.5),math.floor(y+0.5),math.floor(z+0.5)  
  local index = zi + yi*zmag + xi*zmag*ymag

  for i=index-10,index+10 do print(i,Config.arm.iklookup.dat[i]) end

  local yawangle = Config.arm.iklookup.dat[index]*DEG_TO_RAD
  print("yawangle:",yawangle/DEG_TO_RAD)
  if yawangle>90*DEG_TO_RAD then 
    print("no solution here")
    return 
  end

  local qArmNext = Body.get_inverse_rarm(qArm,trArmNext, yawangle, 0, {0,0}) 

  if not qArmNext then print("NO SOLUTION WTF") end

  return qArmNext 

--[[
  local margin = calculate_margin(qArmNext,isLeft,trArmNext)
  if margin>=0 then 
    print("margin:",margin/DEG_TO_RAD)
    return qArmNext 
  end
  print("no margin!")
  return 
--]]  
end






local function search_shoulder_angle(self,qArm,trArmNext,isLeft, yawMag, qWaist)


  if isLeft==0 and trArmNext[6] ==45*DEG_TO_RAD then
    return get_shoulder_yaw_angle_lookup(trArmNext,qArm)
  end


  local step = Config.arm.plan.search_step

  --Calculte the margin for current shoulder yaw angle
  local qArmMaxMargin, qArmNext
  local max_margin = -math.huge  
  local debugmsg=false

--  local debugmsg=true

  local arm_endpoint_compensation = mcm.get_arm_endpoint_compensation()
  if isLeft>0 and arm_endpoint_compensation[1]==0 then return qArm end
  if isLeft<=0 and arm_endpoint_compensation[2]==0 then return qArm end

  if isLeft>0 and self.shoulder_yaw_target_left then 
    local shoulderYaw = util.approachTol(qArm[3],self.shoulder_yaw_target_left, yawMag, 1)
    local qArmNext = Body.get_inverse_larm(qArm,trArmNext, shoulderYaw, mcm.get_stance_bodyTilt(), qWaist)            
    return qArmNext
  elseif isLeft==0 and self.shoulder_yaw_target_right then
    local shoulderYaw = util.approachTol(qArm[3],self.shoulder_yaw_target_right, yawMag, 1)
    local qArmNext = Body.get_inverse_rarm(qArm,trArmNext, shoulderYaw, mcm.get_stance_bodyTilt(), qWaist)
    return qArmNext
  end

  for div = -1,1,step do
    local qShoulderYaw = qArm[3] + div * yawMag
    local qArmNext
    if isLeft>0 then qArmNext = Body.get_inverse_larm(qArm,trArmNext, qShoulderYaw, mcm.get_stance_bodyTilt(), qWaist)
    else qArmNext = Body.get_inverse_rarm(qArm,trArmNext, qShoulderYaw, mcm.get_stance_bodyTilt(), qWaist) end
    local margin = self.calculate_margin(qArmNext,isLeft,trArmNext)
    if debugmsg then print("CHECKING SHOULDERYAW ",qShoulderYaw*180/math.pi,margin*180/math.pi) end
    if margin>=max_margin then qArmMaxMargin, max_margin = qArmNext, margin end
  end
  if max_margin<=0 then
    print("CANNOT FIND CORRECT SHOULDER ANGLE, at trNext:",util.print_transform(trArmNext))
    return
  else
    if debugmsg then print(sformat("shoulder angle::%.2f",qArmMaxMargin[3]*180/math.pi)) end
    return qArmMaxMargin
  end
end


--[[
local function search_shoulder_angle(self,qArm,trArmNext,isLeft, yawMag, qWaist, dt_step)
  local step = Config.arm.plan.search_step

  --Calculte the margin for current shoulder yaw angle
  local qArmMaxMargin, qArmNext
  local max_margin = -math.huge  
  local debugmsg=false
  if isLeft==0 then  debugmsg=true end
  if isLeft>0 and self.shoulder_yaw_target_left then 
    local shoulderYaw = util.approachTol(qArm[3],self.shoulder_yaw_target_left, yawMag, 1)
    local qArmNext = Body.get_inverse_larm(qArm,trArmNext, shoulderYaw, mcm.get_stance_bodyTilt(), qWaist)            
    return qArmNext
  elseif isLeft==0 and self.shoulder_yaw_target_right then
    local shoulderYaw = util.approachTol(qArm[3],self.shoulder_yaw_target_right, yawMag, 1)
    local qArmNext = Body.get_inverse_rarm(qArm,trArmNext, shoulderYaw, mcm.get_stance_bodyTilt(), qWaist)
    return qArmNext
  end

  local s0,s1
  if isLeft>0 then
    s0 = math.max(0,qArm[3] - 30*math.pi/180)
    s1 = math.min(math.pi/2,qArm[3] + 30*math.pi/180)
  else
    s0 = math.max(-math.pi/2,qArm[3] - 30*math.pi/180)
    s1 = math.min(0,qArm[3] + 30*math.pi/180)    
  end

--  print(s0,qArm[3],s1)

  for qShoulderYaw = s0,s1,math.pi/180 do    
    local qArmNext
    if isLeft>0 then qArmNext = Body.get_inverse_larm(qArm,trArmNext, qShoulderYaw, mcm.get_stance_bodyTilt(), qWaist)
    else qArmNext = Body.get_inverse_rarm(qArm,trArmNext, qShoulderYaw, mcm.get_stance_bodyTilt(), qWaist) end
    local margin = self.calculate_margin(qArmNext,isLeft)

--    print("yaw searching",qShoulderYaw,margin)

    if margin>=max_margin then qArmMaxMargin, max_margin = qArmNext, margin end
  end
  if max_margin<0 then
    print("CANNOT FIND CORRECT SHOULDER ANGLE, at trNext:",util.print_transform(trArmNext))
    return
  else

    local next_yaw_angle = util.approachTol(qArm[3], qArmMaxMargin[3], 20*math.pi/180, dt_step)

    if debugmsg then print(sformat("shoulder angle::%.2f",next_yaw_angle*180/math.pi)) end

    if isLeft>0 then qArmNext = Body.get_inverse_larm(qArm,trArmNext, next_yaw_angle, mcm.get_stance_bodyTilt(), qWaist)
    else  qArmNext = Body.get_inverse_rarm(qArm,trArmNext, next_yaw_angle, mcm.get_stance_bodyTilt(), qWaist) end

    return qArmNext
  end
end
--]]
















local function get_admissible_dt_accel(qMovement, qVelLast,accLimit)
  local dt_min , dt_max = -math.huge, math.huge
  for j = 1,7 do
    local dx = util.mod_angle(qMovement[j])
    local min_vel, max_vel = qVelLast[j] - accLimit[j] ,qVelLast[j] + accLimit[j] 
    --    dt * min_vel   <= dx <= dt* max_vel
    
    if max_vel> 0 then dt_min = math.max(dt_min, dx/max_vel)
    elseif max_vel<0 then dt_max = math.min(dt_max, dx/max_vel)
    else       --dx <= 0
    end
    if min_vel>0 then dt_max = math.min(dt_max, dx/min_vel)
    elseif min_vel<0 then dt_min = math.max(dt_min, dx/min_vel)
    else       --dx >=0
    end
  end
  return dt_min, dt_max
end

local function get_admissible_dt_vel(qMovement, velLimit)
  local dt_min = -math.huge
  for j = 1,7 do dt_min = math.max(dt_min, util.mod_angle(qMovement[j])/velLimit[j]) end
  return dt_min
end

--now linear joint level accelleration
local function filter_arm_plan(plan)
  local num = #plan.LAP
  local velLimit = dqArmMax
  local t0 =unix.time()
  
  local velLimit0 = Config.arm.vel_angular_limit
  local accLimit0 = vector.new({20,20,20,20,30,30,30})*DEG_TO_RAD --accelleration value
  local accMax = vector.new({60,60,60,60,90,90,90})

  local dt = {}
  local qLArmMov,qRArmMov = {},{}
  local qLArmVel,qRArmVel = {},{}

  --Initial filtering based on max velocity
  for i=1,num-1 do    
    qLArmMov[i] =  vector.new(plan.LAP[i+1][1]) - vector.new(plan.LAP[i][1])
    qRArmMov[i] =  vector.new(plan.RAP[i+1][1]) - vector.new(plan.RAP[i][1])
    local dt_min_left  = get_admissible_dt_vel(qLArmMov[i], velLimit0)
    local dt_min_right = get_admissible_dt_vel(qRArmMov[i], velLimit0)
--    dt[i] = math.max(dt_min_left, dt_min_right,0.01)
    dt[i] = math.max(dt_min_left, dt_min_right, Config.arm.plan.dt_step_min)
    qLArmVel[i] = qLArmMov[i]/dt[i]    
    qRArmVel[i] = qLArmMov[i]/dt[i]
  end

  --Accelleration filtering
--[[
  for i=2,num-2 do    
    local accLimit = accLimit0 * dt[i-1]    
    local dt_min_left, dt_max_left = get_admissible_dt(qLArmMov[i], qLArmVel[i-1], qLArmVel[i+1], accLimit)
    local dt_min_right, dt_max_right = get_admissible_dt(qLArmMov[i], qLArmVel[i-1], qLArmVel[i+1], accLimit)    
    local dt_accel = math.max(dt_min_left, dt_min_right)
    print("Accelerated dt:")
    qLArmVel[i] = qLArmMov[i]/dt[i]    
    qRArmVel[i] = qLArmMov[i]/dt[i]
  end  
--]]
  local total_time1,total_time2=0,0

  for i=1,num-1 do 
    total_time1=total_time1+plan.LAP[i][2]
    plan.LAP[i][2] = dt[i] 
    total_time2 = total_time2+dt[i]
  end

  local t1 =unix.time()
  print(sformat("%d segments, Filtering time: %.2f ms\nOrg: %.fs Execution time: %.1fs",
    num-1,(t1-t0)*1000,total_time1,total_time2))  
end




local function set_hand_mass(self,mLeftHand, mRightHand) self.mLeftHand, self.mRightHand = mLeftHand, mRightHand end

local function get_torso_compensation(qLArm,qRArm,qWaist,massL,massR)
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()
  local uTorso = mcm.get_status_uTorso()
  local zLeg = mcm.get_status_zLeg()
  local zSag = mcm.get_walk_zSag()
  local zLegComp = mcm.get_status_zLegComp()
  local zLeft,zRight = zLeg[1]+zSag[1]+zLegComp[1],zLeg[2]+zSag[2]+zLegComp[2]


  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})  
  local aShiftX = mcm.get_walk_aShiftX()
  local aShiftY = mcm.get_walk_aShiftY()
  local torsoX    = Config.walk.torsoX

  local count,revise_max = 1,4
  local adapt_factor = 1.0

 --Initial guess 
  local uTorsoAdapt = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  local pTorso = vector.new({
    uTorsoAdapt[1], uTorsoAdapt[2], mcm.get_stance_bodyHeight(),
            0,mcm.get_stance_bodyTilt(),uTorsoAdapt[3]})
  local qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso,aShiftX,aShiftY)
  
  -------------------Incremental COM filtering
  while count<=revise_max do
    local qLLeg = vector.slice(qLegs,1,6)
    local qRLeg = vector.slice(qLegs,7,12)
    com = K.calculate_com_pos(qWaist,qLArm,qRArm,qLLeg,qRLeg,0,0,3*DEG_TO_RAD)
    local uCOM = util.pose_global(
      vector.new({com[1]/com[4], com[2]/com[4],0}),uTorsoAdapt)

   uTorsoAdapt[1] = uTorsoAdapt[1]+ adapt_factor * (uTorso[1]-uCOM[1])
   uTorsoAdapt[2] = uTorsoAdapt[2]+ adapt_factor * (uTorso[2]-uCOM[2])
   local pTorso = vector.new({
            uTorsoAdapt[1], uTorsoAdapt[2], mcm.get_stance_bodyHeight(),
            0,mcm.get_stance_bodyTilt(),uTorsoAdapt[3]})
   qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso, aShiftX, aShiftY)
   count = count+1
  end
  local uTorsoOffset = util.pose_relative(uTorsoAdapt, uTorso)
  return {uTorsoOffset[1],uTorsoOffset[2]}
  
end

local function reset_torso_comp(self,qLArm,qRArm)
  local qWaist = Body.get_waist_command_position()
  self.torsoCompBias = get_torso_compensation(qLArm,qRArm,qWaist,0,0)  
  mcm.set_stance_uTorsoCompBias(self.torsoCompBias)  
  self:save_boundary_condition({qLArm,qRArm,qLArm,qRArm,{0.0}})
end



local function get_next_movement(self, init_cond, trLArm1,trRArm1, dt_step, waistYaw, waistPitch)
  local default_hand_mass = Config.arm.default_hand_mass or 0
  local dqVelLeft = mcm.get_arm_dqVelLeft()
  local dqVelRight = mcm.get_arm_dqVelRight()

  local velTorsoComp = Config.arm.torso_comp_limit
  local velYaw = Config.arm.shoulder_yaw_limit
  local massL, massR = self.mLeftHand + default_hand_mass, self.mRightHand + default_hand_mass

  local qLArm,qRArm, qLArmComp , qRArmComp, uTorsoComp = unpack(init_cond)

  local yawMag = dt_step * velYaw
  local qLArmNext, qRArmNext = qLArm, qRArm
  local qWaist = {waistYaw, waistPitch}

  qLArmNext = self:search_shoulder_angle(qLArm,trLArm1,1, yawMag, qWaist,dt_step)
  qRArmNext = self:search_shoulder_angle(qRArm,trRArm1,0, yawMag, qWaist,dt_step)
  


  if not qLArmNext or not qRArmNext then return end


  Kinematics.collision_check(qLArmNext,qRArmNext)



  local trLArmNext = Body.get_forward_larm(qLArmNext,mcm.get_stance_bodyTilt(),qWaist)
  local trRArmNext = Body.get_forward_rarm(qRArmNext,mcm.get_stance_bodyTilt(),qWaist)
  local vec_comp = vector.new({-uTorsoComp[1],-uTorsoComp[2],0,0,0,0})
  local trLArmNextComp = vector.new(trLArmNext) + vec_comp
  local trRArmNextComp = vector.new(trRArmNext) + vec_comp

  local endpoint_compensation = mcm.get_arm_endpoint_compensation()
  local qLArmNextComp, qRArmNextComp = qLArmNext, qRArmNext

  --Actual arm angle considering the torso compensation
  if endpoint_compensation[1]>0 then
    qLArmNextComp = self:search_shoulder_angle(qLArmComp,trLArmNextComp,1, yawMag, qWaist,dt_step)
  end
  if endpoint_compensation[2]>0 then
    qRArmNextComp = self:search_shoulder_angle(qRArmComp,trRArmNextComp,0, yawMag, qWaist,dt_step)
  end

  if not qLArmNextComp or not qRArmNextComp or not qLArmNext or not qRArmNext then 
    print("ERROR")
    return 
  else
    local dt_step_current = dt_step
    local uTorsoCompNextTarget = get_torso_compensation(qLArmNext,qRArmNext,qWaist, massL,massR)
    local uTorsoCompNext, torsoCompDone = util.approachTol(uTorsoComp, uTorsoCompNextTarget, velTorsoComp, dt_step_current )
    local new_cond = {qLArmNext, qRArmNext, qLArmNextComp, qRArmNextComp, uTorsoCompNext, waistYaw, waistPitch}
    return new_cond, dt_step_current, torsoCompDone    
  end
end


local function plan_unified(self, plantype, init_cond, init_param, target_param)
  local dpVelLeft = mcm.get_arm_dpVelLeft()
  local dpVelRight = mcm.get_arm_dpVelRight()

  --param: {trLArm,trRArm} for move
  --param: {trLArm,trRArm} for wrist

  if not init_cond then return end
  local done, failed = false, false, false

  local qWaist = {init_cond[6] or Body.get_waist_command_position()[1],init_cond[7] or Body.get_waist_command_position()[2]}
  local current_cond = {init_cond[1],init_cond[2],init_cond[3],init_cond[4],{init_cond[5][1],init_cond[5][2]},qWaist[1],qWaist[2]}
  local trLArm = Body.get_forward_larm(init_cond[1],mcm.get_stance_bodyTilt(),qWaist) 
  local trRArm =Body.get_forward_rarm(init_cond[2],mcm.get_stance_bodyTilt(),qWaist) 


  local qLArm0, qRArm0 = init_cond[1],init_cond[2]

  --Insert initial arm joint angle to the queue
  local dt_step0 = Config.arm.plan.dt_step0
  local dt_step = Config.arm.plan.dt_step
  local qLArmQueue,qRArmQueue, uTorsoCompQueue = {{init_cond[3],dt_step0}}, {{init_cond[4],dt_step0}}, {init_cond[5]}

--now pitch too
  local qWaistQueue={{current_cond[6],current_cond[7]}}

  local current_param={unpack(init_param)}
  local qArmCount = 2
  local vel_param  

  if plantype=="move" then
    current_param[3] = current_cond[6]
    current_param[4] = current_cond[7]
    target_param[3] = target_param[3] or current_cond[6]
    target_param[4] = target_param[4] or current_cond[7]

    vel_param = {dpVelLeft,dpVelRight,Config.arm.vel_waist_limit[1],Config.arm.vel_waist_limit[2]}
  elseif plantype=="wrist" then
--[[    
  elseif plantype=="door" then
    target_param[4] = target_param[4] or current_cond[6]

    local dpDoorMax = vector.slice(dpVelRight,1,3)
    local dqDoorRollMax = Config.armfsm.dooropen.velDoorRoll
    local dqDoorYawMax = Config.armfsm.dooropen.velDoorYaw
    local dqWaistYawMax = Config.armfsm.dooropen.velWaistYaw
    vel_param={dpDoorMax, dqDoorRollMax, dqDoorYawMax, dqWaistYawMax}

  elseif plantype=="dooredge" then
    target_param[4] = target_param[4] or current_cond[6]
    local dpDoorMax = vector.slice(dpVelRight,1,3)
    local dqDoorYawMax = Config.armfsm.dooropen.velDoorYaw
    local dqWaistYawMax = Config.armfsm.dooropen.velWaistYaw
    vel_param={dpDoorMax, dqDoorYawMax, dqWaistYawMax}

  elseif plantype=="doorleft" then
    target_param[4] = target_param[4] or current_cond[6]
    local dpDoorMax = vector.slice(dpVelLeft,1,3)
    local dqDoorRollMax = Config.armfsm.dooropen.velDoorRoll
    local dqDoorYawMax = Config.armfsm.dooropen.velDoorYaw
    vel_param={dpDoorMax, dqDoorRollMax, dqDoorYawMax}

  elseif plantype=="valvetwoarm" then
    vel_param={
      Config.armfsm.valvetwoarm.velTurnAngle,
      Config.armfsm.valvetwoarm.velTurnAngle,
      Config.armfsm.valvetwoarm.velInsert,
      Config.armfsm.valvetwoarm.velInsert}
  elseif plantype=="valveonearm" then
    vel_param={
      Config.armfsm.valveonearm.velTurnAngle,
      Config.armfsm.valveonearm.velInsert}
  elseif plantype=="barvalve" then        
    vel_param={
      Config.armfsm.valvebar.velTurnAngle,
      Config.armfsm.valvebar.velTurnAngle,
      Config.armfsm.valvebar.velInsert}
--]]      
  end
  
  local done, torsoCompDone = false, false
  local trLArmNext, trRArmNext, waistNext
  local new_param = {}

  local t0 = unix.time()  
  local t_robot = 0
  local done2 = false

  while not done and not failed  do --we were skipping the last frame
    if plantype=="move" then
      local distL = tr_dist(init_param[1],target_param[1])
      local distR = tr_dist(init_param[2],target_param[2])
      local cdistL = tr_dist(init_param[1],trLArm)
      local cdistR = tr_dist(init_param[2],trRArm)
      
      local velL = movfunc(cdistL,distL)
      local velR = movfunc(cdistR,distR)

      trLArmNext,doneL = util.approachTolTransform(trLArm, target_param[1], dpVelLeft*velL, dt_step )
      trRArmNext,doneR = util.approachTolTransform(trRArm, target_param[2], dpVelRight*velR, dt_step )

      --Waist yaw
      new_param[3],done3 = util.approachTol(current_param[3],target_param[3],vel_param[3],dt_step )
      --Waist pitch
      new_param[4],done4 = util.approachTol(current_param[4],target_param[4],vel_param[4],dt_step )

      waistNext = {new_param[3], new_param[4]}
--      print("waistNext:",unpack(waistNext))


--print("waist:",new_param[3]*Body.RAD_TO_DEG,new_param[4]*Body.RAD_TO_DEG)

      done = doneL and doneR and done3 and done4
    elseif plantype=="wrist" then
      trLArmNext,doneL = util.approachTolWristTransform(trLArm, target_param[1], dpVelLeft, dt_step )      
      trRArmNext,doneR = util.approachTolWristTransform(trRArm, target_param[2], dpVelRight, dt_step )
      done = doneL and doneR

      local qLArmTemp = Body.get_inverse_arm_given_wrist( qLArm0, trLArmNext)
      local qRArmTemp = Body.get_inverse_arm_given_wrist( qRArm0, trRArmNext)
      trLArmNext = Body.get_forward_larm(qLArmTemp)
      trRArmNext = Body.get_forward_rarm(qRArmTemp)  
      waistNext = {current_cond[6], current_cond[7]}

--      print("waistNext:",unpack(waistNext))      

    elseif plantype=="door" then
      local done1,done2,done3,done4      
      new_param[1], done1 = util.approachTol(current_param[1],target_param[1], vel_param[1],dt_step)
      new_param[2], done2 = util.approachTol(current_param[2],target_param[2], vel_param[2],dt_step)
      new_param[3], done3 = util.approachTol(current_param[3],target_param[3], vel_param[3],dt_step)
      new_param[4], done4 = util.approachTol(current_param[4],target_param[4], vel_param[4],dt_step)
      done = done1 and done2 and done3 and done4
      trLArmNext = trLArm
      trRArmNext = movearm.getDoorHandlePosition(current_param[1],current_param[2],current_param[3])      
--      waistNext = new_param[4]
      waistNext = {new_param[4], current_cond[7]}

--      print("waist:",waistNext*Body.RAD_TO_DEG)

    elseif plantype=="doorleft" then
      local done1,done2,done3
      new_param[1], done1 = util.approachTol(current_param[1],target_param[1], vel_param[1],dt_step)
      new_param[2], done2 = util.approachTol(current_param[2],target_param[2], vel_param[2],dt_step)
      new_param[3], done3 = util.approachTol(current_param[3],target_param[3], vel_param[3],dt_step)

      done = done1 and done2 and done3 
      
      trLArmNext = movearm.getDoorHandlePosition(current_param[1],current_param[2],current_param[3],1)      
      trRArmNext = trRArm
      waistNext = {current_cond[6], current_cond[7]}

    elseif plantype =="valvetwoarm" then
      new_param[1], done1 = util.approachTol(current_param[1],target_param[1], vel_param[1],dt_step)
      new_param[2], done2 = util.approachTol(current_param[2],target_param[2], vel_param[2],dt_step)
      new_param[3], done3 = util.approachTol(current_param[3],target_param[3], vel_param[3],dt_step)
      new_param[4], done4 = util.approachTol(current_param[4],target_param[4], vel_param[4],dt_step)
      done = done1 and done2 and done3 and done4
      trLArmNext, trRArmNext =  movearm.getLargeValvePosition(unpack(new_param))
      waistNext = {current_cond[6], current_cond[7]}

    elseif plantype =="valveonearm" then
      new_param[1], done1 = util.approachTol(current_param[1],target_param[1], vel_param[1],dt_step)
      new_param[2], done2 = util.approachTol(current_param[2],target_param[2], vel_param[2],dt_step)
      new_param[3],new_param[4] = target_param[3],target_param[4]      
      done = done1 and done2

      if new_param[3]>0 then --left arm      
        trLArmNext, trRArmNext =  movearm.getLargeValvePositionSingle(unpack(new_param)), trRArm
      else
        trLArmNext, trRArmNext =  trLArm, movearm.getLargeValvePositionSingle(unpack(new_param))
      end
      waistNext = {current_cond[6], current_cond[7]}

    elseif plantype =="barvalve" then
      new_param[1], done1 = util.approachTol(current_param[1],target_param[1], vel_param[1],dt_step)
      new_param[2], done2 = util.approachTol(current_param[2],target_param[2], vel_param[2],dt_step)
      new_param[3], done3 = util.approachTol(current_param[3],target_param[3], vel_param[3],dt_step)
      new_param[4] = target_param[4]      
      done = done1 and done2 and done3

      if new_param[4]>0 then --left arm      
        trLArmNext, trRArmNext =  movearm.getBarValvePositionSingle(unpack(new_param)), trRArm
      else
        trLArmNext, trRArmNext =  trLArm, movearm.getBarValvePositionSingle(unpack(new_param))
      end
      waistNext = {current_cond[6], current_cond[7]}
    elseif plantype =="hoseattach" then
      new_param[1], done1 = util.approachTol(current_param[1],target_param[1], vel_param[1],dt_step)
      new_param[2], done2 = util.approachTol(current_param[2],target_param[2], vel_param[2],dt_step)
      new_param[3], done3 = util.approachTol(current_param[3],target_param[3], vel_param[3],dt_step)
      new_param[4], done4 = util.approachTol(current_param[4],target_param[4], vel_param[4],dt_step)
      done = done1 and done2 and done3 and done4
      trLArmNext, trRArmNext =  movearm.getHoseAttachPosition(new_param[1],new_param[2],new_param[3],new_param[4])
      waistNext = {current_cond[6], current_cond[7]}
    end

    local new_cond, dt_step_current, torsoCompDone=    
      self:get_next_movement(current_cond, trLArmNext, trRArmNext, dt_step, waistNext[1], waistNext[2])

    done = done and torsoCompDone
    if not new_cond then 
      if plantype=="door" then
        print("door fail, yaw:",current_param[3]*Body.RAD_TO_DEG)
      elseif plantype=="dooredge" then
        print("dooredge fail, yaw:",current_param[2]*Body.RAD_TO_DEG)
      end
      failed = true    
    else
      t_robot = t_robot + dt_step_current
      trLArm, trRArm = trLArmNext, trRArmNext
      qLArmQueue[qArmCount] = {new_cond[3],dt_step_current}
      qRArmQueue[qArmCount] = {new_cond[4],dt_step_current}
      qWaistQueue[qArmCount] = waistNext 
      uTorsoCompQueue[qArmCount] = {new_cond[5][1],new_cond[5][2]}      
      current_cond = new_cond
      current_param = new_param
      qArmCount = qArmCount + 1
    end    
  end

  local t1 = unix.time()  

  if failed then return end

--[[
  local trLArm0, trRArm0 = Body.get_forward_larm(qLArm0), Body.get_forward_rarm(qRArm0)
  local trLArm1, trRArm1 = Body.get_forward_larm(current_cond[1]), Body.get_forward_rarm(current_cond[2])
  local distL = math.sqrt(
      (trLArm0[1]-trLArm1[1])^2+
      (trLArm0[2]-trLArm1[2])^2+
      (trLArm0[3]-trLArm1[3])^2)
  local distR = math.sqrt(
      (trRArm0[1]-trRArm1[1])^2+
      (trRArm0[2]-trRArm1[2])^2+
      (trRArm0[3]-trRArm1[3])^2)
  print(string.format("%.1f/ %.1f cm, %d steps, %.2fs real time, %.2f ms planning time",
      distL*100,distR*100,
      qArmCount,
      t_robot,
      (t1-t0)*1000 ))
--]]
  
  if debug_on_2 then
    print("trLArm:",self.print_transform( Body.get_forward_larm( qLArmQueue[1][1]  ) ))
    print("trRArm:",self.print_transform( Body.get_forward_rarm( qRArmQueue[1][1]  )))
    print(string.format("TorsoComp: %.3f %.3f",uTorsoCompQueue[1][1],uTorsoCompQueue[1][2]) )
  end


  return qLArmQueue,qRArmQueue, uTorsoCompQueue, qWaistQueue, current_cond, current_param
end



local function plan_arm_sequence(self,arm_seq)
  --This function plans for a arm sequence using multiple arm target positions
  --and initializes the playback if it is possible
  
--  {
--    {'move', trLArm, trRArm}  : move arm transform to target
--    {'wrist', trLArm, trRArm} : rotate wrist to target transform
--    {'valve', }      
--  }
  local t0 =unix.time()


  local init_cond = self:load_boundary_condition()
  local LAPs, RAPs, uTPs, WPs = {},{},{},{}
  local counter = 1

  for i=1,#arm_seq do
    local trLArm = Body.get_forward_larm(init_cond[1])
    local trRArm = Body.get_forward_rarm(init_cond[2])
    local LAP, RAP, uTP,end_cond
    local WP, end_doorparam --for door
    if arm_seq[i][1] =='move' then
      LAP, RAP, uTP, WP, end_cond  = self:plan_unified('move',
        init_cond,
        {trLArm,trRArm},
        {arm_seq[i][2] or trLArm, arm_seq[i][3] or trRArm, arm_seq[i][4], arm_seq[i][5],} )
    elseif arm_seq[i][1] =='wrist' then

      LAP, RAP, uTP, WP, end_cond  = self:plan_unified('wrist',
        init_cond,
        {trLArm,trRArm},
        {arm_seq[i][2] or trLArm,  arm_seq[i][3] or trRArm} )

    elseif arm_seq[i][1] =='door' then
      LAP, RAP, uTP, WP, end_cond, end_doorparam  = self:plan_unified('door',
        init_cond, 
        self.init_doorparam, 
        vector.slice(arm_seq[i],2,#arm_seq[i]) )

    elseif arm_seq[i][1] =='doorleft' then
      LAP, RAP, uTP, WP, end_cond, end_doorparam  = self:plan_unified('doorleft',
        init_cond, 
        self.init_doorparam, 
        vector.slice(arm_seq[i],2,#arm_seq[i]) )

    elseif arm_seq[i][1] =='dooredge' then
      LAP, RAP, uTP, WP, end_cond, end_doorparam  = self:plan_unified('dooredge',
        init_cond, 
        self.init_doorparam, 
        vector.slice(arm_seq[i],2,#arm_seq[i]) )

    elseif arm_seq[i][1] =='valvetwoarm' then
      LAP, RAP, uTP, WP, end_cond, end_valveparam  = self:plan_unified('valvetwoarm',
        init_cond, 
        self.init_valveparam, 
        vector.slice(arm_seq[i],2,#arm_seq[i]) )

    elseif arm_seq[i][1] =='valveonearm' then
      LAP, RAP, uTP, WP, end_cond, end_valveparam  = self:plan_unified('valveonearm',
        init_cond, 
        self.init_valveparam, 
        vector.slice(arm_seq[i],2,#arm_seq[i]) )      

    elseif arm_seq[i][1] =='barvalve' then
      LAP, RAP, uTP, WP, end_cond, end_valveparam  = self:plan_unified('barvalve',
        init_cond, 
        self.init_valveparam, 
        vector.slice(arm_seq[i],2,#arm_seq[i]) )      

    end
    if not LAP then 
      hcm.set_state_success(-1) --Report plan failure
      print("FAIL")
      return 
    end
    init_cond = end_cond    
    if end_doorparam then self.init_doorparam = end_doorparam end
    if end_valveparam then self.init_valveparam = end_valveparam end
    
    for j=1,#LAP do
      LAPs[counter],RAPs[counter],uTPs[counter],WPs[counter]= LAP[j],RAP[j],uTP[j],WP[j]
      counter = counter+1
    end
  end

  self:save_boundary_condition(init_cond)
  local arm_plan = {LAP = LAPs, RAP = RAPs,  uTP =uTPs, WP=WPs}

  filter_arm_plan(arm_plan)
  print_arm_plan(arm_plan)

  self:init_arm_sequence(arm_plan,Body.get_time())

  local t1 =unix.time()
  print(sformat("Total planning time: %.2f ms",(t1-t0)*1000))  


  return true
end






local function init_arm_sequence(self,arm_plan,t0)
  if not arm_plan then return end
  self.leftArmQueue = arm_plan.LAP
  self.rightArmQueue = arm_plan.RAP
  self.torsoCompQueue = arm_plan.uTP
  
  self.t_last = t0
  self.armQueuePlayStartTime = t0
  self.armQueuePlayEndTime = t0 + self.leftArmQueue[2][2]

  self.qLArmStart = self.leftArmQueue[1][1]
  self.qLArmEnd = self.leftArmQueue[2][1]
  self.qRArmStart = self.rightArmQueue[1][1]
  self.qRArmEnd = self.rightArmQueue[2][1]
  self.uTorsoCompStart=vector.new(self.torsoCompQueue[1])
  self.uTorsoCompEnd=vector.new(self.torsoCompQueue[2])

  if arm_plan.WP then
    self.waistQueue = arm_plan.WP
    self.waistStart = self.waistQueue[1]
    self.waistEnd = self.waistQueue[2]
  else
    self.waistQueue = nil
  end
  self.armQueuePlaybackCount = 2


  self:print_segment_info() 
end

local function print_segment_info(self)
  if debug_on then
    print(string.format("%d uTC: %.3f %.3f to %.3f %.3f, t=%.2f",
    self.armQueuePlaybackCount,
    self.uTorsoCompStart[1],self.uTorsoCompStart[2],
    self.uTorsoCompEnd[1],self.uTorsoCompEnd[2],
    self.armQueuePlayEndTime - self.armQueuePlayStartTime 
     ))
  end
end

local function play_arm_sequence(self,t)
  if not self.t_last then return true end
  local dt =  t - self.t_last
  self.t_last = t
  if #self.leftArmQueue < self.armQueuePlaybackCount then
    return true
  else


   --Skip keyframes if needed
    while t>self.armQueuePlayEndTime do        
      self.armQueuePlaybackCount = self.armQueuePlaybackCount +1        
      if #self.leftArmQueue < self.armQueuePlaybackCount then
        --Passed the end of the queue. return the last joint angle
        return 
          self.leftArmQueue[#self.leftArmQueue][1],
          self.rightArmQueue[#self.leftArmQueue][1],
          self.torsoCompQueue[#self.leftArmQueue]
      end
      --Update the frame start and end time
      self.armQueuePlayStartTime = self.armQueuePlayEndTime        
      self.armQueuePlayEndTime = self.armQueuePlayStartTime + 
          self.leftArmQueue[self.armQueuePlaybackCount][2]
      --Update initial and final joint angle
      self.qLArmStart = vector.new(self.leftArmQueue[self.armQueuePlaybackCount-1][1])
      self.qLArmEnd = vector.new(self.leftArmQueue[self.armQueuePlaybackCount][1])
      self.qRArmStart = vector.new(self.rightArmQueue[self.armQueuePlaybackCount-1][1])
      self.qRArmEnd = vector.new(self.rightArmQueue[self.armQueuePlaybackCount][1])
      self.uTorsoCompStart = vector.new(self.torsoCompQueue[self.armQueuePlaybackCount-1])
      self.uTorsoCompEnd = vector.new(self.torsoCompQueue[self.armQueuePlaybackCount])

      if self.waistQueue then
        self.waistStart = self.waistQueue[self.armQueuePlaybackCount-1]
        self.waistEnd = self.waistQueue[self.armQueuePlaybackCount]
      end

      self:print_segment_info()      
    end
    local ph = (t-self.armQueuePlayStartTime)/ 
              (self.armQueuePlayEndTime-self.armQueuePlayStartTime)
--    print(sformat("%d/%d ph:%.2f",self.armQueuePlaybackCount,#self.leftArmQueue,ph))
    local qLArm,qRArm,qWaist={},{}
    for i=1,7 do
      qLArm[i] = self.qLArmStart[i] + ph * (util.mod_angle(self.qLArmEnd[i]-self.qLArmStart[i]))
      qRArm[i] = self.qRArmStart[i] + ph * (util.mod_angle(self.qRArmEnd[i]-self.qRArmStart[i]))
    end
    local uTorsoComp = (1-ph)*self.uTorsoCompStart + ph*self.uTorsoCompEnd


    --Update transform information
    local trLArmComp = Body.get_forward_larm(qLArm)
    local trRArmComp = Body.get_forward_rarm(qRArm)
    local trLArm = vector.new(trLArmComp)+
              vector.new({uTorsoComp[1],uTorsoComp[2],0, 0,0,0})
    local trRArm = vector.new(trRArmComp)+
              vector.new({uTorsoComp[1],uTorsoComp[2],0, 0,0,0})
    hcm.set_hands_left_tr(trLArm)
    hcm.set_hands_right_tr(trRArm)
    hcm.set_hands_left_tr_target(trLArm)
    hcm.set_hands_right_tr_target(trRArm)

    --Move joints
    movearm.setArmJoints(qLArm,qRArm,dt)
    mcm.set_stance_uTorsoComp(uTorsoComp)    

    if self.waistQueue then
      local qWaist={}
      qWaist[1] = self.waistStart[1] + ph * (self.waistEnd[1] - self.waistStart[1])
      qWaist[2] = self.waistStart[2] + ph * (self.waistEnd[2] - self.waistStart[2])
      Body.set_waist_command_position(qWaist)
    end
  end
  return false
end

local function save_boundary_condition(self,arm_end)
  mcm.set_arm_qlarm(arm_end[1])
  mcm.set_arm_qrarm(arm_end[2])        
  mcm.set_arm_qlarmcomp(arm_end[3])
  mcm.set_arm_qrarmcomp(arm_end[4])
end

local function load_boundary_condition(self)
  local qLArm=mcm.get_arm_qlarm()
  local qRArm=mcm.get_arm_qrarm()        
  local qLArmComp=mcm.get_arm_qlarmcomp()
  local qRArmComp=mcm.get_arm_qrarmcomp()
  local uTorsoComp = mcm.get_stance_uTorsoComp()
  local init_cond = {qLArm,qRArm,qLArmComp,qRArmComp,uTorsoComp}
  return init_cond
end

local function save_doorparam(self,doorparam)
  self.init_doorparam=doorparam
end

local function save_valveparam(self,valveparam)
  self.init_valveparam=valveparam
end

local function set_shoulder_yaw_target(self,left,right)
  self.shoulder_yaw_target_left = left
  self.shoulder_yaw_target_right = right
end





local libArmPlan={}

libArmPlan.new_planner = function (params)

  params = params or {}
  local s = {}
  --member variables
  s.armQueue = {}
  s.armQueuePlaybackCount = 1
  s.armQueuePlayStartTime = 0
  s.mLeftHand = 0
  s.mRightHand = 0
  s.shoulder_yaw_target_left = nil
  s.shoulder_yaw_target_right = nil

  s.torsoCompBias = {0,0}



  s.leftArmQueue={}
  s.rightArmQueue={}
  s.torsoCompQueue={}
  s.waistQueue={}

  s.init_cond = {}
  s.current_plan = {}
  s.current_endcond = {}

  s.init_doorparam = {}
  s.init_valveparam = {0,0,0,0}

  --member functions
  s.print_transform = print_transform
  s.print_jangle = print_jangle
  s.print_segment_info = print_segment_info

  s.calculate_margin = calculate_margin
  s.search_shoulder_angle = search_shoulder_angle    
  s.set_hand_mass = set_hand_mass
  s.reset_torso_comp = reset_torso_comp
  s.get_next_movement = get_next_movement

  s.plan_arm_sequence = plan_arm_sequence
  s.plan_arm_sequence2 = plan_arm_sequence
  
  s.plan_unified = plan_unified

  s.init_arm_sequence = init_arm_sequence
  s.play_arm_sequence = play_arm_sequence

  s.save_boundary_condition=save_boundary_condition
  s.load_boundary_condition=load_boundary_condition

  s.save_doorparam = save_doorparam  
  s.save_valveparam = save_valveparam
  s.set_shoulder_yaw_target = set_shoulder_yaw_target

  s.range_test=range_test

  return s
end

return libArmPlan
