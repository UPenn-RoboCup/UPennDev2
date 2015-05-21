-- libArmPlan
-- (c) 2013 Seung-Joon Yi
-- Arm movement planner

local vector = require'vector'
local util = require'util'
require'mcm'

local movearm = require'movearm'
local libTransform = require'libTransform'
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

  
  local min_vel = 0.02 --min speed: 2 cm/s
--  local max_vel = 0.04 --max speed: 8 cm/s
  local max_vel = 0.02 --max speed: 8 cm/s


  local ramp1 = 0.04 --accellerate over 4cm
  local ramp2 = 0.04 --desccellerate over 4cm

  local vel = math.min(
    max_vel,
    min_vel + (max_vel-min_vel) * (cdist/ramp1), 
    min_vel + (max_vel-min_vel) * ((dist-cdist)/ramp2) 
    )
  vel = math.max(min_vel,vel)
  return vel
end


local function print_arm_plan(arm_plan)
  print("Arm plan: total ", #arm_plan.LAP)  
  --  local arm_plan = {LAP = LAPs, RAP = RAPs,  uTP =uTPs, WP=WPs}
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
    dt[i] = math.max(dt_min_left, dt_min_right,Config.arm.plan.dt_step_min_jacobian)
--    dt[i] = math.max(dt_min_left, dt_min_right)
    qLArmVel[i] = qLArmMov[i]/dt[i]    
    qRArmVel[i] = qLArmMov[i]/dt[i]
  end

  local total_time1,total_time2=0,0

  for i=1,num-1 do 
    total_time1=total_time1+plan.LAP[i][2]
    plan.LAP[i][2] = dt[i] 
    total_time2 = total_time2+dt[i]
  end

  local t1 =unix.time()
  print(sformat("%d segments, Filtering time: %.2f ms\nExecution time Org: %.fs : Filtered %.1fs",
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
    com = K.calculate_com_pos(qWaist,qLArm,qRArm,qLLeg,qRLeg,0,0,0,Config.birdwalk or 0)
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

local function get_armangle_jacobian(self,qArm,trArmTarget,isLeft,  qWaist,dt_step, linear_vel, debug)  
  if not qArm or not trArmTarget then return end
  local handOffset = Config.arm.handoffset.right
  local trArm
  
  if isLeft ==1 then 
    handOffset=Config.arm.handoffset.left    
    trArm= Body.get_forward_larm(qArm)
  else
    trArm= Body.get_forward_rarm(qArm)
  end
  local JacArm=K.calculate_arm_jacobian(
    qArm,qWaist,
    {0,0,0}, --rpy angle
    isLeft,
    handOffset[1],handOffset[2],handOffset[3]
    );  --tool xyz
  local trArmDiff = util.diff_transform(trArmTarget,trArm)  

--Calculate target velocity
  local trArmVelTarget={
    0,0,0,
    util.procFunc(-trArmDiff[4],0,15*math.pi/180),
    util.procFunc(-trArmDiff[5],0,15*math.pi/180),
    util.procFunc(-trArmDiff[6],0,15*math.pi/180),
  }  
  local linear_dist = util.norm(trArmDiff,3)
  local total_angular_vel = 
     math.abs(trArmVelTarget[4])+math.abs(trArmVelTarget[5])+math.abs(trArmVelTarget[6])
  
  if linear_dist>0 then
    trArmVelTarget[1],trArmVelTarget[2],trArmVelTarget[3]=
    trArmDiff[1]/linear_dist *linear_vel,
    trArmDiff[2]/linear_dist *linear_vel,
    trArmDiff[3]/linear_dist *linear_vel    
  end
    
  if linear_dist<0.001 and total_angular_vel<1*math.pi/180 then 
    return qArm,true,1
  end --reached

  local J= torch.Tensor(JacArm):resize(6,7)  
  local JT = torch.Tensor(J):transpose(1,2)
  local e = torch.Tensor(trArmVelTarget)
  local I = torch.Tensor():resize(7,7):zero()
  local I2 = torch.Tensor():resize(7,6):zero()
  local qArmVel = torch.Tensor(7):fill(0)
  
  -- lambda_i = c*((2*q-qmin-qmax)/(qmax-qmin))^p + (1/w_i)
  -- todo: hard limit joint angles

  local lambda=torch.eye(7)
  local c, p = 2,10  

  local joint_limits={
    {-math.pi/2*100/180, math.pi*200/180},
    {0,math.pi/2},
    {-math.pi/2, math.pi/2},
    {-math.pi, -0.2}, --temp value
    {-math.pi*1.5, math.pi*1.5},
    {-math.pi/2, math.pi/2},
    {-math.pi, math.pi}
  }
  if isLeft==0 then
    joint_limits[2]={-math.pi/2,0}
--    joint_limits[2]={-math.pi/180*135,0}
    joint_limits[5]={-math.pi*1.5, math.pi*1.5}
  end

  for i=1,7 do
    lambda[i][i]=0.1*0.1 + c*
      ((2*qArm[i]-joint_limits[i][1]-joint_limits[i][2])/
       (joint_limits[i][2]-joint_limits[i][1]))^p
  end

  I:addmm(JT,J):add(1,lambda)
  local Iinv=torch.inverse(I)  
  I2:addmm(Iinv,JT)   
  qArmVel:addmv(I2,e)

  local jacobianVelFactor = Config.arm.plan.jacobianVelFactor or 1

  local qArmTarget = vector.new(qArm)+vector.new(qArmVel)*dt_step*jacobianVelFactor

  local trArmNext = Body.get_forward_rarm(qArmTarget)
  local trArmDiffActual = util.diff_transform(trArmNext,trArm)
  local linearDistActual = util.norm(trArmDiffActual,3)
  local angularDistActual = 
    math.abs(trArmDiffActual[4])+math.abs(trArmDiffActual[5])+math.abs(trArmDiffActual[6])

  local linearVelActual = linearDistActual/dt_step
  local angularVelActual = angularDistActual/dt_step

  if linearVelActual<0.001 and angularVelActual<1*DEG_TO_RAD then
    print("Movement stuck")
    return
  end

--  if debug and isLeft==0 then
  if false then
    print(util.print_transform(trArmNext,3).." => "..util.print_transform(trArmTarget,3))
    print(sformat("T dist:%.3f Movement: %.3f vel:T%.3f A:%.3f ( %.1f percent)",
        linear_dist,linearDistActual, linear_vel,
        linearVelActual, linearVelActual/linear_vel*100 )
      )
  end

  return qArmTarget,false, linearVelActual/linear_vel
end


local function get_next_movement_jacobian(self, init_cond, trLArm1,trRArm1, dt_step, waistYaw, waistPitch, velL, velR)

  local default_hand_mass = Config.arm.default_hand_mass or 0
  local dqVelLeft,dqVelRight = mcm.get_arm_dqVelLeft(),mcm.get_arm_dqVelRight()    
  local massL, massR = self.mLeftHand + default_hand_mass, self.mRightHand + default_hand_mass

  local trLArm, trRArm, qLArmComp, qRArmComp, uTorsoComp = unpack(init_cond)

  local qLArmNextComp, qRArmNextComp = qLArmComp, qRArmComp
  local qWaist = {waistYaw, waistPitch}

  local endpoint_compensation = mcm.get_arm_endpoint_compensation()
  local doneL,doneR,scaleL,scaleR = true,true,1,1

  local uTorsoCompNext = get_torso_compensation(qLArmComp,qRArmComp,qWaist, massL,massR)
  local vec_comp = vector.new({uTorsoCompNext[1],uTorsoCompNext[2],0,0,0,0})

  --arm transform from torso frame (which moves around for compensation)
  local trLArm1Comp = vector.new(trLArm1) - vec_comp
  local trRArm1Comp = vector.new(trRArm1) - vec_comp

  if endpoint_compensation[1]>0 then
    qLArmNextComp,doneL,scaleL = self:get_armangle_jacobian(qLArmComp,trLArm1Comp, 1, qWaist,dt_step, velL,true)
  end
  if endpoint_compensation[2]>0 then
    qRArmNextComp,doneR,scaleR = self:get_armangle_jacobian(qRArmComp,trRArm1Comp, 0, qWaist,dt_step, velR,true)
  end

  if not qLArmNextComp or not qRArmNextComp then
    print("ERROR: Arm STUCK")
    return 
  end

  --arm transform in fixed frame
  local trLArmNext = Body.get_forward_larm(qLArmNextComp,mcm.get_stance_bodyTilt(),qWaist) + vec_comp  
  local trRArmNext = Body.get_forward_rarm(qRArmNextComp,mcm.get_stance_bodyTilt(),qWaist) + vec_comp
  
  local new_cond = {trLArmNext, trRArmNext, qLArmNextComp, qRArmNextComp, uTorsoCompNext, waistYaw, waistPitch}

  local scale=math.max(Config.arm.plan.scale_limit[1], math.min(scaleL,scaleR,Config.arm.plan.scale_limit[2]))
--print("scale:",scale)

  return new_cond, dt_step*scale, doneL and doneR
end


local function plan_unified(self, plantype, init_cond, init_param, target_param)
  local dpVelLeft = mcm.get_arm_dpVelLeft()
  local dpVelRight = mcm.get_arm_dpVelRight()
  local t00 = unix.time()
  if not init_cond then return end

  --local init_cond = {trLArm, trRArm, qLArmComp, qRArmComp, uTorsoComp}

  --param: {trLArm,trRArm} for move
  --param: {trLArm,trRArm} for wrist


  local done, failed = false, false, false

  local trLArm,trRArm = init_cond[1],init_cond[2]
  local qLArmComp0, qRArmComp0 = init_cond[3],init_cond[4]

  local qWaist = {init_cond[6] or Body.get_waist_command_position()[1],init_cond[7] or Body.get_waist_command_position()[2]}
  local current_cond = {init_cond[1],init_cond[2],init_cond[3],init_cond[4],{init_cond[5][1],init_cond[5][2]},qWaist[1],qWaist[2]}
  
  --Insert initial arm joint angle to the queue
  local dt_step0 = Config.arm.plan.dt_step0_jacobian
  local dt_step = Config.arm.plan.dt_step_jacobian

  local qLArmQueue,qRArmQueue, uTorsoCompQueue, qWaistQueue = 
    {{init_cond[3],dt_step0}}, {{init_cond[4],dt_step0}},  {init_cond[5]}, {{current_cond[6],current_cond[7]}}

  local current_param={unpack(init_param)}
  local qArmCount = 2
  local vel_param  

  if plantype=="move" then
    --waist movement
    current_param[3],current_param[4] = current_cond[6],current_cond[7]
    target_param[3],target_param[4] = target_param[3] or current_cond[6],target_param[4] or current_cond[7]
    vel_param = {dpVelLeft,dpVelRight,Config.arm.vel_waist_limit[1],Config.arm.vel_waist_limit[2]}
  elseif plantype=="wrist" then
  end

  
  local done, torsoCompDone = false, false
  local trLArmNext, trRArmNext, waistNext
  local new_param = {}

  local t0 = unix.time()  
  local t_robot = 0
  local done2 = false

  while not done and not failed  do --we were skipping the last frame
    local t01 = unix.time()    

    local plan_timeout = 1.0

    if t01-t00>plan_timeout then
      print("Arm planning stuck")
      return
    end
    local new_cond, dt_step_current, torsoCompDone, t10, t11

    local distL = tr_dist(init_param[1],target_param[1])
    local distR = tr_dist(init_param[2],target_param[2])
    if plantype=="move" then
      trLArmNext,trRArmNext = target_param[1],target_param[2]

      local cdistL = tr_dist(init_param[1],trLArm)
      local cdistR = tr_dist(init_param[2],trRArm)
      local velL,velR = movfunc(cdistL,distL),movfunc(cdistR,distR)



      velL, velR = 0.01,0.01 --constant speed


      --Waist yaw and pitch
      new_param[3],done3 = util.approachTol(current_param[3],target_param[3],vel_param[3],dt_step )
      new_param[4],done4 = util.approachTol(current_param[4],target_param[4],vel_param[4],dt_step )
      waistNext = {new_param[3], new_param[4]}
  
      t10 = unix.time() 
      new_cond, dt_step_current, done  = self:get_next_movement_jacobian(
          current_cond, trLArmNext,trRArmNext, dt_step, waistNext[1], waistNext[2], velL, velR)
      if new_cond then trLArmNext, trRArmNext, torsoCompDone = new_cond[1], new_cond[2],new_cond[5] end

      t11 = unix.time() 

    elseif plantype=="wrist" then

      trLArmNext,doneL = util.approachTolWristTransform(trLArm, target_param[1], dpVelLeft, dt_step )      
      trRArmNext,doneR = util.approachTolWristTransform(trRArm, target_param[2], dpVelRight, dt_step )
      done = doneL and doneR
      local velL,velR = 0.02, 0.02 --for some reason, accelleration does not work very well with this

      local cdistL = tr_dist(init_param[1],trLArmNext)
      local cdistR = tr_dist(init_param[2],trRArmNext)

--hack
      trLArmNext,trRArmNext = target_param[1],target_param[2]
      waistNext = {current_cond[6], current_cond[7]}


--[[
      local qLArmTemp = Body.get_inverse_arm_given_wrist( current_cond[3], trLArmNext)
      local qRArmTemp = Body.get_inverse_arm_given_wrist( current_cond[4], trRArmNext)
      trLArmNext = Body.get_forward_larm(qLArmTemp)
      trRArmNext = Body.get_forward_rarm(qRArmTemp)  
--]]      
     

      t10 = unix.time() 
      new_cond, dt_step_current, torsoCompDone=    
        self:get_next_movement_jacobian(current_cond, trLArmNext, trRArmNext, dt_step, waistNext[1], waistNext[2],velL,velR)
      t11 = unix.time()
    end

    done = done and torsoCompDone
    if not new_cond then       
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
  
  if debug_on_2 then
    print("trLArm:",self.print_transform( Body.get_forward_larm( qLArmQueue[1][1]  ) ))
    print("trRArm:",self.print_transform( Body.get_forward_rarm( qRArmQueue[1][1]  )))
    print(string.format("TorsoComp: %.3f %.3f",uTorsoCompQueue[1][1],uTorsoCompQueue[1][2]) )
  end


  return qLArmQueue,qRArmQueue, uTorsoCompQueue, qWaistQueue, current_cond, current_param
end



local function plan_arm_sequence(self,arm_seq, current_stage_name,next_stage_name)
  --This function plans for a arm sequence using multiple arm target positions
  --and initializes the playback if it is possible
  
--  {
--    {'move', trLArm, trRArm}  : move arm transform to target
--    {'wrist', trLArm, trRArm} : rotate wrist to target transform
--    {'valve', }      
--  }
  local t0 =unix.time()
  local init_cond = self:load_boundary_condition()
  print("init cond:")
  print(util.print_transform(init_cond[1]))

  local LAPs, RAPs, uTPs, WPs = {},{},{},{}
  local counter = 1

  for i=1,#arm_seq do
    local trLArm, trRArm = init_cond[1],init_cond[2]
    local LAP, RAP, uTP,end_cond
    local WP, end_doorparam --for door
    if arm_seq[i][1] =='move' then
      LAP, RAP, uTP, WP, end_cond  = self:plan_unified('move',
        init_cond, {trLArm,trRArm}, {arm_seq[i][2] or trLArm, arm_seq[i][3] or trRArm, arm_seq[i][4], arm_seq[i][5]} )

    elseif arm_seq[i][1] =='move0' then
      local trLArmTarget,trRArmTarget = trLArm,trRArm
      local lOffset,rOffset = mcm.get_arm_lhandoffset(),mcm.get_arm_rhandoffset()
      if arm_seq[i][2] then trLArmTarget = libTransform.trans6D(arm_seq[i][2],lOffset) end 
      if arm_seq[i][3] then trRArmTarget = libTransform.trans6D(arm_seq[i][3],rOffset) end
      LAP, RAP, uTP, WP, end_cond  = self:plan_unified('move',
        init_cond,   {trLArm,trRArm}, {trLArmTarget,trRArmTarget, arm_seq[i][4], arm_seq[i][5]} )

    elseif arm_seq[i][1] =='wrist' then
      LAP, RAP, uTP, WP, end_cond  = self:plan_unified('wrist',
        init_cond, {trLArm,trRArm}, {arm_seq[i][2] or trLArm,  arm_seq[i][3] or trRArm} )
    end
    if not LAP then 
      hcm.set_state_success(-1) --Report plan failure
      print("PLAN FAIL")
      return nil,current_stage_name
    end
    init_cond = end_cond    
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
  return true, next_stage_name
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

local function reset_torso_comp(self,qLArmComp,qRArmComp)
  local qWaist = Body.get_waist_command_position()
  local uTorsoComp = get_torso_compensation(qLArmComp,qRArmComp,qWaist, 0,0)
  self.torsoCompBias = uTorsoComp
  mcm.set_stance_uTorsoCompBias(self.torsoCompBias)  

  local vec_comp = vector.new({uTorsoComp[1],uTorsoComp[2],0,0,0,0})
  local trLArmComp = Body.get_forward_larm(qLArmComp)
  local trRArmComp = Body.get_forward_rarm(qRArmComp)
  local trLArm = vector.new(trLArmComp) + vec_comp
  local trRArm = vector.new(trRArmComp) + vec_comp 

  self:save_boundary_condition({trLArm, trRArm, qLArmComp, qRArmComp, uTorsoComp})
end

local function save_boundary_condition(self,arm_end)
  mcm.set_arm_trlarm(arm_end[1])
  mcm.set_arm_trrarm(arm_end[2])
  mcm.set_arm_qlarmcomp(arm_end[3])
  mcm.set_arm_qrarmcomp(arm_end[4])
  mcm.set_stance_uTorsoComp(arm_end[5])
end

local function load_boundary_condition(self)
  local trLArm = mcm.get_arm_trlarm()
  local trRArm = mcm.get_arm_trrarm()
  local qLArmComp=mcm.get_arm_qlarmcomp()
  local qRArmComp=mcm.get_arm_qrarmcomp()
  local uTorsoComp = mcm.get_stance_uTorsoComp()
  local init_cond = {trLArm, trRArm, qLArmComp, qRArmComp, uTorsoComp}
  return init_cond
end

local function load_current_condition(self)
  local trLArm = mcm.get_arm_trlarm()
  local trRArm = mcm.get_arm_trrarm()
  local qLArmComp=mcm.get_arm_qlarmcomp()
  local qRArmComp=mcm.get_arm_qrarmcomp()
  local uTorsoComp = mcm.get_stance_uTorsoComp()
  return trLArm, trRArm, qLArmComp, qRArmComp, uTorsoComp
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
  s.load_current_condition=load_current_condition

  s.save_doorparam = save_doorparam  
  s.save_valveparam = save_valveparam
  s.set_shoulder_yaw_target = set_shoulder_yaw_target

  s.range_test=range_test

  s.get_armangle_jacobian=get_armangle_jacobian
  s.get_next_movement_jacobian=get_next_movement_jacobian
  return s
end

return libArmPlan
