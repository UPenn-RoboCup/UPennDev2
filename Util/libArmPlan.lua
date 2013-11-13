--Arm movement planner
--2013/11 SJ

local vector = require'vector'
local util = require'util'
require'mcm'
local movearm = require'movearm'


local function print_transform(tr)
  if not tr then return end
  local str= string.format("%.2f %.2f %.2f (%.1f %.1f %.1f)",
    tr[1],tr[2],tr[3],
    tr[4]*180/math.pi,tr[5]*180/math.pi,tr[6]*180/math.pi
    )
  return str
end

local function print_jangle(q)
  local str= string.format("%d %d %d %d %d %d %d",    
    unpack(vector.new(q)*180/math.pi)  )
  return str
end

local function calculate_margin(qArm,isLeft)
  local jointangle_margin
  if not qArm then return -math.huge 
  elseif isLeft==1 then --Left arm
    jointangle_margin = math.min(
      math.abs(qArm[2]-math.pi/2),       --Shoulder Roll 
      math.abs(qArm[2]),
      math.abs(qArm[6]-math.pi/2),        --Wrist Roll
      math.abs(qArm[6]+math.pi/2)      
      )
  else --Right arm
    jointangle_margin = math.min(
      math.abs(qArm[2]+math.pi/2),       --Shoulder Roll
      math.abs(qArm[2]),
      math.abs(qArm[6]-math.pi/2), --Wrist Roll
      math.abs(qArm[6]+math.pi/2)      
      )
  end

  --Clamp the margin
  jointangle_margin = math.min(
    Config.arm.plan.max_margin,
    jointangle_margin)

  return jointangle_margin
end

local function search_shoulder_angle(self,qArm,trArmNext,isLeft, yawMag, waistYaw)
  local step = Config.arm.plan.search_step

  --Calculte the margin for current shoulder yaw angle
  local qArmMaxMargin, qArmNext
  if isLeft>0 then 
    local shoulderYawTarget = self.shoulder_yaw_target_left
    if shoulderYawTarget then
      local shoulderYaw = util.approachTol(qArm[3],shoulderYawTarget, yawMag, 1)
      qArmNext = Body.get_inverse_larm(qArm,trArmNext, shoulderYaw, mcm.get_stance_bodyTilt(), waistYaw)
      if qArmNext then 
        return qArmNext 
      end
    end
    qArmNext = Body.get_inverse_larm(qArm,trArmNext, qArm[3], mcm.get_stance_bodyTilt(), waistYaw)
  else 
    local shoulderYawTarget = self.shoulder_yaw_target_right    
    if shoulderYawTarget then
      local shoulderYaw = util.approachTol(qArm[3],shoulderYawTarget, yawMag, 1)
      qArmNext = Body.get_inverse_rarm(qArm,trArmNext, shoulderYaw, mcm.get_stance_bodyTilt(), waistYaw)
      if qArmNext then 
        return qArmNext 
      end
    end
    qArmNext = Body.get_inverse_rarm(qArm,trArmNext, qArm[3], mcm.get_stance_bodyTilt(), waistYaw) 
  end

  local max_margin = self.calculate_margin(qArmNext,isLeft)
  qArmMaxMargin = qArmNext

  for div = -1,1,step do
    local qShoulderYaw = qArm[3] + div * yawMag
    local qArmNext
    if isLeft>0 then qArmNext = Body.get_inverse_larm(qArm,trArmNext, qShoulderYaw, mcm.get_stance_bodyTilt(), waistYaw)
    else qArmNext = Body.get_inverse_rarm(qArm,trArmNext, qShoulderYaw, mcm.get_stance_bodyTilt(), waistYaw) end
    local margin = self.calculate_margin(qArmNext,isLeft)
    if margin>max_margin then
      qArmMaxMargin = qArmNext
      max_margin = margin
    end
  end
  if max_margin<0 then
    print("CANNOT FIND CORRECT SHOULDER ANGLE")
    print("trNext:",unpack(trArmNext))
  end
  return qArmMaxMargin
end

local function get_next_transform(trArm, trArmTarget, dpArmMax,dt_step)  
  return util.approachTolTransform(trArm, trArmTarget, dpArmMax, dt_step )
end

local function check_arm_joint_velocity(qArm0, qArm1, dt,velLimit)
  --Slow down the total movement time based on joint velocity limit
  velLimit = velLimit or Config.arm.joint_vel_limit_plan
  local qArmMovement = vector.new(qArm1) - vector.new(qArm0);
  local max_movement_ratio = 1
  for i=1,7 do
    local movement_ratio = math.abs(util.mod_angle(qArmMovement[i]))
      /(velLimit[i]*dt)
    max_movement_ratio = math.max(max_movement_ratio,movement_ratio)
  end
  return max_movement_ratio
end

local function set_hand_mass(self,mLeftHand, mRightHand)
  self.mLeftHand, self.mRightHand = mLeftHand, mRightHand
end

local function reset_torso_comp(self,qLArm,qRArm)
  local qWaist = {0,0}--TODO: will we use waist position as well?
  local com = Kinematics.com_upperbody(qWaist,qLArm,qRArm,
        mcm.get_stance_bodyTilt(), 0,0)        
  self.torsoCompBias = {-com[1]/com[4],-com[2]/com[4]}
end


local function get_torso_compensation(self,qLArm,qRArm,massL,massR)
  local qWaist = {0,0}--TODO: will we use waist position as well?
  if mcm.get_status_iskneeling()==1 or 
    Config.stance.enable_torso_compensation==0 then
    return {0,0}
  else      
    local com = Kinematics.com_upperbody(qWaist,qLArm,qRArm,
       mcm.get_stance_bodyTilt(), massL, massR)
    return {-com[1]/com[4]-self.torsoCompBias[1],-com[2]/com[4]-self.torsoCompBias[2]}
  end
end

local function get_next_movement(self, init_cond, trLArm1,trRArm1, dt_step, waistYaw)

  local velTorsoComp = {0.005,0.005} --5mm per sec
  local velYaw = 10*math.pi/180
  --local velYaw = 20*math.pi/180

  local massL, massR = self.mLeftHand, self.mRightHand
  local qLArm,qRArm, qLArmComp , qRArmComp, uTorsoComp = unpack(init_cond)

  local yawMag = dt_step * velYaw

  local qLArmNext, qRArmNext = qLArm, qRArm
  qLArmNext = self:search_shoulder_angle(qLArm,trLArm1,1, yawMag, {waistYaw,0})
  qRArmNext = self:search_shoulder_angle(qRArm,trRArm1,0, yawMag, {waistYaw,0})

  if not qLArmNext or not qRArmNext then print("ARM PLANNING ERROR")
    return 
  end

  local trLArmNext, trRArmNext = Body.get_forward_larm(qLArmNext),Body.get_forward_rarm(qRArmNext)
  local vec_comp = vector.new({-uTorsoComp[1],-uTorsoComp[2],0,0,0,0})
  local trLArmNextComp = vector.new(trLArmNext) + vec_comp
  local trRArmNextComp = vector.new(trRArmNext) + vec_comp
    
  --Actual arm angle considering the torso compensation
  local qLArmNextComp = self:search_shoulder_angle(qLArmComp,trLArmNextComp,1, yawMag)
  local qRArmNextComp = self:search_shoulder_angle(qRArmComp,trRArmNextComp,0, yawMag)

  if not qLArmNextComp or not qRArmNextComp or not qLArmNext or not qRArmNext then 
    print("ARM PLANNING ERROR")
    return 
  else
    local max_movement_ratioL = check_arm_joint_velocity(qLArmComp, qLArmNextComp, dt_step)
    local max_movement_ratioR = check_arm_joint_velocity(qRArmComp, qRArmNextComp, dt_step)
    local dt_step_current = dt_step * math.max(max_movement_ratioL,max_movement_ratioR)
    --TODO: WAIST 
    local uTorsoCompNextTarget = self:get_torso_compensation(qLArmNext,qRArmNext,massL,massR)
    local uTorsoCompNext, torsoCompDone = util.approachTol(uTorsoComp, uTorsoCompNextTarget, velTorsoComp, dt_step_current )

    local new_cond = {qLArmNext, qRArmNext, qLArmNextComp, qRArmNextComp, uTorsoCompNext}
    return new_cond, dt_step_current, torsoCompDone    
  end
end

local function plan_double_arm_linear(self, init_cond, trLArm1, trRArm1)  
-- Now we compensate for the COM movement
-- if we reach out, we have to move torso back to compensate
-- and then we need to reach out further to compensate for that movement
-- Here qLArm and qRArm values are the values BEFORE compensation
-- qLArmQueue and qRArmQueue uses the compensated arm positions

  if not init_cond then return end
  local doneL, doneR, failed = false, false, false

  --SJ: stupid reference issue if we directly use the init_cond
  local current_cond = {init_cond[1],init_cond[2],init_cond[3],init_cond[4],{init_cond[5][1],init_cond[5][2]}}
  local trLArm, trRArm = Body.get_forward_larm(init_cond[1]), Body.get_forward_rarm(init_cond[2])

  --Insert initial arm joint angle to the queue
  local dt_step0 = Config.arm.plan.dt_step0
  local dt_step = Config.arm.plan.dt_step
  local qLArmQueue,qRArmQueue, uTorsoCompQueue = {{init_cond[3],dt_step0}}, {{init_cond[4],dt_step0}}, {init_cond[5]}
  local qArmCount = 2

  local dpArmMax = Config.arm.linear_slow_limit  
  local torsoCompDone = false
  local trLArmNext, trRArmNext

  local t0 = unix.time()  
  while not (doneL and doneR and torsoCompDone) and not failed  do        
    trLArmNext, doneL = self.get_next_transform(trLArm,trLArm1,dpArmMax,dt_step)
    trRArmNext, doneR = self.get_next_transform(trRArm,trRArm1,dpArmMax,dt_step)
    local new_cond, dt_step_current
    new_cond, dt_step_current, torsoCompDone = 
      self:get_next_movement(current_cond, trLArmNext, trRArmNext, dt_step)
    if not new_cond then 
      print("FAIL")
      --TODO: We can just skip singularities
      failed = true    
    else
      qLArmQueue[qArmCount] = {new_cond[3],dt_step_current}
      qRArmQueue[qArmCount] = {new_cond[4],dt_step_current}
      uTorsoCompQueue[qArmCount] = {new_cond[5][1],new_cond[5][2]}      
      current_cond = new_cond
      qArmCount = qArmCount + 1
    end    
  end

  local t1 = unix.time()  
  print(string.format("%d steps planned, %.2f ms elapsed:",qArmCount,(t1-t0)*1000 ))
  if failed then 
    print("Plan failure at",self.print_transform(trLArmNext))
    print("Arm angle:",unpack(vector.new(qArm)*180/math.pi))
    return
  else      
    return qLArmQueue,qRArmQueue, uTorsoCompQueue, current_cond
  end
end



local function plan_double_wrist(self, init_cond, trLArm1, trRArm1)  
--We fix the wrist position and just rotate the wrist 
  if not init_cond then return end
  local doneL, doneR, failed = false, false, false

  --SJ: stupid reference issue if we directly use the init_cond
  local current_cond = {init_cond[1],init_cond[2],init_cond[3],init_cond[4],{init_cond[5][1],init_cond[5][2]}}
  local trLArm, trRArm = Body.get_forward_larm(init_cond[1]), Body.get_forward_rarm(init_cond[2])
  local qLArm0, qRArm0 = init_cond[1],init_cond[2]

  --Insert initial arm joint angle to the queue
  local dt_step0 = Config.arm.plan.dt_step0
  local dt_step = Config.arm.plan.dt_step
  local qLArmQueue,qRArmQueue, uTorsoCompQueue = {{init_cond[3],dt_step0}}, {{init_cond[4],dt_step0}}, {init_cond[5]}
  local qArmCount = 2

--  local dpArmMax = Config.arm.linear_slow_limit  
  local dpArmMax = Config.arm.plan.velWrist

  local torsoCompDone = false
  local trLArmNext, trRArmNext

  local t0 = unix.time()  
  while not (doneL and doneR and torsoCompDone) and not failed  do   
    trLArmNext,doneL = util.approachTolTransform(trLArm, trLArm1, dpArmMax, dt_step )
    trRArmNext,doneR = util.approachTolTransform(trRArm, trRArm1, dpArmMax, dt_step )
    local qLArmTemp = Body.get_inverse_arm_given_wrist( qLArm0, trLArmNext)
    local qRArmTemp = Body.get_inverse_arm_given_wrist( qRArm0, trRArmNext)
    trLArmNext = Body.get_forward_larm(qLArmTemp)
    trRArmNext = Body.get_forward_rarm(qRArmTemp)  

    local new_cond, dt_step_current
    new_cond, dt_step_current, torsoCompDone = 
      self:get_next_movement(current_cond, trLArmNext, trRArmNext, dt_step)
    if not new_cond then failed = true       
    else
      qLArmQueue[qArmCount] = {new_cond[3],dt_step_current}
      qRArmQueue[qArmCount] = {new_cond[4],dt_step_current}
      uTorsoCompQueue[qArmCount] = {new_cond[5][1],new_cond[5][2]}      
    end
    current_cond = new_cond
    qArmCount = qArmCount + 1
  end
  local t1 = unix.time()  
  print(string.format("%d steps planned, %.2f ms elapsed:",qArmCount,(t1-t0)*1000 ))
  if failed then 
    print("Plan failure at",self.print_transform(trLArmNext))
    print("Arm angle:",unpack(vector.new(qArm)*180/math.pi))
    return
  else      
    return qLArmQueue,qRArmQueue, uTorsoCompQueue, current_cond
  end
end


local function plan_opendoor(self, init_cond, init_doorparam, doorparam)  
  if not init_cond then return end
  local done1, done2, done3, done4, failed = false, false, false, false, false
  local current_cond = {init_cond[1],init_cond[2],init_cond[3],init_cond[4],{init_cond[5][1],init_cond[5][2]}}
  local trLArm, trRArm = Body.get_forward_larm(init_cond[1]), Body.get_forward_rarm(init_cond[2])
 
  --Insert initial arm joint angle to the queue
  local dt_step0 = Config.arm.plan.dt_step0
  local dt_step = Config.arm.plan.dt_step
  local qLArmQueue,qRArmQueue, uTorsoCompQueue = {{init_cond[3],dt_step0}}, {{init_cond[4],dt_step0}}, {init_cond[5]}
  local qWaistQueue={init_cond[6] or Body.get_waist_command_position()[1]}
  local qArmCount = 2

  local dpArmMax = Config.arm.linear_slow_limit  
  local torsoCompDone = false  

  local dpDoorMax = vector.slice(Config.arm.linear_slow_limit,1,3)
  local dqDoorRollMax = Config.arm.plan.velDoorRoll
  local dqDoorYawMax = Config.arm.plan.velDoorYaw
  local dqWaistYawMax = 3*Body.DEG_TO_RAD

  local t0 = unix.time()  

  doorparam[4] = doorparam[4] or init_doorparam[4]
  
  local current_doorparam={
    {init_doorparam[1][1],init_doorparam[1][2],init_doorparam[1][3]},
    init_doorparam[2],init_doorparam[3],init_doorparam[4]
  }

  while not (done1 and done2 and done3 and done4 and torsoCompDone) and not failed do
    current_doorparam[1], done1 = util.approachTol(current_doorparam[1],doorparam[1], dpDoorMax,dt_step)
    current_doorparam[2], done2 = util.approachTol(current_doorparam[2],doorparam[2], dqDoorRollMax,dt_step)
    current_doorparam[3], done3 = util.approachTol(current_doorparam[3],doorparam[3], dqDoorYawMax,dt_step)
    current_doorparam[4], done4 = util.approachTol(current_doorparam[4],doorparam[4], dqWaistYawMax,dt_step)
    waistNext = current_doorparam[4]
    
    local trLArmNext = trLArm
    local trRArmNext = movearm.getDoorHandlePosition(current_doorparam[1],current_doorparam[2],current_doorparam[3])
    
    local new_cond, dt_step_current
    new_cond, dt_step_current, torsoCompDone = 
      self:get_next_movement(current_cond, trLArmNext, trRArmNext, dt_step, waistNext)
    if not new_cond then 
      print("FAIL at:",doorparam[3]*180/math.pi)
      failed = true       
    else
      qLArmQueue[qArmCount] = {new_cond[3],dt_step_current}
      qRArmQueue[qArmCount] = {new_cond[4],dt_step_current}
      uTorsoCompQueue[qArmCount] = {new_cond[5][1],new_cond[5][2]}
      qWaistQueue[qArmCount] = waistNext 
    end
    current_cond = new_cond
    qArmCount = qArmCount + 1
  end

  local t1 = unix.time()  
  print(string.format("%d steps planned, %.2f ms elapsed:",qArmCount,(t1-t0)*1000 ))
  if failed then 
    print("Plan failure at",self.print_transform(trLArmNext))
    print("Arm angle:",unpack(vector.new(qArm)*180/math.pi))
    return
  else      
    return qLArmQueue,qRArmQueue, uTorsoCompQueue, qWaistQueue, current_cond, current_doorparam
  end
end









local function plan_open_door_sequence(self,doorparam)
  local init_cond = self:load_boundary_condition()
  local LAPs, RAPs, uTPs, WPs = {},{},{},{}
  local counter = 1

  for i=1,#doorparam do
    local LAP, RAP, uTP, WP, end_cond, end_doorparam  = self:plan_opendoor(
      init_cond, self.init_doorparam, doorparam[i])
    if not LAP then return end
    init_cond = end_cond
    self.init_doorparam = end_doorparam
    for j=1,#LAP do
      LAPs[counter],RAPs[counter],uTPs[counter],WPs[counter]= 
        LAP[j],RAP[j],uTP[j],WP[j]
      counter = counter+1
    end
  end

  if init_cond then --plan success    
    print("Arm plan success")
--  for i=1,#RAPs do print(i,":",self.print_jangle(RAPs[i][1] )) end
    self:save_boundary_condition(init_cond)
    local arm_plan = {LAP = LAPs, RAP = RAPs,  uTP =uTPs, WP=WPs}
    self:init_arm_sequence(arm_plan,Body.get_time())
    return true
  else --failure
    print("Arm plan failure!!!")
    return
  end
end

local function plan_arm_sequence(self,arm_seq)
  --This function plans for a arm sequence using multiple arm target positions
  --and initializes the playback if it is possible
  if arm_seq.mass then 
    self.mLeftHand,self.mRightHand = arm_seq.mass[1], arm_seq.mass[2]
  end
  local init_cond = self:load_boundary_condition()
  local LAPs, RAPs, uTPs = {},{},{}
  local counter = 1

  for i=1,#arm_seq.armseq do
    local LAP, RAP, uTP, end_cond  = self:plan_double_arm_linear(
      init_cond,arm_seq.armseq[i][1],arm_seq.armseq[i][2])
    if not LAP then return end
    init_cond = end_cond
    for j=1,#LAP do
      LAPs[counter],RAPs[counter],uTPs[counter]= LAP[j],RAP[j],uTP[j]
      counter = counter+1
    end
  end
  if init_cond then --plan success    
    self:save_boundary_condition(init_cond)
    local arm_plan = {LAP = LAPs, RAP = RAPs,  uTP =uTPs}
    self:init_arm_sequence(arm_plan,Body.get_time())
    return true
  else --failure
    print("Arm plan failure!!!")
    return
  end
end

local function plan_wrist_sequence(self,arm_seq)
  local init_cond = self:load_boundary_condition()
  local LAPs, RAPs, uTPs = {},{},{}
  local counter = 1
  for i=1,#arm_seq.armseq do
    local LAP, RAP, uTP, end_cond  = self:plan_double_wrist(
      init_cond,arm_seq.armseq[i][1],arm_seq.armseq[i][2])
    if not LAP then return end
    init_cond = end_cond
    for j=1,#LAP do
      LAPs[counter],RAPs[counter],uTPs[counter]= LAP[j],RAP[j],uTP[j]
      counter = counter+1
    end
  end
  if init_cond then --plan success    
    self:save_boundary_condition(init_cond)
    local arm_plan = {LAP = LAPs, RAP = RAPs,  uTP =uTPs}
    self:init_arm_sequence(arm_plan,Body.get_time())
    return true
  else --failure
    print("Arm plan failure!!!")
    return
  end
end

local function init_arm_sequence(self,arm_plan,t0)
  if not arm_plan then return end
  self.leftArmQueue = arm_plan.LAP
  self.rightArmQueue = arm_plan.RAP
  self.torsoCompQueue = arm_plan.uTP
  

  self.t_last = t0
  self.armQueuePlayStartTime = t0
  self.armQueuePlayEndTime = t0 + self.leftArmQueue[1][2]

  self.qLArmStart = self.leftArmQueue[1][1]
  self.qLArmEnd = self.leftArmQueue[1][1]
  self.qRArmStart = self.rightArmQueue[1][1]
  self.qRArmEnd = self.rightArmQueue[1][1]
  self.uTorsoCompStart=vector.new(self.torsoCompQueue[1])
  self.uTorsoCompEnd=vector.new(self.torsoCompQueue[1])

  if arm_plan.WP then
    self.waistQueue = arm_plan.WP
    self.waistStart = self.waistQueue[1]
    self.waistEnd = self.waistQueue[1]
  else
    self.waistQueue = nil
  end

  self.armQueuePlaybackCount = 1

  self:print_segment_info() 
end

local function print_segment_info(self)
  print(string.format("%d uTC: %.3f %.3f to %.3f %.3f, t=%.2f",
    self.armQueuePlaybackCount,
    self.uTorsoCompStart[1],self.uTorsoCompStart[2],
    self.uTorsoCompEnd[1],self.uTorsoCompEnd[2],
    self.armQueuePlayEndTime - self.armQueuePlayStartTime 
     ))


--  print("RArm:",self.print_jangle(self.qRArmStart))
end

local function play_arm_sequence(self,t)
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
      qWaist = self.waistStart + ph * (self.waistEnd - self.waistStart)
      Body.set_waist_command_position({qWaist,0})
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

  --member functions
  s.print_transform = print_transform
  s.print_jangle = print_jangle
  s.print_segment_info = print_segment_info

  s.calculate_margin = calculate_margin
  s.search_shoulder_angle = search_shoulder_angle
  s.get_next_transform = get_next_transform
  s.get_torso_compensation = get_torso_compensation
  s.set_hand_mass = set_hand_mass
  s.check_arm_joint_velocity = check_arm_joint_velocity
  s.reset_torso_comp = reset_torso_comp

  s.get_next_movement = get_next_movement
  s.plan_double_arm_linear = plan_double_arm_linear
  s.plan_arm_sequence = plan_arm_sequence
  s.init_arm_sequence = init_arm_sequence
  s.play_arm_sequence = play_arm_sequence

  s.plan_open_door_sequence = plan_open_door_sequence
  s.plan_opendoor = plan_opendoor

  s.plan_wrist_sequence = plan_wrist_sequence
  s.plan_double_wrist = plan_double_wrist

  s.save_boundary_condition=save_boundary_condition
  s.load_boundary_condition=load_boundary_condition

  s.save_doorparam = save_doorparam  
  s.set_shoulder_yaw_target = set_shoulder_yaw_target

  return s
end

return libArmPlan