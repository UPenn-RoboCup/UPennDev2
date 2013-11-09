--Arm movement planner
--2013/11 SJ

local vector = require'vector'
local util = require'util'
require'mcm'
local movearm = require'movearm'

--Returns transform trajectories from trArm0 to trArm1
--Satisfying 


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
    q[1]*180/math.pi,
    q[2]*180/math.pi,
    q[3]*180/math.pi,
    q[4]*180/math.pi,
    q[5]*180/math.pi,
    q[6]*180/math.pi,
    q[7]*180/math.pi
    )
  return str
end



local function calculate_margin(qArm,isLeft)
  local jointangle_margin
  if not qArm then return -math.huge 


--

  elseif isLeft==1 then --Left arm
    jointangle_margin = math.min(
      --Shoulder Roll: 
      math.abs(qArm[2]-math.pi/2),
      math.abs(qArm[2]),

      --Wrist Roll
      math.abs(qArm[6]-math.pi/2),
      math.abs(qArm[6]+math.pi/2)      
      )
  else --Right arm
    jointangle_margin = math.min(
      --Shoulder Roll
      math.abs(qArm[2]+math.pi/2),
      math.abs(qArm[2]) ,

      --Wrist Roll
      math.abs(qArm[6]-math.pi/2),
      math.abs(qArm[6]+math.pi/2)      
      )
  end
-- 

--[[
  elseif isLeft==1 then --Left arm
    jointangle_margin = math.min(
  
      --Wrist Roll
      math.abs(qArm[6]-math.pi/2),
      math.abs(qArm[6]+math.pi/2)      
      )
  else --Right arm
    jointangle_margin = math.min(
  
      --Wrist Roll
      math.abs(qArm[6]-math.pi/2),
      math.abs(qArm[6]+math.pi/2)      
      )
  end
--]]

  return jointangle_margin
end

local function search_shoulder_angle(self,qArm,trArmNext,isLeft, yawMag)
  local step = 1

  --finer search
  --local step = 0.5

  local margins={} 

  local max_margin = -math.huge
  local qArmMaxMargin



  for div = -1,1,step do
    local qShoulderYaw = qArm[3] + div * yawMag
    local qArmNext
    if isLeft>0 then
     qArmNext = Body.get_inverse_larm(qArm,trArmNext, qShoulderYaw)
    else
     qArmNext = Body.get_inverse_rarm(qArm,trArmNext, qShoulderYaw)
    end
    local margin = self.calculate_margin(qArmNext,isLeft)

--print("Margin",qShoulderYaw,": ",margin)

    if margin>max_margin then
      qArmMaxMargin = qArmNext
      max_margin = margin
    end
  end

--print("shoulderYaw with Maxmargin:",qArmMaxMargin[3])

  if not qArmMaxMargin then
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



local function plan_double_arm_linear(self,qLArm0,qRArm0,qLArmComp0, qRArmComp0, trLArm1, trRArm1, uTorsoComp0)
-- Now we compensate for the COM movement
-- if we reach out, we have to move torso back to compensate
-- and then we need to reach out further to compensate for that movement
-- Here qLArm and qRArm values are the values BEFORE compensation
-- qLArmQueue and qRArmQueue uses the compensated arm positions

  if not qLArm0 or not qRArm0 then return end

  local massL, massR = self.mLeftHand, self.mRightHand
--print("Object mass:",massL, massR)
  local t0 = unix.time()
  local doneL, doneR, failed = false, false, false
  local qLArm,qRArm = qLArm0 , qRArm0
  local qLArmComp, qRArmComp = qLArmComp0, qRArmComp0
  local uTorsoComp = {uTorsoComp0[1],uTorsoComp0[2]}

  local trLArm = Body.get_forward_larm(qLArm0)
  local trRArm = Body.get_forward_rarm(qRArm0)
  local trLArmNext, trRArmNext, qLArmNext, qRArmNext

  local dpArmMax = Config.arm.linear_slow_limit  
  local dt_step = 0.5
  local yawMag = dt_step * 10*math.pi/180

  --Insert initial arm joint angle to the queue
  local qLArmQueue,qRArmQueue, uTorsoCompQueue = {{qLArmComp0,dt_step}}, {{qRArmComp0,dt_step}}, {uTorsoComp0}
  local qArmCount = 2
  
--[[  
  print("Current trL:",self.print_transform(trLArm))
  print("Target trL:",self.print_transform(trLArm1))
--]]

  local uTorsoCompNext
  
  while (not doneL or not doneR) and not failed and not torsoCompDone do        
    trLArmNext, doneL = self.get_next_transform(trLArm,trLArm1,dpArmMax,dt_step)
    trRArmNext, doneR = self.get_next_transform(trRArm,trRArm1,dpArmMax,dt_step)

    --We use this for COM calculation
    qLArmNext = self:search_shoulder_angle(qLArm,trLArmNext,1, yawMag)
    qRArmNext = self:search_shoulder_angle(qRArm,trRArmNext,0, yawMag)
    vec_comp = vector.new({-uTorsoComp[1],-uTorsoComp[2],0,0,0,0})

    local torsoCompDone = true

    local trLArmNextComp = vector.new(trLArmNext) + vec_comp
    local trRArmNextComp = vector.new(trRArmNext) + vec_comp
    
    --Actual arm angle considering the torso compensation
    local qLArmNextComp = self:search_shoulder_angle(qLArmComp,trLArmNextComp,1, yawMag)
    local qRArmNextComp = self:search_shoulder_angle(qRArmComp,trRArmNextComp,0, yawMag)

    if not qLArmNextComp or not qRArmNextComp then 
      if not qLArmNextComp then print("LEFT ERROR") end
      if not qRArmNextComp then print ("RIGHT ERROR") end
      failed = true       
    else      
      local max_movement_ratioL = check_arm_joint_velocity(qLArmComp, qLArmNextComp, dt_step)
      local max_movement_ratioR = check_arm_joint_velocity(qRArmComp, qRArmNextComp, dt_step)
      local dt_step_current = dt_step * math.max(max_movement_ratioL,max_movement_ratioR)

      qLArmQueue[qArmCount] = {qLArmNextComp,dt_step_current}
      qRArmQueue[qArmCount] = {qRArmNextComp,dt_step_current}

      --update the compensation value for next step      

      uTorsoCompNextTarget = self:get_torso_compensation(qLArmNext,qRArmNext,massL,massR)
      local velTorsoComp = {0.005,0.005} --5mm per sec
      uTorsoCompNext, torsoCompDone = util.approachTol(uTorsoComp, uTorsoCompNextTarget, velTorsoComp, dt_step_current )
      uTorsoCompQueue[qArmCount] = {uTorsoCompNext[1],uTorsoCompNext[2]}
--      print(string.format("Com comp: %.3f %.3f",unpack(uTorsoCompNext)))
      qLArm,qRArm,trLArm,trRArm, uTorsoComp = qLArmNext, qRArmNext, trLArmNext, trRArmNext, uTorsoCompNext
      qLArmComp,qRArmComp = qLArmNextComp, qRArmNextComp
      qArmCount = qArmCount + 1
    end
  end

  local t1 = unix.time()  
  print(string.format("%d steps planned, %.2f ms elapsed:",qArmCount,(t1-t0)*1000 ))
  if failed then 
    print("Plan failure at",self.print_transform(trLArmNext))
    print("Plan failure at",self.print_transform(trLArmCompensated))
    print("Arm angle:",unpack(vector.new(qArm)*180/math.pi))
    return
    --return qLArmQueue,qRArmQueue, uTorsoCompQueue, qLArmNext, qRArmNext, qLArmComp, qRArmComp, uTorsoCompNext
  else

    return qLArmQueue,qRArmQueue, uTorsoCompQueue, qLArmNext, qRArmNext, qLArmComp, qRArmComp, uTorsoCompNext
  end
end

local function plan_arm_sequence(self,arm_seq)
  local qLArm = arm_seq.init[1]
  local qRArm = arm_seq.init[2]
  local qLArmComp = arm_seq.init[3]
  local qRArmComp = arm_seq.init[4]
  local uTorsoComp = arm_seq.init[5]
  local LAPs, RAPs, uTPs = {},{},{}
  local counter = 1
  self.mLeftHand = arm_seq.mass[1]
  self.mRightHand = arm_seq.mass[2]

  for i=1,#arm_seq.armseq do
    local LAP, RAP, uTP, qLArm1, qRArm1, qLArmComp1, qRArmComp1, uTorsoComp1
    LAP,RAP,uTP,qLArm1,qRArm1,qLArmComp1, qRArmComp1,uTorsoComp1 = 
      self:plan_double_arm_linear(
        qLArm,qRArm,qLArmComp,qRArmComp,
        arm_seq.armseq[i][1],
        arm_seq.armseq[i][2],
        uTorsoComp)
    if not LAP then return end --failure
    qLArm, qRArm, qLArmComp, qRArmComp,uTorsoComp = 
      qLArm1,qRArm1,qLArmComp1,qRArmComp1,uTorsoComp1
    for j=1,#LAP do
      LAPs[counter],RAPs[counter],uTPs[counter]= 
        LAP[j],RAP[j],uTP[j]
      counter = counter+1
    end
  end
  local arm_plan = {LAP = LAPs, RAP = RAPs,  uTP =uTPs}
  local end_cond={qLArm,qRArm,qLArmComp,qRArmComp,uTorsoComp}
  if qLArm then --plan success
    return arm_plan,end_cond
  else --failure
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

  self.armQueuePlaybackCount = 1
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
--
      print(string.format("%d uTC: %.3f %.3f to %.3f %.3f, t=%.2f",
        self.armQueuePlaybackCount,
        self.uTorsoCompStart[1],self.uTorsoCompStart[2],
        self.uTorsoCompEnd[1],self.uTorsoCompEnd[2],
        self.armQueuePlayEndTime - self.armQueuePlayStartTime 
         ))
--
      local trLArm0 = Body.get_forward_larm(self.qLArmStart)
      local trLArm1 = Body.get_forward_larm(self.qLArmEnd)
--[[
      print(string.format("trLArm: %.3f %.3f %.3f to %.3f %.3f %.3f\n",
        trLArm0[1],trLArm0[2],trLArm0[3],
        trLArm1[1],trLArm1[2],trLArm1[3] ))
--]]
    end
    local ph = (t-self.armQueuePlayStartTime)/ 
              (self.armQueuePlayEndTime-self.armQueuePlayStartTime)
    local qLArm,qRArm={},{}
    for i=1,7 do
      qLArm[i] = self.qLArmStart[i] + ph * (util.mod_angle(self.qLArmEnd[i]-self.qLArmStart[i]))
      qRArm[i] = self.qRArmStart[i] + ph * (util.mod_angle(self.qRArmEnd[i]-self.qRArmStart[i]))
    end
    local uTorsoComp = (1-ph)*self.uTorsoCompStart + ph*self.uTorsoCompEnd
--[[
    local trLArm = Body.get_forward_larm(qLArm)
    print(string.format("KF:%d ph:%.3f TRLARM:%.3f %.3f %.3f uTorsoComp: %.3f %.3f"
      ,self.armQueuePlaybackCount, ph,
      trLArm[1],trLArm[2],trLArm[3],
      uTorsoComp[1],uTorsoComp[2] ))
--]]
    movearm.setArmJoints(qLArm,qRArm,dt)
    mcm.set_stance_uTorsoComp(uTorsoComp)    
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
  s.torsoCompBias = {0,0}

  s.leftArmQueue={}
  s.rightArmQueue={}
  s.torsoCompQueue={}

  --member functions
  s.print_transform = print_transform
  s.print_jangle = print_jangle
  s.calculate_margin = calculate_margin
  s.search_shoulder_angle = search_shoulder_angle
  s.get_next_transform = get_next_transform
  s.get_torso_compensation = get_torso_compensation
  s.set_hand_mass = set_hand_mass
  s.check_arm_joint_velocity = check_arm_joint_velocity
  s.reset_torso_comp = reset_torso_comp

  s.plan_double_arm_linear = plan_double_arm_linear
  s.plan_arm_sequence = plan_arm_sequence
  s.init_arm_sequence = init_arm_sequence
  s.play_arm_sequence = play_arm_sequence
  s.save_boundary_condition=save_boundary_condition
  s.load_boundary_condition=load_boundary_condition

  return s
end

return libArmPlan