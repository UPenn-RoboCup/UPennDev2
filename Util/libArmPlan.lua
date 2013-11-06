--Arm movement planner
--2013/11 SJ

local vector = require'vector'
local util = require'util'
require'mcm'
local movearm = require'movearm'

--Returns transform trajectories from trArm0 to trArm1
--Satisfying 


local function print_transform(tr)
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
  return jointangle_margin
end

local function search_shoulder_angle(self,qArm,trArmNext,isLeft, yawMag)
  local step = 1
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
    if margin>max_margin then
      qArmMaxMargin = qArmNext
      margin = max_margin
    end
  end
  return qArmMaxMargin
end

local function get_next_transform(trArm, trArmTarget, dpArmMax,dt_step)
  --TODO: singularity rejection  
  return util.approachTolTransform(trArm, trArmTarget, dpArmMax, dt_step )
end

local function check_arm_joint_velocity(qArm0, qArm1, dt,velLimit)
  --Slow down the total movement time based on joint velocity limit
  velLimit = velLimit or vector.new({10,10,10,10,30,10,30})*Body.DEG_TO_RAD 
  local qArmMovement = vector.new(qArm1) - vector.new(qArm0);
  local max_movement_ratio = 1
  for i=1,7 do
    local movement_ratio = math.abs(util.mod_angle(qArmMovement[i]))
      /(velLimit[i]*dt)
    max_movement_ratio = math.max(max_movement_ratio,movement_ratio)
  end
  return max_movement_ratio
end

local function plan_arm(self,qArm0, trArm1, isLeft)
  if not qArm0 then return nil end
  local t0 = unix.time()
  local done, failed = false, false
  local qArm = qArm0;
  local trArm, trArmNext, qArmNext    
  if isLeft>0 then trArm = Body.get_forward_larm(qArm0)
  else trArm = Body.get_forward_rarm(qArm0)   end
--[[  
  print("Current tr:",self.print_transform(trArm))
  print("Target tr:",self.print_transform(trArm1))
  print("Current jangle:",self.print_jangle(qArm0))
--]]

  local dpArmMax = vector.new(Config.arm.linear_slow_limit)
  local dt_step = 0.01
  local dt_step = 0.1
  local dt_step = 0.5
  local yawMag = dt_step * 10*math.pi/180
  
  --Insert initial arm transform to the queue
  local qArmQueue={{qArm,dt_step}}
  local qArmCount = 2

  while not done and not failed do        
    trArmNext, done = self.get_next_transform(trArm,trArm1,dpArmMax,dt_step)
    qArmNext = self:search_shoulder_angle(qArm,trArmNext,isLeft, yawMag)
    if not qArmNext then failed = true 
    else      
      local max_movement_ratio = check_arm_joint_velocity(qArm, qArmNext, dt_step)
      local dt_step_current = dt_step * max_movement_ratio
      qArmQueue[qArmCount] = {qArmNext,dt_step_current}
      qArmCount = qArmCount + 1
      qArm = qArmNext
      trArm = trArmNext      
    end
  end

  local t1 = unix.time()  
  print(string.format("%d steps planned, %.2f ms elapsed:", qArmCount,(t1-t0)*1000 ))
  if failed then 
    print("Plan failure at",self.print_transform(trArmNext))
    print("Arm angle:",unpack(vector.new(qArm)*180/math.pi))
    return qArmQueue,qArm
  else
    return qArmQueue, qArm
  end
end

local function set_hand_mass(self,mLeftHand, mRightHand)
  self.mLeftHand = mLeftHand
  self.mRightHand = mRightHand
end

local function reset_torso_comp(self,qLArm,qRArm)
  local qWaist = {0,0}--TODO: will we use waist position as well?
  local com = Kinematics.com_upperbody(qWaist,qLArm,qRArm,
        Config.walk.bodyTilt, 0,0)
  self.torsoCompBias = {-com[1]/com[4],-com[2]/com[4]}
end


local function get_torso_compensation(self,qLArm,qRArm,massL,massR)
  local qWaist = {0,0}--TODO: will we use waist position as well?

  if mcm.get_status_iskneeling()==1 or 
    Config.stance.enable_torso_compensation==0 then
    return {0,0}
  else  
    local com = Kinematics.com_upperbody(qWaist,qLArm,qRArm,
       Config.walk.bodyTilt, massL, massR)
    return {-com[1]/com[4]-self.torsoCompBias[1],-com[2]/com[4]-self.torsoCompBias[2]}
  end
end



local function plan_double_arm(self,qLArm0,qRArm0,qLArmComp0, qRArmComp0, trLArm1, trRArm1, uTorsoComp0)
-- Now we compensate for the COM movement
-- if we reach out, we have to move torso back to compensate
-- and then we need to reach out further to compensate for that movement
-- Here qLArm and qRArm values are the values BEFORE compensation
-- qLArmQueue and qRArmQueue uses the compensated arm positions


  if not qLArm0 or not qRArm0 then return nil end

  local massL, massR = self.mLeftHand, self.mRightHand
print("Object mass:",massL, massR)
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

--      qLArmQueue[qArmCount] = {qLArmNext,dt_step_current}
--      qRArmQueue[qArmCount] = {qRArmNext,dt_step_current}

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
    return qLArmQueue,qRArmQueue, uTorsoCompQueue, qLArmNext, qRArmNext, qLArmComp, qRArmComp, uTorsoCompNext
  else

    return qLArmQueue,qRArmQueue, uTorsoCompQueue, qLArmNext, qRArmNext, qLArmComp, qRArmComp, uTorsoCompNext
  end
end



local function init_trajectory(self,armQueue, t0)
  self.armQueue = armQueue
  self.armQueuePlayStartTime = t0
  self.armQueuePlayEndTime = t0 + armQueue[1][2]
  self.qArmStart = armQueue[1][1]
  self.qArmEnd = armQueue[1][1]
  self.armQueuePlaybackCount = 1
end

local function init_trajectory_double(self,LArmQueue,RArmQueue,torsoCompQueue,t0)
  self.leftArmQueue = LArmQueue
  self.rightArmQueue = RArmQueue
  self.torsoCompQueue = torsoCompQueue

  self.t_last = t0
  self.armQueuePlayStartTime = t0
  self.armQueuePlayEndTime = t0 + LArmQueue[1][2]

  self.qLArmStart = LArmQueue[1][1]
  self.qLArmEnd = LArmQueue[1][1]
  self.qRArmStart = RArmQueue[1][1]
  self.qRArmEnd = RArmQueue[1][1]
  self.uTorsoCompStart=vector.new(torsoCompQueue[1])
  self.uTorsoCompEnd=vector.new(torsoCompQueue[1])

  self.armQueuePlaybackCount = 1

--
  print(string.format("====================\nK=%d, uTOrsoComp: %.3f %.3f to %.3f %.3f",
    self.armQueuePlaybackCount,
    self.uTorsoCompStart[1],self.uTorsoCompStart[2],
    self.uTorsoCompEnd[1],self.uTorsoCompEnd[2] ))
--[[

  local trLArm0 = Body.get_forward_larm(self.qLArmStart)
  local trLArm1 = Body.get_forward_larm(self.qLArmEnd)

  print(string.format("trLArm: %.3f %.3f %.3f to %.3f %.3f %.3f\n",
    trLArm0[1],trLArm0[2],trLArm0[3],
    trLArm1[1],trLArm1[2],trLArm1[3] ))
--]]
end




local function playback_trajectory(self,t)
  if #self.armQueue < self.armQueuePlaybackCount then
    return nil
  else
   --Skip keyframes if needed
    while t>self.armQueuePlayEndTime do        
      self.armQueuePlaybackCount = self.armQueuePlaybackCount +1        
      if #self.armQueue < self.armQueuePlaybackCount then
        --Passed the end of the queue. return the last joint angle
        return self.armQueue[#self.armQueue][1]
      end
      --Update the frame start and end time
      self.armQueuePlayStartTime = self.armQueuePlayEndTime        
      self.armQueuePlayEndTime = self.armQueuePlayStartTime + 
          self.armQueue[self.armQueuePlaybackCount][2]
      --Update initial and final joint angle
      self.qArmStart = vector.new(self.armQueue[self.armQueuePlaybackCount-1][1])
      self.qArmEnd = vector.new(self.armQueue[self.armQueuePlaybackCount][1])
    end
    local ph = (t-self.armQueuePlayStartTime)/ 
              (self.armQueuePlayEndTime-self.armQueuePlayStartTime)
    local qArm={}
    for i=1,7 do
      qArm[i] = self.qArmStart[i] + ph * (util.mod_angle(self.qArmEnd[i]-self.qArmStart[i]))
    end
    return qArm
  end
end

local function playback_trajectory_double(self,t)
  local dt =  t - self.t_last
  self.t_last = t
  if #self.leftArmQueue < self.armQueuePlaybackCount then
    return nil
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
      print(string.format("====================\nK=%d, uTOrsoComp: %.3f %.3f to %.3f %.3f",
        self.armQueuePlaybackCount,
        self.uTorsoCompStart[1],self.uTorsoCompStart[2],
        self.uTorsoCompEnd[1],self.uTorsoCompEnd[2] ))
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
    return qLArm, qRArm, uTorsoComp
  end
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
  s.plan_arm = plan_arm
  s.plan_double_arm = plan_double_arm
  s.print_transform = print_transform
  s.print_jangle = print_jangle
  s.calculate_margin = calculate_margin
  s.search_shoulder_angle = search_shoulder_angle
  s.get_next_transform = get_next_transform
  s.get_torso_compensation = get_torso_compensation
  s.set_hand_mass = set_hand_mass
  s.check_arm_joint_velocity = check_arm_joint_velocity
  s.reset_torso_comp = reset_torso_comp

  s.init_trajectory = init_trajectory
  s.playback_trajectory = playback_trajectory

  s.init_trajectory_double = init_trajectory_double
  s.playback_trajectory_double = playback_trajectory_double
  return s
end

return libArmPlan