--Arm movement planner
--2013/11 SJ

local vector = require'vector'
local util = require'util'
require'mcm'


--Returns transform trajectories from trArm0 to trArm1
--Satisfying 


local function print_transform(tr)
  local str= string.format("%.2f %.2f %.2f (%.1f %.1f %.1f)",
    tr[1],tr[2],tr[3],
    tr[4]*180/math.pi,tr[5]*180/math.pi,tr[6]*180/math.pi
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


local function plan_arm(self,qArm0, trArm1, isLeft)
  if not qArm0 then return nil end
  local t0 = unix.time()
  local done, failed = false, false
  local trArm
  if isLeft>0 then trArm = Body.get_forward_larm(qArm0)
  else trArm = Body.get_forward_rarm(qArm0)   end
  print("Current:",self.print_transform(trArm))
  print("Target:",self.print_transform(trArm1))

  local trArmQueue = {}

  local dpArmMax = Config.arm.linear_slow_limit
  local dt_step = 0.01
  local yawMag = dt_step * 10*math.pi/180

  local qArm = qArm0;
  local trArmNext, qArmNext

  local qArmQueue={}
  local qArmCount = 1
  
  while not done and not failed do        
    trArmNext, done = self.get_next_transform(trArm,trArm1,dpArmMax,dt_step)
    qArmNext = self:search_shoulder_angle(qArm,trArmNext,isLeft, yawMag)
    if not qArmNext then failed = true 
    else
      --TODO: check joint velocity and adjust timestep
      local qArmMovement = vector.new(qArmNext) - vector.new(qArm);
      qArmQueue[qArmCount] = {qArmNext,dt_step}
      qArmCount = qArmCount + 1
      qArm = qArmNext
      trArm = trArmNext      
    end
  end

  local t1 = unix.time()  
  print("Plan time:",t1-t0)
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

local function get_torso_compensation(qLArm,qRArm,massL,massR)
  local qWaist = {0,0}--TODO: will we use waist position as well?
  local com = Kinematics.com_upperbody(qWaist,qLArm,qRArm,
        Config.walk.bodyTilt, massL, massR)
  return {-com[1]/com[4],-com[2]/com[4]}
end

local function plan_double_arm(self,qLArm0,qRArm0, trLArm1, trRArm1)
  if not qLArm0 or not qRArm0 then return nil end

  local massL, massR = self.mLeftHand, self.mRightHand

  local t0 = unix.time()
  local doneL, doneR, failed = false, false, false
  local trLArm = Body.get_forward_larm(qLArm0)
  local trRArm = Body.get_forward_rarm(qRArm0)
  
  local dpArmMax = Config.arm.linear_slow_limit
  local dt_step = 0.01
  local yawMag = dt_step * 10*math.pi/180

  local qLArm,qRArm = qLArm0 , qRArm0
  local trLArmNext, trRArmNext, qLArmNext, qRArmNext
  local qLArmQueue,qRArmQueue,uTorsoCompensation={},{},{}
  local qArmCount = 1
  
  while (not doneL or not doneR) and not failed do        
    trLArmNext, doneL = self.get_next_transform(trLArm,trLArm1,dpArmMax,dt_step)
    trRArmNext, doneR = self.get_next_transform(trRArm,trRArm1,dpArmMax,dt_step)

    --We use previous joint angle to calculate the COM movement to compensate
    local com_compensation = self.get_torso_compensation(
        qLArm,qRArm,massL,massR)
    local vec_compensation = vector.new({
          -com_compensation[1],-com_compensation[2],0,0,0,0})

--    print("com compensation:",unpack(com_compensation))
--    vec_compensation = vector.zeros(6)
    local trLArmCompensated = vector.new(trLArmNext) + vec_compensation
    local trRArmCompensated = vector.new(trRArmNext) + vec_compensation

    qLArmNext = self:search_shoulder_angle(qLArm,trLArmCompensated,1, yawMag)
    qRArmNext = self:search_shoulder_angle(qRArm,trRArmCompensated,0, yawMag)

    if not qLArmNext or not qRArmNext then 
      failed = true 
      print("failure")
    else      
      qLArmQueue[qArmCount] = {qLArmNext,dt_step}
      qRArmQueue[qArmCount] = {qRArmNext,dt_step}

      uTorsoCompensation[qArmCount] = com_compensation

      qLArm,qRArm,trLArm,trRArm = qLArmNext, qRArmNext, trLArmNext, trRArmNext
      qArmCount = qArmCount + 1
    end
  end

  local t1 = unix.time()  
  print("Plan time:",t1-t0)
  if failed then 
    print("Plan failure at",self.print_transform(trArmNext))
    print("Arm angle:",unpack(vector.new(qArm)*180/math.pi))
    return qLArmQueue,qRArmQueue, qLArm, qRArm, uTorsoCompensation
  else
    return qLArmQueue,qRArmQueue, qLArm, qRArm, uTorsoCompensation
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

local function playback_trajectory(self,t)
  if #self.armQueue < self.armQueuePlaybackCount then
    return nil
  else
     --Skip keyframes if needed
      while t>self.armQueuePlayEndTime do        
        self.armQueuePlaybackCount = self.armQueuePlaybackCount +1        

        --Update the frame start time
        self.armQueuePlayStartTime = self.armQueuePlayEndTime

        if #self.armQueue < self.armQueuePlaybackCount then
          --Passed the end of the queue. return the last joint angle
          return self.armQueue[#self.armQueue][1]
        end

        --Update the frame end time
        self.armQueuePlayEndTime = self.armQueuePlayStartTime + 
            self.armQueue[self.armQueuePlaybackCount][2]

        --Update initial and final joint angle
        self.qArmStart = vector.new(self.armQueue[self.armQueuePlaybackCount-1][1])
        self.qArmEnd = vector.new(self.armQueue[self.armQueuePlaybackCount][1])
      end

         --Now  t should be between playstarttime and playendtime
      local ph = (t-self.armQueuePlayStartTime)/ 
                (self.armQueuePlayEndTime-self.armQueuePlayStartTime)
--  print("ph:",ph," count",self.armQueuePlaybackCount)
      local qArm = (1-ph)*self.qArmStart + ph*self.qArmEnd

      return qArm
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


  --member functions
  s.plan_arm = plan_arm
  s.plan_double_arm = plan_double_arm
  s.print_transform = print_transform
  s.calculate_margin = calculate_margin
  s.search_shoulder_angle = search_shoulder_angle
  s.get_next_transform = get_next_transform
  s.get_torso_compensation = get_torso_compensation
  s.set_hand_mass = set_hand_mass

  s.init_trajectory = init_trajectory
  s.playback_trajectory = playback_trajectory
  return s
end

return libArmPlan