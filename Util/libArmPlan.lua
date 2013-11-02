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

local function plan_arm_linear(self,qArm0, trArm1, isLeft)
  if not qArm0 then return nil end
  local t0 = unix.time()
  local done, failed = false, false
  local trArm
  if isLeft>0 then
    trArm = Body.get_forward_larm(qArm0)
  else
    trArm = Body.get_forward_rarm(qArm0)
  end
  print("Current:",self.print_transform(trArm))
  print("Target:",self.print_transform(trArm1))

  local trArmQueue = {}

  local dpArmMax = Config.arm.linear_slow_limit
  local dt_step = 0.01

  local qArm = qArm0;
  local trArmNext, qArmNext

  local qArmQueue={}
  local qArmCount = 1
  
  while not done and not failed do        
    trArmNext, done = 
      util.approachTolTransform(trArm, trArm1, dpArmMax, dt_step )
--    print("Arm tr:",self.print_transform(trArm))
    local yawMag = dt_step * 10*math.pi/180
    qArmNext = self:search_shoulder_angle(qArm,trArmNext,isLeft, yawMag)
    if not qArmNext then failed = true 
    else
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
    print("Plan failure!")
    return nil
  else
    return qArmQueue, qArm
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
  print("ph:",ph," count",self.armQueuePlaybackCount)
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

  --member functions
  s.plan_arm_linear = plan_arm_linear
  s.print_transform = print_transform
  s.calculate_margin = calculate_margin
  s.search_shoulder_angle = search_shoulder_angle

  s.init_trajectory = init_trajectory
  s.playback_trajectory = playback_trajectory
  return s
end

return libArmPlan