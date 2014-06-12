-- Torch/Lua Walk step generator
-- (c) 2013 Stephen McGill, Seung-Joon Yi
local vector = require'vector'
local util = require'util'
require'mcm'

-- Velocity limits used in update_velocity function
local velLimitX = Config.walk.velLimitX or {-.06, .08}
local velLimitY = Config.walk.velLimitY or {-.06, .06}
local velLimitA = Config.walk.velLimitA or {-.4, .4}
local velDelta  = Config.walk.velDelta or {.03,.015,.15}
local vaFactor  = Config.walk.vaFactor or 0.6

-- Foothold Generation parameters ---------------------------------------
local stanceLimitX = Config.walk.stanceLimitX or {-0.10 , 0.10}
local stanceLimitY = Config.walk.stanceLimitY or {0.09 , 0.20}
local stanceLimitA = Config.walk.stanceLimitA or {-0*math.pi/180, 40*math.pi/180}

-- Toe/heel overlap checking values
local footSizeX = Config.walk.footSizeX or {-0.05,0.05}
local stanceLimitMarginY = Config.walk.stanceLimitMarginY or 0.015
local stanceLimitY2 = 2* Config.walk.footY-stanceLimitMarginY
-------------------------------------------------------------------------

local footY    = Config.walk.footY
local supportX = Config.walk.supportX
local supportY = Config.walk.supportY
local torsoX    = Config.walk.torsoX

------------------------------------------------------------------------

local velocityBias = vector.new(Config.walk.velocityBias or {0,0,0})

local function update_velocity(self,vel)  
  -- Grab from the shared memory the desired walking speed
  local vx,vy,va = unpack(vel)
--  local vx,vy,va = unpack(hcm.get_motion_velocity())

  --Filter the commanded speed
  vx = math.min(math.max(vx,velLimitX[1]),velLimitX[2])
  vy = math.min(math.max(vy,velLimitY[1]),velLimitY[2])
  va = math.min(math.max(va,velLimitA[1]),velLimitA[2])
  -- Find the magnitude of the velocity
  local stepMag = math.sqrt(vx^2+vy^2)
  --Slow down when turning
  local vFactor   = 1-math.abs(va)/vaFactor
  local magFactor = math.min(velLimitX[2]*vFactor,stepMag)/(stepMag+0.000001)
  -- Limit the forwards and backwards velocity
  vx = math.min(math.max(vx*magFactor,velLimitX[1]),velLimitX[2])
  vy = math.min(math.max(vy*magFactor,velLimitY[1]),velLimitY[2])
  va = math.min(math.max(va,velLimitA[1]),velLimitA[2])
  -- Check the change in the velocity
  local velDiff = vector.new({vx,vy,va}) - self.velCurrent
  -- Limit the maximum velocity change PER STEP
  velDiff[1] = util.procFunc(velDiff[1],0,velDelta[1])
  velDiff[2] = util.procFunc(velDiff[2],0,velDelta[2])
  velDiff[3] = util.procFunc(velDiff[3],0,velDelta[3])

  -- Update the current velocity command
  self.velCurrent = self.velCurrent+velDiff    
  return self.velCurrent
end

local function init_stance(self)
  local uTorso = mcm.get_status_uTorso()  
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()
  return uLeft,uRight,uTorso,uLeft,uRight,uTorso
end

local function save_stance(self,uLeft,uRight,uTorso)
  supportFoot = supportFoot or 0;
  mcm.set_status_uLeft(uLeft)
  mcm.set_status_uRight(uRight)
  mcm.set_status_uTorso(uTorso)
end

local function get_next_step_velocity(self,uLeft_now, uRight_now, uTorso_now, supportLeg, initialStep,lastStep)  
  local uLeft_next, uRight_next, uTorso_next = uLeft_now, uRight_now, uTorso_now
  local uLSupport,uRSupport = self.get_supports(uLeft_now,uRight_now)
  local uSupport

--OMG why do we still have these?
initialStep = false
velocityBias = vector.new({0,0,0})

  local velWalk = self.velCurrent + velocityBias




  if initialStep then
    if supportLeg == 0 then    -- Left support      
      uSupport = util.se2_interpolate(0.3,uLSupport,uTorso_next)      
    else    -- Right support      
      uSupport = util.se2_interpolate(0.3,uRSupport,uTorso_next)      
    end  
  else
    if supportLeg == 0 then    -- Left support
      uSupport = uLSupport
      uRight_next = self.step_destination_right(velWalk, uLeft_now, uRight_now,lastStep)    
      local uLSupport_next,uRSupport_next = self.get_supports(uLeft_next,uRight_next)
      uTorso_next = util.se2_interpolate(0.5, uLSupport_next, uRSupport_next)
    else    -- Right support
      uSupport = uRSupport
      uLeft_next = self.step_destination_left(velWalk, uLeft_now, uRight_now,lastStep)    
      local uLSupport_next,uRSupport_next = self.get_supports(uLeft_next,uRight_next)
      uTorso_next = util.se2_interpolate(0.5, uLSupport_next, uRSupport_next)
    end  
  end
  return uLeft_now, uRight_now,uTorso_now, uLeft_next, uRight_next, uTorso_next, uSupport 
end




local function get_next_step_increment(self,uLeft_now, uRight_now, uTorso_now, supportLeg, increment)
  local uLeft_next, uRight_next, uTorso_next = uLeft_now, uRight_now, uTorso_now
  local uLSupport,uRSupport = self.get_supports(uLeft_now,uRight_now)
  local uSupport
  if supportLeg == 0 then    -- Left support
      uSupport = uLSupport
      uRight_next = util.pose_global(increment,uRight_now)
      local uLSupport_next,uRSupport_next = self.get_supports(uLeft_next,uRight_next)
      uTorso_next = util.se2_interpolate(0.5, uLSupport_next, uRSupport_next)
  elseif supportLeg==1 then    -- Right support
      uSupport = uRSupport
      uLeft_next = util.pose_global(increment,uLeft_now)
      local uLSupport_next,uRSupport_next = self.get_supports(uLeft_next,uRight_next)
      uTorso_next = util.se2_interpolate(0.5, uLSupport_next, uRSupport_next)
  end
  return uLeft_now, uRight_now,uTorso_now, uLeft_next, uRight_next, uTorso_next, uSupport 
end

--------------------------------------------------------------
--
-- Step queue: future foothold informations
--
--------------------------------------------------------------

local function step_enque(self,uFoot,supportLeg,tStep, zmpMod, stepParams)
  local step = {}
  step.uFoot = uFoot
  step.supportLeg = supportLeg
  step.tStep = tStep
  step.zmpMod = zmpMod or vector.new({0,0,0})
  step.stepParams = stepParams
  table.insert(self.stepqueue,step)
end

local function step_enque_trapezoid(self,uFoot,supportLeg, t0,t1,t2, zmpMod, stepParams, is_last)

--[[
  local step_ds = {}
  step_ds.is_trapezoid = true
  step_ds.supportLeg = 2 --Double Support
  step_ds.tStep = tDoubleSupport
  step_ds.zmpMod = vector.new({0,0,0})
  step_ds.is_last = is_last --transition signal
  table.insert(self.stepqueue,step_ds)

  --SS step with stationary ZMP
  local step_ss = {}
  step_ss.uFoot = uFoot
  step_ss.supportLeg = supportLeg
  step_ss.tStep = tStep
  step_ss.zmpMod = zmpMod or vector.new({0,0,0})
  step_ss.stepParams = stepParams  
  table.insert(self.stepqueue,step_ss)
--]]
  local step={}
  step.is_trapezoid = true
  step.uFoot = uFoot
  step.supportLeg = supportLeg
  step.tStep = t0+t1+t2
  step.t0 = t0
  step.t1 = t1
  step.t2 = t2
  step.zmpMod = zmpMod or vector.new({0,0,0})
  step.stepParams = stepParams  
  step.is_last = is_last --transition signal
  table.insert(self.stepqueue,step)

end


local function get_next_step_queue(self,uLeft_now, uRight_now, uTorso_now, initialStep, uSupport_now)  
  if #self.stepqueue==0 then return end

  local uLeft_next, uRight_next, uTorso_next = uLeft_now, uRight_now, uTorso_now
  local uLSupport,uRSupport = self.get_supports(uLeft_now,uRight_now)
  local uSupport, uSupport_next
  local current_step =table.remove(self.stepqueue,1) 
  supportLeg = current_step.supportLeg
  --Foot position based step positions
  if supportLeg==0 then --Left support
    uSupport = util.pose_global(current_step.zmpMod,uLSupport)
    uRight_next = util.pose_global(current_step.uFoot,uRight_now)
    local uLSupport_next,uRSupport_next = self.get_supports(uLeft_next,uRight_next)
    uTorso_next = util.se2_interpolate(0.5, uLSupport_next, uRSupport_next)
  elseif supportLeg==1 then    
    uSupport = util.pose_global(current_step.zmpMod,uRSupport)
    uLeft_next =  util.pose_global(current_step.uFoot,uLeft_now)
    local uLSupport_next,uRSupport_next = self.get_supports(uLeft_next,uRight_next)
    uTorso_next = util.se2_interpolate(0.5, uLSupport_next, uRSupport_next)
  else --Double support    
    uSupport = util.se2_interpolate(0.5, uLSupport, uRSupport)
  end
  local trapezoidparams={}
  if current_step.is_trapezoid then
    trapezoidparams[1]=current_step.t0
    trapezoidparams[2]=current_step.t1
    trapezoidparams[3]=current_step.t2
  end

  return uLeft_now, uRight_now,uTorso_now, uLeft_next, uRight_next, uTorso_next,
         uSupport, supportLeg, current_step.tStep, current_step.stepParams, current_step.is_last,
         trapezoidparams
end


local function get_supports(uLeft,uRight)    
  local uLSupport = util.pose_global({supportX, supportY, 0}, uLeft)
  local uRSupport = util.pose_global({supportX, -supportY, 0}, uRight)
  return uLSupport,uRSupport
end

local function get_torso(uLeft,uRight)
  local uLSupport = util.pose_global({supportX, supportY, 0}, uLeft)
  local uRSupport = util.pose_global({supportX, -supportY, 0}, uRight)
  local uTorso = util.se2_interpolate(0.5, uLSupport, uRSupport)
  return uTorso
end

local function step_destination_left(vel, uLeft, uRight, lastStep)
  if lastStep then
    local uLeftNext = util.pose_global({0, 2*footY,0},uRight)
    return uLeftNext
  end

  local u0 = util.se2_interpolate(.5, uLeft, uRight)
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = util.pose_global(vel, u0)
  local u2 = util.pose_global(.5*vel, u1)
  local uLeftPredict = util.pose_global({0,footY,0}, u2)
  local uLeftRight = util.pose_relative(uLeftPredict, uRight)
  -- Do not pidgeon toe, cross feet:

  --Check toe and heel overlap
  local toeOverlap  = -footSizeX[1] * uLeftRight[3]
  local heelOverlap = -footSizeX[2] * uLeftRight[3]
  local limitY = math.max(stanceLimitY[1],
  stanceLimitY2+math.max(toeOverlap,heelOverlap))

  --print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)

  uLeftRight[1] = math.min(math.max(uLeftRight[1], stanceLimitX[1]), stanceLimitX[2])
  uLeftRight[2] = math.min(math.max(uLeftRight[2], limitY),stanceLimitY[2])
  uLeftRight[3] = math.min(math.max(uLeftRight[3], stanceLimitA[1]), stanceLimitA[2])

  return util.pose_global(uLeftRight, uRight)
end

local function step_destination_right(vel, uLeft, uRight,lastStep)
  if lastStep then
    local uRightNext = util.pose_global({0, -2*footY,0},uLeft)
    return uRightNext
  end
  local u0 = util.se2_interpolate(.5, uLeft, uRight)
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = util.pose_global(vel, u0)
  local u2 = util.pose_global(.5*vel, u1)
  local uRightPredict = util.pose_global({0,-footY,0}, u2)
  local uRightLeft = util.pose_relative(uRightPredict, uLeft)
  -- Do not pidgeon toe, cross feet:

  --Check toe and heel overlap
  local toeOverlap  = footSizeX[1] * uRightLeft[3]
  local heelOverlap = footSizeX[2] * uRightLeft[3]
  local limitY = math.max(stanceLimitY[1], stanceLimitY2+math.max(toeOverlap,heelOverlap))

  --print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)

  uRightLeft[1] = math.min(math.max(uRightLeft[1], stanceLimitX[1]), stanceLimitX[2])
  uRightLeft[2] = math.min(math.max(uRightLeft[2], -stanceLimitY[2]), -limitY)
  uRightLeft[3] = math.min(math.max(uRightLeft[3], -stanceLimitA[2]), -stanceLimitA[1])

  return util.pose_global(uRightLeft, uLeft)
end



local libStep={}

libStep.new_planner = function (params)
  params = params or {}
  local s = {}
  --member variables
  s.stepqueue = {}
  s.velCurrent = vector.new({0,0,0})
  
  --Functions
  s.update_velocity = update_velocity
  s.get_next_step_velocity = get_next_step_velocity

  s.step_enque = step_enque
  s.step_enque_trapezoid = step_enque_trapezoid
  s.get_next_step_queue = get_next_step_queue

  s.get_next_step_increment = get_next_step_increment

  --Functions #2
  s.get_supports = get_supports
  s.get_torso = get_torso
  s.step_destination_left = step_destination_left
  s.step_destination_right = step_destination_right

  s.init_stance = init_stance
  s.save_stance = save_stance 
  return s
end

return libStep