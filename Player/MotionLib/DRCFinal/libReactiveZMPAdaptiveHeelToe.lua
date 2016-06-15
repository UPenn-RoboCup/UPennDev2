-- ZMP library for REACTIVE walking
-- Torch/Lua Zero Moment Point Library
-- (c) 2013 Stephen McGill, Seung-Joon Yi
local vector = require'vector'
local torch  = require'torch'
torch.Tensor = torch.DoubleTensor

-- Perform some math
local function solve( solver, z_support, z_start, z_finish, x1, x2 )
  local tStep = solver.tStep
  local tZMP = solver.tZMP
  local T1 = tStep*solver.start_phase
  local T2 = tStep*solver.finish_phase
  --[[
  Solves ZMP equation:
  x(t) = z(t) + aP*exp(t/tZMP) + aN*exp(-t/tZMP) - tZMP*mi*sinh((t-Ti)/tZMP)
  where the ZMP point is piecewise linear:
  z(0) = z_start, z(T1 < t < T2) = z_support, z(tStep) = z_finish
  --]]
  local m1 = (z_support-z_start)/T1
  local m2 = -(z_support-z_finish)/(tStep-T2)
  local c1 = x1-z_start  + tZMP*m1*math.sinh(-T1/tZMP)
  local c2 = x2-z_finish + tZMP*m2*math.sinh((tStep-T2)/tZMP)
  local expTStep = math.exp(tStep/tZMP)
  local aP = (c2 - c1/expTStep)/(expTStep-1/expTStep)
  local aN = (c1*expTStep - c2)/(expTStep-1/expTStep)
  return aP, aN
end


-- Torso goes from uStart to uFinish through uSupport
-- Trapezoidal: / is start to support
--              _ is through support
--              \ is support to finish
-- This computes internal parameters for a new step
local function compute( self, uSupport, uStart, uFinish )
  local tStep = self.tStep
  local start_phase   = tStep*self.start_phase
  local finish_phase  = tStep*(1-self.finish_phase)


  local uSupport1 = util.pose_global({self.zmpHeel,0,0},uSupport)
  local uSupport2 = util.pose_global({self.zmpToe,0,0},uSupport)


  local walkvel = mcm.get_walk_vel()
  if not Config.walk.use_heeltoe_walk or walkvel[1]<Config.walk.heeltoe_vel_min then
    uSupport1 = uSupport
    uSupport2 = uSupport
  end




  local support_from_start  = uSupport - uStart
  local finish_from_support = uFinish - uSupport
  -- Compute ZMP coefficients
  -- Solve for the x direction
  self.m1X = support_from_start[1]  / start_phase
  self.m2X = finish_from_support[1] / finish_phase
  self.aXP, self.aXN = solve( self,
    uSupport[1],
    uStart[1], uFinish[1],
    uStart[1], uFinish[1]
  )
  -- Solve for the y direction
  self.m1Y = support_from_start[2]  / start_phase
  self.m2Y = finish_from_support[2] / finish_phase
  self.aYP, self.aYN = solve( self,
    uSupport[2],
    uStart[2], uFinish[2],
    uStart[2], uFinish[2]
  )



  local support_from_start1  = uSupport1 - uStart
  local finish_from_support1 = uFinish - uSupport1
  -- Compute ZMP coefficients
  -- Solve for the x direction
  self.m1X1 = support_from_start1[1]  / start_phase
  self.m2X1 = finish_from_support1[1] / finish_phase
  self.aXP1, self.aXN1 = solve( self,
    uSupport1[1],
    uStart[1], uFinish[1],
    uStart[1], uFinish[1]
  )
  -- Solve for the y direction
  self.m1Y1 = support_from_start1[2]  / start_phase
  self.m2Y1 = finish_from_support1[2] / finish_phase
  self.aYP1, self.aYN1 = solve( self,
    uSupport1[2],
    uStart[2], uFinish[2],
    uStart[2], uFinish[2]
  )


  local support_from_start2  = uSupport2 - uStart
  local finish_from_support2 = uFinish - uSupport2
  -- Compute ZMP coefficients
  -- Solve for the x direction
  self.m1X2 = support_from_start2[1]  / start_phase
  self.m2X2 = finish_from_support2[1] / finish_phase
  self.aXP2, self.aXN2 = solve( self,
    uSupport2[1],
    uStart[1], uFinish[1],
    uStart[1], uFinish[1]
  )
  -- Solve for the y direction
  self.m1Y2 = support_from_start2[2]  / start_phase
  self.m2Y2 = finish_from_support2[2] / finish_phase
  self.aYP2, self.aYN2 = solve( self,
    uSupport2[2],
    uStart[2], uFinish[2],
    uStart[2], uFinish[2]
  )

  -- Save the torso points
  self.uSupport = uSupport
  self.uSupport1 = uSupport1
  self.uSupport2 = uSupport2
  self.uStart   = uStart
  self.uFinish  = uFinish



--print("uSupport:",uSupport1[1],uSupport[1],uSupport2[1])

--[[
  local support_from_start  = uSupport - uStart
  local finish_from_support = uFinish - uSupport
  -- Compute ZMP coefficients
  -- Solve for the x direction
  self.m1X = support_from_start[1]  / start_phase
  self.m2X = finish_from_support[1] / finish_phase
  self.aXP, self.aXN = solve( self,
    uSupport[1],
    uStart[1], uFinish[1],
    uStart[1], uFinish[1]
  )
  -- Solve for the y direction
  self.m1Y = support_from_start[2]  / start_phase
  self.m2Y = finish_from_support[2] / finish_phase
  self.aYP, self.aYN = solve( self,
    uSupport[2],
    uStart[2], uFinish[2],
    uStart[2], uFinish[2]
  )
  -- Save the torso points
  self.uSupport = uSupport
  self.uStart   = uStart
  self.uFinish  = uFinish
--]]








end















local function get_zmp( self, ph )
  local tStep = self.tStep


  local zmp = self.uSupport+vector.new({0,0,0})
  if ph < self.start_phase then
    local start_time = tStep*( ph - self.start_phase )
    zmp[1] = zmp[1] + self.m1X*start_time
    zmp[2] = zmp[2] + self.m1Y*start_time
  elseif ph > self.finish_phase then
    local finish_time = tStep*(ph-self.finish_phase)
    zmp[1] = zmp[1] + self.m2X*finish_time
    zmp[2] = zmp[2] + self.m2Y*finish_time
  end



  if ph < self.start_phase then
    local start_time = tStep*( ph - self.start_phase )
    zmp = self.uSupport1+vector.new({0,0,0})
    zmp[1] = zmp[1] + self.m1X1*start_time
    zmp[2] = zmp[2] + self.m1Y1*start_time
  elseif ph > self.finish_phase then
    zmp = self.uSupport2+vector.new({0,0,0})
    local finish_time = tStep*(ph-self.finish_phase)
    zmp[1] = zmp[1] + self.m2X2*finish_time
    zmp[2] = zmp[2] + self.m2Y2*finish_time
  else
    local mid_ph = (  ph - self.start_phase ) / (self.finish_phase-self.start_phase)
    zmp = self.uSupport1+vector.new({0,0,0})
    zmp[1] = zmp[1] + mid_ph*(self.uSupport2[1]-self.uSupport1[1])
    zmp[2] = zmp[2] + mid_ph*(self.uSupport2[2]-self.uSupport1[2])
  end

  return zmp
end

-- Finds the necessary COM for stability, given the current uSupport
-- Must call compute upon a new step, or CoM is groundless...
local function get_com( self, ph )
  local tStep = self.tStep
  local tZMP  = self.tZMP
  local expT = math.exp( ph * tStep/tZMP )
  -- Initial Center of mass is for single support
  -- Angle *should* be unused at this point
  local com = self.uSupport + vector.new{
    self.aXP*expT + self.aXN/expT,
    self.aYP*expT + self.aYN/expT,
    0
  }
  -- Check if we are in double<->single transition zone
  if ph < self.start_phase then
    local start_time = tStep*( ph - self.start_phase )
    -- From double to single
    com[1] = com[1] + self.m1X*start_time - tZMP*self.m1X*math.sinh(start_time/tZMP)
    com[2] = com[2] + self.m1Y*start_time - tZMP*self.m1Y*math.sinh(start_time/tZMP)
  elseif ph > self.finish_phase then
    local finish_time = tStep*(ph-self.finish_phase)
    -- From single to double
    com[1] = com[1] + self.m2X*finish_time - tZMP*self.m2X*math.sinh(finish_time/tZMP)
    com[2] = com[2] + self.m2Y*finish_time - tZMP*self.m2Y*math.sinh(finish_time/tZMP)
  end




  if ph < self.start_phase then

    com = self.uSupport1 + vector.new{
      self.aXP1*expT + self.aXN1/expT,
      self.aYP1*expT + self.aYN1/expT,
      0
      }

    local start_time = tStep*( ph - self.start_phase )
    -- From double to single
    com[1] = com[1] + self.m1X1*start_time - tZMP*self.m1X1*math.sinh(start_time/tZMP)
    com[2] = com[2] + self.m1Y1*start_time - tZMP*self.m1Y1*math.sinh(start_time/tZMP)
  elseif ph > self.finish_phase then
    com = self.uSupport2 + vector.new{
      self.aXP2*expT + self.aXN2/expT,
      self.aYP2*expT + self.aYN2/expT,
      0
      }
    local finish_time = tStep*(ph-self.finish_phase)
    -- From single to double
    com[1] = com[1] + self.m2X2*finish_time - tZMP*self.m2X2*math.sinh(finish_time/tZMP)
    com[2] = com[2] + self.m2Y2*finish_time - tZMP*self.m2Y2*math.sinh(finish_time/tZMP)
  else
    local mid_ph = (  ph - self.start_phase ) / (self.finish_phase-self.start_phase)
    com1 = self.uSupport1 + vector.new{
        self.aXP1*expT + self.aXN1/expT,
        self.aYP1*expT + self.aYN1/expT,
        0
        }
    com2 = self.uSupport2 + vector.new{
        self.aXP2*expT + self.aXN2/expT,
        self.aYP2*expT + self.aYN2/expT,
        0
        }
      com[1]=com1[1]+mid_ph*(com2[1]-com1[1])
      com[2]=com1[2]+mid_ph*(com2[2]-com1[2])
  end










  return com
end

local function get_com_vel(self,ph)
  local tStep = self.tStep
  local tZMP  = self.tZMP
  local expT = math.exp( ph * tStep/tZMP )
  -- Initial Center of mass is for single support
  -- Angle *should* be unused at this point
  local com_vel = vector.new{
    (self.aXP*expT  - self.aXN/expT) / tZMP,
    (self.aYP*expT -  self.aYN/expT) / tZMP,
    0
  }
  -- Check if we are in double<->single transition zone
  if ph < self.start_phase then
    local start_time = tStep*( ph - self.start_phase )
    -- From double to single
    com_vel[1] = com_vel[1] + self.m1X*(1 - math.cosh(start_time/tZMP))
    com_vel[2] = com_vel[2] + self.m1Y*(1 - math.cosh(start_time/tZMP))
  elseif ph > self.finish_phase then
    local finish_time = tStep*(ph-self.finish_phase)
    -- From single to double
    com_vel[1] = com_vel[1] + self.m2X*(1 - math.cosh(finish_time/tZMP))
    com_vel[2] = com_vel[2] + self.m2Y*(1 - math.cosh(finish_time/tZMP))
  end
  return com_vel
end

local function set_param(self,tStep,tZmp)
  if tStep then self.tStep = tStep end
  if tZmp then self.tZmp = tZmp end
end

local function get_ph(self,t,t_last_step)
  local tPassed = t-t_last_step
  local is_next_step = false

  if tPassed>(self.tStepLift+self.tStepLand) then
    tPassed = tPassed % (self.tStepLift+self.tStepLand)
    is_next_step = true
  end

  local ph
  if tPassed<self.tStepLift then
    ph = (tPassed/self.tStepLift)/2 --lifting phase
  else
    ph = 0.5 + (tPassed-self.tStepLift)/self.tStepLand/2 --landing phase
  end

  return ph, is_next_step
end


local function set_landing_delay_factor(self,factor)
  self.tStepLand = Config.walk.tStep / 2 *factor
end






-- Begin the library code
local libReaciveZMP = {}
-- Make a new solver with certain parameters
-- You can update these paramters on the fly, of course

libReaciveZMP.new_solver = function( params )
  params = params or {}
	local s = {}
  s.tStep = params.tStep or Config.walk.tStep

  s.tStepLift = params.tStepLift or Config.walk.tStep/2
  s.tStepLand = params.tStepLand or Config.walk.tStep/2


  s.zmpHeel = params.zmpHeel or 0
  s.zmpToe = params.zmpToe or 0



  s.tZMP  = params.tZMP or Config.walk.tZmp

  -- Trapezoidal ZMP parameters
  s.start_phase  = Config.walk.phZmp[1]
  s.finish_phase = Config.walk.phZmp[2]

  s.compute  = compute
  s.get_com  = get_com
  s.get_com_vel  = get_com_vel
  s.get_zmp  = get_zmp
  s.set_param = set_param

  s.get_ph = get_ph
  s.set_landing_delay_factor = set_landing_delay_factor
	return s
end

return libReaciveZMP
