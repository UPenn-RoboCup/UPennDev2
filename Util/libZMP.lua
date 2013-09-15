-- Torch/Lua Zero Moment Point Library
-- (c) 2013 Stephen McGill, Seung-Joon Yi
local vector = require'vector'
local torch = require'torch'
torch.Tensor = torch.DoubleTensor

local function compute_preview( solver )
  -- Preview parameters
  local ts = 0.010 -- 10ms timestep
  local preview_interval = 1.50 -- 1500ms
  local nPreview = math.ceil(preview_interval / ts)
  local r_q = 1e-6; -- balancing parameter for optimization
  --
  local tZMP  = solver.tZMP
  -- Cache some common operations
  local ts_integrated  = ts*ts/2
  local ts_twice_integrated = ts^3
  local tZMP_sq = tZMP*tZMP

  -- Make the preview parameter matrices
  local px  = torch.Tensor(nPreview,3)
  local pu0 = torch.Tensor(nPreview)
  local pu  = torch.Tensor(nPreview,nPreview):zero()
  -- Form the parameter matrices
  -- x[k] = [1 k*tStep k^2*tStep^2/2-tZMP^2 ]
  for k=1,nPreview do
    local px_row = px:select(1,k)
    px_row[1] = 1
    px_row[2] = k*ts
    px_row[3] = k*k*ts_integrated - tZMP_sq
    -- initial control
    pu0[k] = (1+3*(k-1)+3*(k-1)^2)*ts_twice_integrated - ts*tZMP_sq
    -- Triangular
    local pu_row = pu:select(1,k)
    for j=k,1,-1 do
      pu_row[j] = pu0[k-j+1]
    end
  end
  -- Cache the transpose
  local pu_trans = pu:t()
  local balancer = torch.eye(nPreview):mul(r_q)

  -- Exports
  -- TODO: Make this more efficient
  local K1 = -torch.inverse( pu_trans*pu + balancer ) * pu_trans 
  local K1_px = K1 * px
  -- Make the discrete control matrices
  local A = torch.Tensor{
    {1,ts,ts_integrated},
    {0,1, ts},
    {0,0, 1}
  }
  -- TODO: why extra ts????
  local B = torch.Tensor{ts_twice_integrated, ts_integrated, ts, ts}
  return K1, K1_px, A, B
end

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
-- This computes internal paramters for a new step
local function compute( self, uSupport, uStart, uFinish )
  local tStep = self.tStep
  local start_phase   = tStep*self.start_phase
  local finish_phase  = tStep*(1-self.finish_phase)
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
  return com
end

local function get_foot(self,ph)
  -- Computes relative x, z motion of foot during single support phase
  -- phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
  -- phSingle is 100% @ finish_phase, and 0% at start_phase
  -- It just ignores the double support phase so we know how long we've been in single support
  local phSingle = math.min( math.max(ph-self.start_phase, 0)/(self.finish_phase-self.start_phase), 1)
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle)
  local xf = .5*(1-math.cos(math.pi*phSingleSkew))
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew))
  -- xf and zf and percentages, it seems
  return xf, zf, phSingle
end

-- Begin the library code
local libZMP = {}
-- Make a new solver with certain parameters
-- You can update these paramters on the fly, of course
libZMP.new_solver = function( params )
	local s = {}
  s.tStep = params.tStep
  s.tZMP  = params.tZMP
  -- Starting and stopping phase of the trapezoidal
  -- transition from double support to single and back
  s.start_phase  = params.start_phase
  s.finish_phase = params.finish_phase
  -- Add the functions
  s.compute = compute
  s.get_com = get_com
  s.get_foot = get_foot
	return s
end

return libZMP