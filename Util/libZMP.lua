-- Torch/Lua Zero Moment Point Library
-- (c) 2013 Stephen McGill, Seung-Joon Yi
local vector = require'vector'
local torch  = require'torch'
torch.Tensor = torch.DoubleTensor

-- step_definition format
--{ Supportfoot relstep zmpmod duration steptype }--
local function generate_step_queue(solver,step_definition)
  -- Make the step_queue table
  solver.step_queue = {}
  for _,step_def in ipairs(step_definition) do
    local supportLeg = step_def[1]
    local step_queue_element = {
      supportLeg = supportLeg,
      zaLeft  = vector.zeros(2),
      zaRight = vector.zeros(2),
      zmp_mod   = step_def[3],
      duration  = step_def[4],
      step_type = step_def[5] or 0
    }
    -- Form the Support Leg positions
    if supportLeg==0 then
      -- left support
      step_queue_element.uRight = util.pose_global(step_def[2],uRightI)
      step_queue_element.uLeft  = vector.pose(uLeftI)
      --step_queue_element.zaRight = zaRight + vector.new(step_def[3]);
    elseif supportLeg==1 then
      -- right support
      step_queue_element.uLeft  = util.pose_global(step_def[2],uLeftI)
      step_queue_element.uRight = vector.pose(uRightI)
      -- step_queue_element.zaLeft = zaLeft + vector.new(step_def[3]);
    elseif supportLeg==2 then --DS
      -- Double Support: Body height change
      -- step_queue_element.zaRight = zaRight - vector.new(step_def[3]);
      -- step_queue_element.zaLeft  = zaLeft  - vector.new(step_def[3]);
    end
    -- Insert the element
    table.insert(solver.step_queue,step_queue_element)
  end
  -- Reset the queue_index
  solver.step_queue_index = 1
end

local function init_preview(solver,nPreview)
  -- Generate the zmp x and y buffers
  solver.zmp_buffer = 1
  solver.zmp_x = {
    torch.Tensor(nPreview):fill(uTorso[1]),
    torch.Tensor(nPreview):fill(uTorso[1])
  }
  solver.zmp_y = {
    torch.Tensor(nPreview):fill(uTorso[2]),
    torch.Tensor(nPreview):fill(uTorso[2]),
  }
  --
  local phs = {}
  local supportLegs   = {}
  local uLeftTargets  = {}
  local uRightTargets = {}
  for i=1,nPreview do
     -- double support
    table.insert(phs,0)
    table.insert(supportLegs,2)
    table.insert(uLeftTargets,
      vector.pose{uLeft[1],uLeft[2],uLeft[3]})
    table.insert(uRightTargets,
      vector.pose{uRight[1],uRight[2],uRight[3]})
  end
  solver.preview_elements.first = 1
  solver.preview_elements.last  = nPreview
  solver.preview_elements.phs   = phs
  solver.preview_elements.supportLegs = supportLegs
  solver.preview_elements.uLeft  = uLeftTargets
  solver.preview_elements.uRight = uRightTargets
end

local function update_preview(solver, t)
  update_zmp_array(tStateUpdate+time_offset)
  local idx = solver.step_queue_index
  local step_queue_element
  -- Initiate the step_queue time if this is the first step
  if not solver.step_queue_time then
    solver.step_queue_time = t + solver.step_queue[1].duration
  end
  local phF = 0
  -- Check if we are a new step
  local t_expire = solver.step_queue_time - t
  if t_expire<=0 then
    -- Update the index in our step queue
    idx = idx + 1
    solver.step_queue_index = idx
    step_queue_element = solver.step_queue[idx]
    -- Add the duration of this next step
    solver.step_queue_time = t + step_queue_element.duration
  else
    -- Just grab the element
    step_queue_element = solver.step_queue[idx]
    if idx<#solver.step_queue then
      -- not the last step in the queue
      phF = 1-t_expire/step_queue_element.duration
    end
  end
  -- Grab the next step - this is treated as the "final"
  -- position for the preview state engine
  -- TODO: We can cache this element, since idx may not
  -- change that often, and we can save some instructions
  local uLeftF  = step_queue_element.uLeft
  local uRightF = step_queue_element.uRight
  local supportLegF = step_queue_element.supportLeg
  -- Grab the new "final" support element
  local uSupportF
  if supportLegF==0 then
    -- left support
    uSupportF = util.pose_global({supportX, supportY, 0}, uLeftF)
  elseif supportLegF==1 then
    -- right support
    uSupportF = util.pose_global({supportX, -supportY, 0}, uRightF)
  else
    -- double support
    local uLeftSupport = util.pose_global({supportX, supportY, 0}, uLeftF);
    local uRightSupport = util.pose_global({supportX, -supportY, 0}, uRightF);
    uSupportF = util.se2_interpolate(0.5,uLeftSupport,uRightSupport);
  end

  -- Update the preview zmp elements
  local x_buffers = solver.zmp_x
  local y_buffers = solver.zmp_y
  -- Grab the current buffer
  local cur_buf  = solver.zmp_buffer
  local next_buf = 3-cur_buf
  --
  local cur_zmp_x  = x_buffers[cur_buf]
  local next_xmp_x = x_buffers[next_buf]
  local cur_zmp_y  = y_buffers[cur_buf]
  local next_xmp_y = y_buffers[next_buf]
  -- Copy over
  next_zmp_x:narrow(1,1,nPreview-1):copy(cur_zmp_x:narrow(1,2,nPreview))
  next_zmp_y:narrow(1,1,nPreview-1):copy(cur_zmp_y:narrow(1,2,nPreview))
  -- Add the final element
  next_zmp_x[nPreview] = uSupportF[1]
  next_zmp_y[nPreview] = uSupportF[2]

  -- Efficient queue for other elements
  -- http://www.lua.org/pil/11.4.html
  local first = solve.preview_elements.first
  local last  = solve.preview_elements.last
  -- Wipe old elements
  solver.preview_elements.phs[first]    = nil
  solver.preview_elements.uLeft[first]  = nil
  solver.preview_elements.uRight[first] = nil
  solver.preview_elements.supportLeg[first] = nil
  --
  first = first + 1
  last  = last  + 1
  solver.preview_elements.first = first
  solver.preview_elements.last  = last
  --
  solver.preview_elements.phs[last] = phF
  solver.preview_elements.uLeft[last]  = uLeftF
  solver.preview_elements.uRight[last] = uRightF
  solver.preview_elements.supportLeg[last] = supportLegF
  
end

local function solve_preview(solver)
  -- 3x2 state matrix
  -- x: torso, ?, ?
  -- y: torso, ?, ?
  local current_state = solver.preview_state
  local feedback_gain1, feedback_gain2 = 0, 0
  --feedback_gain1 = 1;
  --  Update state variable
  --  u = param_k1_px * x - param_k1* zmparray; --Control signal
  -- Select the first row, since we only care about 
  -- the IMMEDIATE next step
  local K1_px_row = solver.K1_px:select(1,1)
  -- x direction
  --current_state[1][1] = current_state[1][1]+x_err[1]*feedback_gain1
  --[[
  local ux = 
    K1_px_row[1] * current_state[1][1] +
    K1_px_row[2] * current_state[2][1] +
    K1_px_row[3] * current_state[3][1]

  -- y direction
  --current_state[1][2] = current_state[1][2]+x_err[2]*feedback_gain1
  local uy =
    K1_px_row[1] * current_state[1][2] +
    K1_px_row[2] * current_state[2][2] +
    K1_px_row[3] * current_state[3][2]
  --]]
  -- The input signal
  local u = torch.mm(K1_px_row,current_state)
--[[
  x[1][1]=x[1][1]+x_err[1]*feedback_gain2;
  x[1][2]=x[1][2]+x_err[2]*feedback_gain2;
--]]

  -- Dot product here
  -- Select the first row, since we only care about 
  -- the IMMEDIATE next step
  local K1_row = solver.K1:select(1,1)
  local zmp_x  = solver.zmp_x[solver.zmp_buffer]
  local zmp_y  = solver.zmp_y[solver.zmp_buffer]
  -- Grab the immediate control input
  local u_x    = torch.dot(K1_row,zmp_x)
  local u_y    = torch.dot(K1_row,zmp_y)

  -- Update the state
  -- x = param_a * x + param_b * u;
  solver.preview_state = torch.mv(solver.A,current_state):add(
    torch.mv( solver.B, u )
  )
  -- Yield the desired torso pose
  return vector.pose{
    solver.preview_state[1][1],
    solver.preview_state[1][2],
    0 -- incorrect...
  }
end

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
  solver.K1 = -torch.inverse( pu_trans*pu + balancer ) * pu_trans 
  solver.K1_px = K1 * px
  -- Make the discrete control matrices
  solver.A = torch.Tensor{
    {1,ts,ts_integrated},
    {0,1, ts},
    {0,0, 1}
  }
  -- TODO: why extra ts????
  solver.B = torch.Tensor{ts_twice_integrated, ts_integrated, ts, ts}
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