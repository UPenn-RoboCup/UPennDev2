-- Torch/Lua Zero Moment Point Library
-- (c) 2013 Stephen McGill, Seung-Joon Yi
local vector = require'vector'
local util   = require'util'
local torch  = require'torch'
torch.Tensor = torch.DoubleTensor

-- step_definition format
--{ Supportfoot relstep zmpmod duration steptype }--
-- Provide initial (relative) feet positions
local function generate_step_queue(solver,step_definition,uLeftI,uRightI)
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
    -- Perform a table copy
    if supportLeg==0 then
      -- left support
      step_queue_element.uRight = util.pose_global(step_def[2],uRightI)
      step_queue_element.uLeft  = vector.pose{unpack(uLeftI)}
      --step_queue_element.zaRight = zaRight + vector.new(step_def[3]);
    elseif supportLeg==1 then
      -- right support
      step_queue_element.uLeft  = util.pose_global(step_def[2],uLeftI)
      step_queue_element.uRight = vector.pose{unpack(uRightI)}
      -- step_queue_element.zaLeft = zaLeft + vector.new(step_def[3]);
    elseif supportLeg==2 then --DS
      -- Double Support: Body height change
      step_queue_element.uLeft  = vector.pose{unpack(uLeftI)}
      step_queue_element.uRight = vector.pose{unpack(uRightI)}
      -- step_queue_element.zaRight = zaRight - vector.new(step_def[3]);
      -- step_queue_element.zaLeft  = zaLeft  - vector.new(step_def[3]);
    end
    -- Insert the element
    print('adding',step_queue_element.uRight)
    table.insert(solver.step_queue,step_queue_element)
  end
  -- Reset the queue_index
  solver.step_queue_index = 1
end

local function update_preview(solver, t, supportX, supportY)
  --update_zmp_array(tStateUpdate+time_offset)
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
    local uLeftSupport  = util.pose_global({supportX, supportY, 0}, uLeftF)
    local uRightSupport = util.pose_global({supportX, -supportY, 0}, uRightF)
    uSupportF = util.se2_interpolate(0.5,uLeftSupport,uRightSupport)
  end

  -- Update the preview zmp elements
  local preview = solver.preview
  local nPreview = preview.nPreview
  -- Grab the current buffer
  local cur_buf  = preview.zmp_buffer
  local next_buf = 3-cur_buf
  --print('n',#preview.zmp_x,cur_buf,next_buf,cur_zmp_x,)
  local cur_zmp_x  = preview.zmp_x[cur_buf]
  local next_zmp_x = preview.zmp_x[next_buf]
  local cur_zmp_y  = preview.zmp_y[cur_buf]
  local next_zmp_y = preview.zmp_y[next_buf]
  -- Copy over
  next_zmp_x:narrow(1,1,nPreview-1):copy(cur_zmp_x:narrow(1,2,nPreview-1))
  next_zmp_y:narrow(1,1,nPreview-1):copy(cur_zmp_y:narrow(1,2,nPreview-1))
  -- Add the final element
  next_zmp_x[nPreview] = uSupportF[1]
  next_zmp_y[nPreview] = uSupportF[2]

  -- Efficient queue for other elements
  -- http://www.lua.org/pil/11.4.html
  local first = preview.first
  local last  = preview.last
  -- Wipe old elements
  preview.phs[first]    = nil
  preview.uLeft[first]  = nil
  preview.uRight[first] = nil
  preview.supportLegs[first] = nil
  --
  first = first + 1
  last  = last  + 1
  preview.first = first
  preview.last  = last
  --
  preview.phs[last] = phF
  preview.uLeft[last]  = uLeftF
  preview.uRight[last] = uRightF
  preview.supportLegs[last] = supportLegF
  
end

local function solve_preview(solver)
  -- 3x2 state matrix
  -- x: torso, ?, ?
  -- y: torso, ?, ?
  local preview = solver.preview
  local current_state = preview.state
  local feedback_gain1, feedback_gain2 = 0, 0
  --current_state[1][1] = current_state[1][1]+x_err[1]*feedback_gain1
  --current_state[1][2] = current_state[1][2]+x_err[2]*feedback_gain1

  -- Update control signal: how to get to our desired states
  -- u = param_k1_px * x - param_k1* zmparray
  -- Select the first row, since we only care about 
  -- the IMMEDIATE next step
  local K1_px_row = preview.K1_px:select(1,1)
  local K1_row = preview.K1:select(1,1)
  -- TODO: Don't like these transposes
  local u      = torch.mv(current_state:t(),K1_px_row)
  local zmp_x  = preview.zmp_x[preview.zmp_buffer]
  local zmp_y  = preview.zmp_y[preview.zmp_buffer]
  -- Grab the immediate control input
  --local u_x    = torch.dot(K1_row,zmp_x)
  --local u_y    = torch.dot(K1_row,zmp_y)

  -- Update the state
  --[[
  x[1][1]=x[1][1]+x_err[1]*feedback_gain2;
  x[1][2]=x[1][2]+x_err[2]*feedback_gain2;
  --]]
  -- x = param_a * x + param_b * u;
  preview.state = torch.mm(preview.A,current_state):add(
    torch.ger( preview.B, u )
  )
  -- Yield the desired torso pose
  return vector.pose{
    preview.state[1][1],
    preview.state[1][2],
    0 -- incorrect...
  }
end

local function init_preview(solver,uTorso,uLeft,uRight)
  assert(solver.preview,'Please precompute the preview engine!')
  local nPreview = solver.preview.nPreview
  -- Generate the zmp x and y buffers
  solver.preview.zmp_buffer = 1
  solver.preview.zmp_x = {
    torch.Tensor(nPreview):fill(uTorso[1]),
    torch.Tensor(nPreview):fill(uTorso[1])
  }
  solver.preview.zmp_y = {
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
  -- Save data to our solver
  solver.preview.first = 1
  solver.preview.last  = nPreview
  solver.preview.phs   = phs
  solver.preview.supportLegs = supportLegs
  solver.preview.uLeft  = uLeftTargets
  solver.preview.uRight = uRightTargets
  -- Our first state:
  solver.preview.state = torch.Tensor{{uTorso[1],uTorso[2]},{0,0},{0,0}}

end

-- preview_interval: How many seconds into the future should we preview?
-- ts: At what granularity should we solver?
local function compute_preview( solver, preview_interval )
  -- Preview parameter defaults
  preview_interval = preview_interval or 1.50 -- 1500ms
  ts = ts or 0.010 -- 10ms timestep
  local nPreview = math.ceil(preview_interval / ts)
  local r_q = 1e-6 -- balancing parameter for optimization
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
  solver.preview = {}
  -- TODO: Make this more efficient
  local tmp = torch.mm(pu_trans,pu):add(balancer)
  local K1 = -torch.inverse( tmp ) * pu_trans 
  solver.preview.K1    = K1
  solver.preview.K1_px = K1 * px
  -- Make the discrete control matrices
  solver.preview.A = torch.Tensor{
    {1,ts,ts_integrated},
    {0,1, ts},
    {0,0, 1}
  }
  -- TODO: why extra ts????
  --solver.preview.B = torch.Tensor{ts_twice_integrated, ts_integrated, ts, ts}
  -- Removing the extra ts for now...
  solver.preview.B = torch.Tensor{ts_twice_integrated, ts_integrated, ts}
  -- Save preview data
  solver.preview.nPreview = nPreview
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
  params = params or {}
	local s = {}
  s.tStep = params.tStep or 1
  s.tZMP  = params.tZMP or .25
  -- Starting and stopping phase of the trapezoidal
  -- transition from double support to single and back
  -- Acquire trapezoidal parameters
  s.start_phase  = params.start_phase
  s.finish_phase = params.finish_phase
  -- Trapezoidal ZMP
  s.compute  = compute
  s.get_com  = get_com
  s.get_foot = get_foot
  -- ZMP Preview
  s.compute_preview = compute_preview
  s.init_preview    = init_preview
  s.update_preview  = update_preview
  s.solve_preview  = solve_preview
  -- General
  s.generate_step_queue = generate_step_queue
	return s
end

return libZMP