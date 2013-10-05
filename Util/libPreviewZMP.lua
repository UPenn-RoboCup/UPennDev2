-- Torch/Lua Zero Moment Point Library
-- (c) 2013 Stephen McGill, Seung-Joon Yi
local vector = require'vector'
local util   = require'util'
local torch  = require'torch'
torch.Tensor = torch.DoubleTensor

--[[
--We maintain Two queue for preview duration

--ZMP position queue : uSupport
--Step status queue : uLeft_now, uRight_now, uLeft_next,uRight_next,supportLeg


--Initialize a stationary preview queue
local function init_preview_queue(self,uLeft,uRight,t)
  local uLSupport,uRSupport = self.get_supports(uLeft_now,uRight_now)
  local uSupport = util.se2_interpolate(0.5, uLSupport, uRSupport)
  self.preview_queue={}
  self.preview_queue_zmpx={}
  self.preview_queue_zmpy={}
  for i=1,self.preview_steps do
    local preview_item = {}    
    preview_item.uLeft_now = uLeft
    preview_item.uLeft_next = uLeft
    preview_item.uRight_now = uRight
    preview_item.uRight_next = uRight
    preview_item.supportLeg = 2 --Double support

    self.preview_queue[i] = preview_item
    self.preview_queue_zmpx[i] = uSupport[1]
    self.preview_queue_zmpy[i] = uSupport[2]
  end
  self.t_last_step = t
  self.t_step_duration = 0
end

local function update_preview_queue(self,t)
  table.remove(self.preview_queue,1)
  table.remove(self.preview_queue_zmpx,1)
  table.remove(self.preview_queue_zmpy,1)

  local last_preview_item = self.preview_queue[#self.preview_queue]
  local last_preview_zmpx = self.preview_queue_zmpx[#self.preview_queue]
  local last_preview_zmpy = self.preview_queue_zmpy[#self.preview_queue]

  if self.t_last_step+self.t_step_duration >=t then
    --Old step
    table.insert(self.preview_queue,last_preview_item)
    table.insert(self.preview_queue_zmpx,last_preview_zmpx)
    table.insert(self.preview_queue_zmpx,last_preview_zmpy)
  else --New step
    local supportLeg
    local uLeft_now, uRight_now, uTorso_now, uLeft_next, uRight_next, uTorso_next
    if last_preview_item.supportLeg==2 then 
      supportLeg = 0 
    else
      supportLeg = 1-last_preview_item.supportLeg
    end
    uLeft_now, uRight_now, uTorso_now, uLeft_next,
      uRight_next, uTorso_next, uSupport =
        step_planner:get_next_step_velocity(
          last_preview_item.uLeft_next,
          last_preview_item.uRight_next,
          step_planner.get_torso(last_preview_item.uLeft_next,last_preview_item.uRight_next),
          supportLeg,false)
    local new_preview_item = {}
    new_preview_item.uLeft_now = uLeft_now
    new_preview_item.uLeft_next = uLeft_next
    new_preview_item.uRight_now = uRight_now
    new_preview_item.uRight_next = uRight_next
    new_preview_item.supportLeg = supportLeg

    table.insert(self.preview_queue,new_preview_item)
    table.insert(self.preview_queue_zmpx,uSupport[1])
    table.insert(self.preview_queue_zmpy,uSupport[2])
  end
end

local function get_com(self)


end


--]]







-- step_definition format
--{ Supportfoot relstep zmpmod duration steptype }--
-- Provide initial (relative) feet positions
local function generate_step_queue(solver,step_definition,uLeftI,uRightI)
  -- Modify the initial point...
  local uLeft  = vector.pose(unpack(uLeftI))
  local uRight = vector.pose(unpack(uRightI))

  -- Make the step_queue table
  solver.step_queue = {}
  for _,step_def in ipairs(step_definition) do
    local supportLeg = step_def[1]
    -- Form the Support Leg positions
    -- Perform a table copy
    if supportLeg==0 then
      -- left support
      uRight = util.pose_global(step_def[2],uRight)
      --step_queue_element.zaRight = zaRight + vector.new(step_def[3]);
    elseif supportLeg==1 then
      -- right support
      uLeft  = util.pose_global(step_def[2],uLeft)
      -- step_queue_element.zaLeft = zaLeft + vector.new(step_def[3]);
    elseif supportLeg==2 then --DS
      -- Double Support: Body height change
      -- step_queue_element.zaRight = zaRight - vector.new(step_def[3]);
      -- step_queue_element.zaLeft  = zaLeft  - vector.new(step_def[3]);
    end
    -- Insert the element
    table.insert(solver.step_queue,{
      supportLeg = supportLeg,
      uLeft      = vector.pose{unpack(uLeft)}, --copy the table
      uRight     = vector.pose{unpack(uRight)},
      zaLeft     = vector.zeros(2),
      zaRight    = vector.zeros(2),
      zmp_mod    = vector.new(step_def[3]),
      duration   = step_def[4],
      step_type  = step_def[5] or 0
    })
    --
  end
  -- Reset the queue_index
  solver.step_queue_index = 1
  solver.last_last = false  
end

local function update_preview(solver, t, supportX, supportY)
  local step_queue_element, t_expire
  local idx, phF = solver.step_queue_index, 0
  
  if not solver.step_queue_time then
    -- Initiate the step_queue time if this is the first step
    local dur = solver.step_queue[1].duration
    solver.step_queue_time = t + dur
    t_expire = dur
  else
    t_expire = solver.step_queue_time - t
  end

  -- Check if we are a new step
  if t_expire<=0 then
    solver.last_step = idx==#solver.step_queue
    -- Check if any elements left in the queue
    if solver.last_step then return'done' end
    -- Update the index in our step queue
    idx = idx + 1
    solver.step_queue_index = idx
    step_queue_element = solver.step_queue[idx]
    print'====='
    util.ptable(step_queue_element)
    print'====='
    -- Add the duration of this next step
    solver.step_queue_time = t + step_queue_element.duration
  else
    -- Just grab the element
    step_queue_element = solver.step_queue[idx]
    last_step = idx==#solver.step_queue
    if solver.last_step then
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
  local preview  = solver.preview
  local nPreview = preview.nPreview
  local zmp_x = preview.zmp_x
  local zmp_y = preview.zmp_y
  -- Shift over
  zmp_x:storage():lshift()
  zmp_y:storage():lshift()
  -- Add the final elements
  --print('uSupportF',uSupportF[1],uSupportF[2])
  zmp_x[nPreview] = uSupportF[1]
  zmp_y[nPreview] = uSupportF[2]

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
  last  = last  + 1
  preview.first = first + 1
  preview.last  = last
  --
  preview.phs[last]    = phF
  preview.uLeft[last]  = uLeftF
  preview.uRight[last] = uRightF
  preview.supportLegs[last] = supportLegF
end

local function solve_preview(solver)
  -- 3x2 state matrix
  -- x: torso, ?, ?
  -- y: torso, ?, ?
  local preview = solver.preview
  local current_state = preview.state:clone()
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
  -- Grab the immediate control input
  local u_x    = torch.dot(K1_row,preview.zmp_x)
  local u_y    = torch.dot(K1_row,preview.zmp_y)
  u[1] = u[1] - u_x
  u[2] = u[2] - u_y

  -- Update the state
  preview.state:mm(preview.A,current_state):add(
    torch.ger( preview.B, u )
  )
  -- Update the clock
  preview.clock = preview.clock + preview.ts
end

local function init_preview(solver,uTorso,uLeft,uRight,t)
  print("INITSTART")
  local preview = solver.preview
  assert(preview,'Please precompute the preview engine!')
  
  local nPreview = preview.nPreview
  -- Generate the x and y zmp trajectories
  -- First, fill the first preview window with where we are
  -- TODO: Splice in a previous preview, or something?
  preview.zmp_x = torch.Tensor(nPreview):fill(uTorso[1])
  preview.zmp_y = torch.Tensor(nPreview):fill(uTorso[2])
  --
  local phs = {}
  local supportLegs   = {}
  local uLeftTargets  = {}
  local uRightTargets = {}
  for i=1,nPreview do
    -- double support to start
    -- TODO: Take possibly a single support start?
    table.insert(phs,0)
    table.insert(supportLegs,2)
    table.insert(uLeftTargets,vector.pose{unpack(uLeft)})
    table.insert(uRightTargets,vector.pose{unpack(uRight)})
  end
  -- Save data to our solver
  preview.first = 1
  preview.last  = nPreview
  preview.phs   = phs
  preview.supportLegs = supportLegs
  preview.uLeft  = uLeftTargets
  preview.uRight = uRightTargets
  -- Our first state:
  preview.state = torch.Tensor{{uTorso[1],uTorso[2]},{0,0},{0,0}}
  -- Our preview clock start time
  preview.clock = t
end

-- preview_interval: How many seconds into the future should we preview?
-- ts: At what granularity should we solver?
local function compute_preview( solver, preview_interval, ts, save_file )
  
print("INITSTART")

  -- Preview parameter defaults
  preview_interval = preview_interval or 1.50 -- 1500ms
  ts = ts or 0.010 -- 10ms timestep
  local nPreview = math.ceil(preview_interval / ts)
  local r_q = 5e-5 -- balancing parameter for optimization
  --
  local tZMP  = solver.tZMP
  -- Cache some common operations
  local ts_integrated  = (ts^2)/2
  local ts_twice_integrated = (ts^3)/6
  local tZMP_sq = tZMP^2

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
    for j=k,1,-1 do pu_row[j] = pu0[k-j+1] end
  end
  local balancer = torch.eye(nPreview):mul(r_q)

  -- Exports
  solver.preview = {}
  -- TODO: Make this more efficient
  -- Cache the transpose
  local pu_trans = pu:t()
  local tmp =  torch.mm(pu_trans,pu):add(balancer)
  --print( 'singular?',torch.determinant(tmp) )
  local inv = torch.inverse( tmp )
  local K1  = -torch.mm( inv, pu_trans )
  
  solver.preview.K1    = K1
  solver.preview.K1_px = K1 * px
  -- Make the discrete control matrices
  solver.preview.A = torch.Tensor({
    {1,ts,ts_integrated},
    {0,1, ts},
    {0,0, 1}
  })
  solver.preview.B = torch.Tensor({
    ts_twice_integrated,
    ts_integrated,
    ts
  })
  -- Save preview data
  solver.preview.nPreview = nPreview
  solver.preview.ts = ts

  if save_file then
    local f = io.open(save_file,'w')
    local data_str = tostring( carray.double(K1:storage():pointer(),K1:nElement()) )
    f:write(data_str)
    f:close()
  end


  print("computed")
end

local function get_preview_com(solver)
  -- Yield the desired torso pose
  local state_position = solver.preview.state:select(1,1)
  local state_velocity = solver.preview.state:select(1,2)
  local state_accel    = solver.preview.state:select(1,3)
  return vector.pose{state_position[1],state_position[2],0},
    vector.pose{state_velocity[1],state_velocity[2],0},
    vector.pose{state_accel[1],state_accel[2],0}
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
  s.get_foot_square = get_foot_square  
  -- ZMP Preview
  s.compute_preview = compute_preview
  s.init_preview    = init_preview
  s.update_preview  = update_preview
  s.solve_preview   = solve_preview
  s.get_preview_com = get_preview_com
  -- General
  s.generate_step_queue = generate_step_queue
	return s
end

return libZMP