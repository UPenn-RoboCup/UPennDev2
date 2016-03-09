#!/usr/bin/env luajit
dofile'../include.lua'
local si = require'simple_ipc'
local util = require'util'
local munpack = require'msgpack'.unpack
local mpack = require'msgpack'.pack
local movearm = require'movearm'
local T = require'Transform'
local vector = require'vector'

LIVE_DEMO = true
local Body
local getch = require'getch'
if LIVE_DEMO then
  Body = require'Body'
end

local counter = 0
local f_adlib
local prevPath
local prevWeights = vector.new{0,0,0}

local qMid = vector.new{0, 0.759218, 0, -1.39626, 0, 0, 0}
local qGravity = vector.new({0, 60, 90, -120, -90, 0, 0})*DEG_TO_RAD;

local state = 'plan'
local interaction_points = {}

local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

local function get_config(path)
	print(unpack(path))
	local data = Config
	for i, v in ipairs(path) do
		data = data[v]
		if not data then return end
	end
	return data
end

local function q2f(q)
  -- % Shoulder Yaw (tight)
  local f1 = q[2]^2
  
  -- % Usage is the mid of range of motion
  local f2 = vector.normsq(q - qMid)
  
  -- % Similar config... (from Config_Arm)
  local f3 = vector.normsq(q - qGravity)
  
  return vector.new{f1, f2, f3}
end

--local WEIGHT_ADJ = 0.05
local WEIGHT_ADJ = 0.01

local function get_armplan(plan)
	print('\narm_wizard | Received a plan')
  local replan = (state=='adlib')
  state = 'plan'
  
  if type(plan)~="table" then return"Nope" end
  util.ptable(plan.right)
  
  local features, alphas = {}, {}
  if replan then
    print('replan', replan)
    -- Grab the new point from the human
    local qHgood = plan.right.qRArm0
    table.insert(interaction_points, qHgood)
    -- Grab the bad points by looking back in time
    assert(prevPath, "There should be a previous plan")
    local alphas = {}
    
    for i, qPrev in ipairs(prevPath) do
      
      table.insert(features, q2f(vector.new(qPrev)))
      
      -- Reverse order of alpha
      table.insert(alphas, 1, -1*math.exp( -(i-1)/2 ) )
      
    end
    -- * prevPath[#prevPath - i + 1]
    
    local fsum = q2f(qHgood)
    print("Good Features", fsum)
    print("Bad features", features[#features])
    for i, f in ipairs(features) do
      fsum = fsum + alphas[i] * f
    end
    print("c_tight c_usage c_similar")
    print("Current weights", prevWeights)
    print("Update weights...", fsum)
    prevWeights = prevWeights - WEIGHT_ADJ * fsum
    print("New weights!", prevWeights)
    
  else
    -- New plan
  end
  
  -- IK weights
  plan.right.weights[1] = prevWeights[2]
  plan.right.weights[3] = prevWeights[1]
  plan.right.weights[6] = prevWeights[3]
  plan.right.wh = prevWeights
  
  plan.right.interactions = interaction_points
  
	local lco, rco = movearm.goto(plan.left, plan.right)

  local Js, nulls

	local wpath = {}
	--
	local lpath = {}
	if type(lco)=='thread' then
		while coroutine.status(lco)~='dead' do
			local okL, qLWaypoint, qWaistpoint, j, n = coroutine.resume(lco)
			table.insert(lpath, qLWaypoint)
			if qWaistpoint then table.insert(wpath, qWaistpoint) end
      if j and n then Js, nulls = j, n end
		end
	else
		print('lco', lco)
	end
	local rpath = {}
	if type(rco)=='thread' then
		while coroutine.status(rco)~='dead' do
			local okR, qRWaypoint, qWaistpoint = coroutine.resume(rco)
			table.insert(rpath, qRWaypoint)
			if qWaistpoint then table.insert(wpath, qWaistpoint) end
      if LIVE_DEMO then
        io.write(#rpath, #rpath%5==0 and '\n' or '\t')
        if getch.nonblock() then istop=1; break end
        Body.set_rarm_command_position(qRWaypoint)
        unix.usleep(1e6/10)
      end
		end
	else
		print('rco', rco)
	end
  table.remove(lpath)
  table.remove(rpath)
  table.remove(wpath)
  
  --[[
  if LIVE_DEMO then
    for i, q in ipairs(rpath) do
      io.write(i, '/', #rpath,'\n')
      if getch.nonblock() then break end
      Body.set_rarm_command_position(q)
      unix.usleep(1e6/10)
    end
  end
  --]]

	-- Optimize the paths
	--[[
	print('\n==\nDone the initial plan!')
	util.ptable(plan.left)
	print('=====\n\n')
	if type(lpath)=='table' then
		local lpath1, wpath1 = movearm.optimize(lpath, rpath, wpath)
		if lpath1 then lpath = lpath1 end
		if wpath1 then wpath = wpath1 end
	end
	--]]

	-- TODO: Check that the waist was not twice populated
  counter = counter + 1
	print(counter, 'Sending the paths', #lpath, #rpath, #wpath)
  
  -- -- -- -- -- -- -- -- -- -- --
  local t_log = unix.time()
  -- Save the path
  local fname = string.format('/tmp/plan_%d_%d.arm', t_log, counter)
  local f = io.open(fname, 'w')
  f:write(mpack{
    plan = plan,
    lpath = lpath,
    rpath = rpath,
    wpath = wpath,
    counter = counter
  })
  f:close()
  -- -- -- -- -- -- -- -- -- -- --
  -- Manage adlib logs
  if f_adlib then f_adlib:close() end
  local fname = string.format('/tmp/adlib_%d_%d.arm', t_log, counter)
  f_adlib = io.open(fname, 'w')
  
  mattorch.saveTable(string.format('/tmp/js_%d_%d.arm', t_log, counter), {
    Js = Js,
    nulls = nulls,
  })
  
  --]]
  
  prevPath = lpath
  
	return {lpath, rpath, wpath}
end

local function adlib(plan)
  --print('\narm_wizard | Received adlib')
  local newAdLib = state =='plan'
  state = 'adlib'
  --util.ptable(plan)
  --print()
  
  -- Grab the planners
  local rPlanner = movearm.rPlanner
  local rPlan = plan.right
  --util.ptable(rPlan)

  local nq = #rPlan.qRArm0
  local nNull = 1 -- For now
  
  -- In degrees... from 0 to 10 degrees away from the keyboard baseline
  local dGamma = util.procFunc(rPlan.gamma, 3, 10)
  local dNull = {dGamma}

  local nullspace, J, Jinv = rPlanner:get_nullspace(rPlan.qRArm0, rPlan.qRArm0)
  local U, S, V = torch.svd(nullspace)
  
  --print("GETTING DIST")
	if #rPlan.tr==6 then
		rPlan.tr = T.transform6D(rPlan.tr)
	elseif #rPlan.tr==7 then
		rPlan.tr = T.from_quatp(rPlan.tr)
	end
	local dp, drpy = rPlanner:get_distance(rPlan.tr, vector.new(rPlan.qRArm0), {0,0})
  --print("DIST", dp)
	-- Check if we are within threshold
  --[[
	dTF = {vnorm(dp), vnorm(drpy)}
	if dTF[1] < POS_THRESH and dTF[2] < 3*DEG_TO_RAD then break end
  --]]
	-- Form our desired velocity
	local vwTarget0 = {
		dp[1], dp[2], dp[3],
		drpy[1], drpy[2], drpy[3],
	}
	local vwTarget = torch.Tensor(vwTarget0)
  
	-- Joint velocities to accomplish the se(3) velocities
	local dqdtArm = torch.mv(Jinv, vwTarget)

  local dGAIN = 0.01
  local qAdlibR = vector.copy(rPlan.qRArm0)
  for i=1, nNull do
    local nullDir = U:select(2, i)
    
    local dqN = dNull[i] * vector.new(nullDir) * dGAIN
    
    -- Maybe consistent?
    --[[
    local dotDir = torch.dot(nullDir, torch.Tensor(rPlanner.qMax))
    print('Directions', dotDir, dNull[i])
    if dotDir<0 then dqN = dqN * -1 end
    --]]
    if nullDir[3]<0 then dqN = dqN * -1 end
    
    qAdlibR = qAdlibR + dqN + dqdtArm * 10 * dGAIN
    --print('dNull '..i, dqN * RAD_TO_DEG, 'deg')
  end
  
  --print(counter, 'Sending adlib', #qAdlibL)
  rPlan.qAdlib = qAdlibR
  rPlan.counter = counter
  -- f_adlib should be open...
  assert(f_adlib, "Why no adlib?")
  f_adlib:write(mpack(rPlan))
  
  if LIVE_DEMO then
      Body.set_rarm_command_position(qAdlibR)
  end
  
  -- TODO: Return three arrays of vectors {left, right, waist}
  return {qAdlibR}
  
end


local poller, lut
local channels = {}
local function cb(skt)
  --print("\n!!GOT MESSAGE!!\n")
	local ch_id = lut[skt]
	local ch = channels[ch_id]
	local mdata = unpack(skt:recv_all())
	local meta = munpack(mdata)
	local result = ch.process(meta)
	if ch.kind=='rep' then
		-- if REQ/REP then reply
		local ret = ch:send(mpack(result))
	end
end

local plan_ch = si.new_replier('armplan')
plan_ch.callback = cb
plan_ch.process = get_armplan
table.insert(channels, plan_ch)

local config_ch = si.new_replier('config')
config_ch.callback = cb
config_ch.process = get_config
table.insert(channels, config_ch)

local adlib_ch = si.new_replier('adlib')
adlib_ch.callback = cb
adlib_ch.process = adlib
table.insert(channels, adlib_ch)

poller = si.wait_on_channels(channels)

local function shutdown()
  print('Saving...')
  local f = io.open('/tmp/interactions.mp', 'w')
  f:write(mpack(interaction_points))
  f:close()
  poller:stop()
end

local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

lut = poller.lut
poller:start()
