#!/usr/bin/env luajit
dofile'../include.lua'
local si = require'simple_ipc'
local util = require'util'
local munpack = require'msgpack'.unpack
local mpack = require'msgpack'.pack
local movearm = require'movearm'
local T = require'Transform'
local vector = require'vector'

local function get_config(path)
	print(unpack(path))
	local data = Config
	for i, v in ipairs(path) do
		data = data[v]
		if not data then return end
	end
	return data
end

local function get_armplan(plan)
	print('arm_wizard | Received a plan')

	local lco, rco = movearm.goto(plan.left, plan.right)


	local wpath = {}
	--
	local lpath = {}
	if type(lco)=='thread' then
		while coroutine.status(lco)~='dead' do
			local okL, qLWaypoint, qWaistpoint = coroutine.resume(lco)
			table.insert(lpath, qLWaypoint)
			if qWaistpoint then table.insert(wpath, qWaistpoint) end
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
		end
	else
		print('rco', rco)
	end

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
	print('Sending the paths',#lpath, #rpath, #wpath)
	return {lpath, rpath, wpath}
end

local function adlib(plan)
  print('\narm_wizard | Received adlib')
  --util.ptable(plan)
  --print()
  
  -- Grab the planners
  local lPlanner = movearm.lPlanner
  local lPlan = plan.left
  util.ptable(lPlan)

  local nq = #lPlan.qLArm0
  local nNull = 1 -- For now
  
  -- In degrees... from 0 to 10 degrees away from the keyboard baseline
  local dGamma = util.procFunc(lPlan.gamma+53, 5, 10)
  local dNull = {dGamma}

  --[[
  print('Gamma', lPlan.gamma, dGamma)
  print(unpack(lPlan.qLArm0))
  print('qL0', vector.new(lPlan.qLArm0) * RAD_TO_DEG, 'deg')
  print('qW0', vector.new(lPlan.qWaist0) * RAD_TO_DEG, 'deg')
  --]]
  local nullspace, J, Jinv = lPlanner:get_nullspace(lPlan.qLArm0, lPlan.qLArm0)
  local U, S, V = torch.svd(nullspace)
  --[[
  local subV = V:sub(1, nq, 1, lPlan.nNull):t()
  print()
  print('V')
  util.ptorch(V)
  print('subV')
  util.ptorch(subV)
  print('S')
  util.ptorch(S)
  print('Nullspace')
  util.ptorch(nullspace)
  print('Reconstructed Null')
  util.ptorch(U*torch.diag(S)*V:t())
  --]]
  --[[
  dqSafe = N * dq
  dqSafe = U * (S * V^T  * dq)
  dλ = (S * V^T  * dq)
  -- assume motion then, in the λ coordinate
  -- Use nNull columns of U, then.
  --]]
  --[[
  print('U')
  util.ptorch(U)
  print('U Columns')
  local U2 = U:sub(1, nq, 1, lPlan.nNull)
  util.ptorch(U2)
  -- Norm is 1 of each column
  print(torch.norm(U2))
  util.ptorch(U2*torch.diag(torch.Tensor{lPlan.gamma}))
  print('U other')
  util.ptorch(U:select(2, i))
  --]]
  local dGAIN = 0.005
  local qAdlibL = vector.copy(lPlan.qLArm0)
  for i=1, nNull do
    local nullDir = U:select(2, i)
    
    print('Directions', dotDir, dNull[i])
    local dqN = dNull[i] * vector.new(nullDir) * dGAIN
    
    -- Maybe consistent?
    local dotDir = torch.dot(nullDir, torch.Tensor(lPlanner.qMax))
    if dotDir<0 then dqN = dqN * -1 end
    
    qAdlibL = qAdlibL + dqN
    print('dNull '..i, dqN * RAD_TO_DEG, 'deg')
  end
  
  -- TODO: Return three arrays of vectors {left, right, waist}
  return {qAdlibL}
  
end

--[[
local lPlan = {
  qLArm0 = {0.294783, 0.00227923, -0.0851542, -0.353859, 0.404588, -0.0413827, -0.325158},
  qWaist0 = {0, 0},
  gamma = 5,
  nNull = 1
}
adlib({left=lPlan})
--]]

local poller, lut
local channels = {}
local function cb(skt)
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
  poller:stop()
end

local signal = require'signal'.signal
signal("SIGINT", shutdown)
signal("SIGTERM", shutdown)

lut = poller.lut
poller:start()
