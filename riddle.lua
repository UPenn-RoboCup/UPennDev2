#!/usr/local/bin/luajit -i
local ok = pcall(dofile, 'include.lua')
if not ok then pcall(dofile, '../include.lua') end

print('Robot', Config.net.robot.wired)

-- Important libraries in the global space
mp = require'msgpack.MessagePack'
si = require'simple_ipc'
local libs = {
  'Config',
  'util',
  'vector',
  'torch',
	'unix'
}
-- Load the libraries
for _,lib in ipairs(libs) do
  local ok, lib_tbl = pcall(require, lib)
  if ok then
    _G[lib] = lib_tbl
  else
    print("Failed to load", lib)
  end
end

-- Track the network usage
local sent_usage = {
	fsm = {},
	shm = {},
}
local recv_usage = {
	fsm = {},
	shm = {},
}
function net()
	for name, usage in pairs(sent_usage) do
		local sum = 0
		for i, use in ipairs(usage) do
			local t_use, bytes_used = unpack(use)
			sum = sum + bytes_used
		end
		print(string.format('%s sent %d bytes', name, sum))
	end
	for name, usage in pairs(recv_usage) do
		local sum = 0
		for i, use in ipairs(usage) do
			local t_use, bytes_used = unpack(use)
			sum = sum + bytes_used
		end
		print(string.format('%s received %d bytes', name, sum))
	end
end


local rpc_req = si.new_requester(Config.net.rpc.tcp_reply, Config.net.robot.wired)
local rpc_udp = si.new_sender(Config.net.robot.wired, Config.net.rpc.udp)

-- FSM communicationg
fsm_chs = {}
local fsm_send = function(t, evt)
	local msg = mp.pack({fsm=t.fsm,evt=evt})
	table.insert(sent_usage.fsm, {unix.time(), #msg})
  rpc_req:send(msg)
  local data = unpack(rpc_req:receive())
	table.insert(recv_usage.fsm, {unix.time(), #data})
  local result = mp.unpack(data)
  return result
end
for sm, en in pairs(Config.fsm.enabled) do
  local fsm_name = sm..'FSM'
  table.insert(fsm_chs, fsm_name)
  _G[sm:lower()..'_ch'] = en and {fsm=sm, send=fsm_send} or si.new_dummy()
end

-- Shared memory
local shm_send = function(t, func)
  local seg, key = func:match('_(%a+)_(%g+)')
  local tbl = {shm = t.shm, seg = seg, key = key}
  return function(val)
		tbl.val = val
		local msg = mp.pack(tbl)
		table.insert(sent_usage.shm, {unix.time(), #msg})
		local ret = rpc_req:send(msg)
		local data = unpack(rpc_req:receive())
		table.insert(recv_usage.shm, {unix.time(), #data})
    local result = mp.unpack(data)
    return type(result)=='table' and vector.new(result) or result
  end
end

-- This should overwrite memory calls...
local listing = unix.readdir(HOME..'/Memory')
local shm_vars = {}
for _,mem in ipairs(listing) do
  local found, found_end = mem:find'cm'
  if found then
    local name = mem:sub(1,found_end)
    table.insert(shm_vars, name)
    _G[name] = setmetatable({shm=name},{__index=shm_send})
  end
end
_G.dcm = setmetatable({shm='dcm'},{__index=shm_send})

_G.Body = {}
-- Be *super* careful here
local actuators = require'libDynamixel'.nx_registers
for actuator in pairs(actuators) do
	for part, jlist in pairs(Config.parts) do
		local idx1, idx2 = jlist[1], jlist[#jlist]
		Body['get_'..part:lower()..'_'..actuator] = function()
			local vals = dcm['get_actuator_'..actuator]()
			return vector.new{unpack(vals, idx1, idx2)}
		end
		Body['set_'..part:lower()..'_'..actuator] = function(vals)
			return dcm['set_actuator_'..actuator](vals)
		end
	end
end
local sensors = require'libDynamixel'.registers_sensor
for _, sensor in ipairs(sensors) do
	for part, jlist in pairs(Config.parts) do
		local idx1, idx2 = jlist[1], jlist[#jlist]
		Body['get_'..part:lower()..'_'..sensor] = function()
			local vals = dcm['get_sensor_'..sensor]()
			return vector.new{unpack(vals, idx1, idx2)}
		end -- Get
	end
end

function pstart(scriptname, idx)
	if scriptname=='rpc' then return end
	local request
	if tostring(idx) then
		request = string.format('pstart("%s", %d)', scriptname, idx)
	else
		request = string.format('pstart("%s")', scriptname)
	end
	local msg = mp.pack({raw = request})
	rpc_req:send(msg)
  local data = unpack(rpc_req:receive())
	if type(data)~='string' then return end
	return mp.unpack(data)
end

function pkill(scriptname, idx)
	if scriptname:find'rpc' then return end
	local request
	if tostring(idx) then
		request = string.format('pkill("%s", %d)', scriptname, idx)
	else
		request = string.format('pkill("%s")', scriptname)
	end
	local msg = mp.pack({raw = request})
	rpc_req:send(msg)
  local data = unpack(rpc_req:receive())
	if type(data)~='string' then return end
	return mp.unpack(data)
end

print(util.color('FSM Channel', 'yellow'), table.concat(fsm_chs, ' '))
print(util.color('SHM access', 'blue'), table.concat(shm_vars, ' '))

if arg and arg[-1]=='-i' and jit then
  -- Interactive LuaJIT
  package.path = package.path..';'..HOME..'/Tools/iluajit/?.lua'
  dofile'Tools/iluajit/iluajit.lua'
end
