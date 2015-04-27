#!/usr/local/bin/luajit -i
local ok = pcall(dofile, 'include.lua')
if not ok then pcall(dofile, '../include.lua') end

print('Robot', Config.net.robot.wired)

-- Important libraries in the global space
mp = require'msgpack.MessagePack'
si = require'simple_ipc'
local libs = {
  'Config',
  'Body',
  'util',
  'vector',
  'torch',
  'ffi',
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

local rpc_req = si.new_requester(Config.net.rpc.tcp_reply, Config.net.robot.wired)
local rpc_udp = si.new_sender(Config.net.robot.wired, Config.net.rpc.udp)

-- FSM communicationg
fsm_chs = {}
local fsm_send = function(t, evt)
  rpc_req:send(mp.pack({fsm=t.fsm,evt=evt}))
  local data = unpack(rpc_req:receive())
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
		-- Use REQ/REP to get data
		local ret = rpc_req:send(mp.pack(tbl))
		local data = unpack(rpc_req:receive())
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
    table.insert(shm_vars,name)
    _G[name] = setmetatable({shm=name},{__index=shm_send})
  end
end

print(util.color('FSM Channel', 'yellow'), table.concat(fsm_chs, ' '))
print(util.color('SHM access', 'blue'), table.concat(shm_vars, ' '))

if arg and arg[-1]=='-i' and jit then
  -- Interactive LuaJIT
  package.path = package.path..';'..HOME..'/Tools/iluajit/?.lua'
  dofile'Tools/iluajit/iluajit.lua'
end
