#!/usr/bin/env luajit -i
dofile'include.lua'

-- Important libraries in the global space
local libs = {
  'Config',
  'unix',
  'util',
  'vector',
  'torch',
  'simple_ipc',
  'udp'
}

mp = require'msgpack'
Body = require'Body'

-- Load the libraries
for _,lib in ipairs(libs) do _G[lib] = require(lib) end
if torch then torch.Tensor = torch.DoubleTensor end

-- Requester
print('REQ |',Config.net.robot.wired,Config.net.reliable_rpc)
local rpc_req =
  simple_ipc.new_requester(Config.net.reliable_rpc,Config.net.robot.wired)
unix.usleep(1e5)

-- Publisher
print('PUB |',Config.net.robot.wired,Config.net.reliable_rpc2)
local rpc_pub =
  simple_ipc.new_publisher(Config.net.reliable_rpc2,true,Config.net.robot.wired)
unix.usleep(1e5)

-- UDP
print('UDP |',Config.net.robot.wired,Config.net.unreliable_rpc)
local rpc_udp = udp.new_sender(Config.net.robot.wired,Config.net.unreliable_rpc)

-- FSM communicationg
local fsm_send = function(t,evt)
  local ret = rpc_req:send(mp.pack({fsm=t.fsm,evt=evt}))
end

local listing = unix.readdir(HOME..'/Player')
-- Add all FSM directories that are in Player
local fsm_ch_vars = {}
for _,sm in ipairs(listing) do
  local found = sm:find'FSM'
  if found then
    -- make GameFSM to game_ch
    local name = sm:sub(1,found-1):lower()..'_ch'
    table.insert(fsm_ch_vars,name)
    -- Put into the global space
    _G[name] = {fsm=sm,send=fsm_send}
  end
end

-- Shared memory
local shm_send = function(t,func)
  local tbl = {}
  tbl.shm = t.shm
  tbl.access = func
  
  return function(val)
		if val then tbl.val=val end
		local packed = mp.pack(tbl)
		----[[
		if val then
			-- Just PUB to the robot for shm set
			tbl.val=val
			local ret = rpc_pub:send(mp.pack(tbl))
			return
		end
		--]]
		-- Use REQ/REP to get data
		rpc_req:send(packed)
		local data = rpc_req:receive()
    local result = mp.unpack(data)
    if type(result)=='table' then return vector.new(result) end
    return result
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

-- Useful constants
DEG_TO_RAD = math.pi/180
RAD_TO_DEG = 180/math.pi

print( util.color('FSM Channel','yellow'), table.concat(fsm_ch_vars,' ') )
print( util.color('SHM access','blue'), table.concat(shm_vars,' ') )

if arg and arg[-1]=='-i' and jit then
  -- Interactive LuaJIT
  package.path = package.path..';'..HOME..'/Tools/iluajit/?.lua'
  dofile'Tools/iluajit/iluajit.lua'
end
