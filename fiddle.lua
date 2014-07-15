#!/usr/bin/env luajit -i

dofile'include.lua'

-- Important libraries in the global space
local libs = {
  'Config',
  'Body',
  'unix',
  'util',
  'vector',
  'torch',
	'fun',
}

-- Load the libraries
for _,lib in ipairs(libs) do _G[lib] = require(lib) end
-- mp
mp = require'msgpack'
-- ffi
ok, ffi = pcall(require,'ffi')
ok = nil

local si = require'simple_ipc'

-- FSM communicationg
fsm_chs = {}
--[[
local listing = unix.readdir(HOME..'/Player')
-- Add all FSM directories that are in Player
for _,sm in ipairs(listing) do
  local found = sm:find'FSM'
  if found then
    -- make GameFSM to game_ch
    local name = sm:sub(1,found-1):lower()..'_ch'
    table.insert(fsm_chs,name)
    -- Put into the global space
    _G[name] = si.new_publisher(sm.."!")
  end
end
--]]
for _,sm in ipairs(Config.fsm.enabled) do
  local fsm_name = sm..'FSM'
  table.insert(fsm_chs, fsm_name)
  _G[sm:lower()..'_ch'] = si.new_publisher(fsm_name.."!")
end

-- Shared memory
local listing = unix.readdir(HOME..'/Memory')
local shm_vars = {}
for _,mem in ipairs(listing) do
  local found, found_end = mem:find'cm'
  if found then
    local name = mem:sub(1,found_end)
    table.insert(shm_vars,name)
    require(name)
  end
end

-- RPC engine
--rpc_ch = si.new_requester(Config.net.reliable_rpc)

-- Body channel
body_ch = si.new_publisher'body!'
dcm_ch = si.new_publisher'dcm!'

print(util.color('FSM Channel','yellow'), table.concat(fsm_chs,' '))
print(util.color('SHM access','blue'), table.concat(shm_vars,' '))

if arg and arg[-1]=='-i' and jit then
  -- Interactive LuaJIT
  package.path = package.path..';'..HOME..'/Tools/iluajit/?.lua'
  dofile'Tools/iluajit/iluajit.lua'
end
