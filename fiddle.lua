#!/usr/local/bin/luajit -i

if IS_FIDDLE then return end
local ok = pcall(dofile, '../include.lua')
if not ok then pcall(dofile, 'include.lua') end

-- Important libraries in the global space
mp = require'msgpack.MessagePack'
si = require'simple_ipc'
local libs = {
	'ffi',
	'torch',
  'util',
  'vector',
	'Body',
}
-- Load the libraries
for _,lib in ipairs(libs) do
  local ok, lib_tbl = pcall(require, lib)
  if ok then
    _G[lib] = lib_tbl
  else
    print("Failed to load", lib)
		--print(lib_tbl)
  end
end

-- FSM communicationg
fsm_chs = {}
for _,sm in ipairs(Config.fsm.enabled) do
  local fsm_name = sm..'FSM'
  local ch = si.new_publisher(fsm_name.."!")
  _G[sm:lower()..'_ch'] = ch
  fsm_chs[fsm_name] = ch
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

-- Local RPC for testing
rpc_ch = si.new_requester'rpc'

--print(util.color('FSM Channel', 'yellow'), table.concat(fsm_chs, ' '))
--print(util.color('SHM access', 'blue'), table.concat(shm_vars,  ' '))

IS_FIDDLE = true

if arg and arg[-1]=='-i' and jit then
  -- Interactive LuaJIT
  package.path = package.path..';'..HOME..'/Tools/iluajit/?.lua'
  dofile'Tools/iluajit/iluajit.lua'
end
