dofile'include.lua'
local Body = require'Body'
local vector = require'vector'
local mp   = require'msgpack'
local colors = require'colors'

local up = nil
local t_last_log = nil
local t_first_log = nil
local j_last_cmd = nil

local function entry()
  local f = io.open('../Logs/larm.log','r')
  local data = f:read("*all")
  f:close()
  up = mp.unpacker( data )
  
  -- Get initial readings
  local jval = up:unpack()
  
  t_last_log = jval[#jval]
  j_last_cmd = vector.slice(jval,1,#jval-1)
  t_first_log = t_last_log
  
  local entry_str = string.format('Read %d bytes\nJoint angles: %s',#data,tostring(j_last_cmd))
  print( colors.wrap(entry_str,'green') )
end

local function update()
  local jval = up:unpack()
  if not jval then return false end
  local j_cmd = vector.slice(jval,1,#jval-1)
  local t_log = jval[#jval]
  Body.set_larm_command_position(j_cmd)
  unix.usleep( 1e6*(t_log-t_last_log) )
  t_last_log = t_log
  return true
end

local function exit()
  print( colors.wrap(
  string.format('Done %.1f seconds of logs',t_last_log-t_first_log),
  'red'))
end

local t0 = unix.time()
local t_last = t0
entry()
while true do
  local t = unix.time()
  if not update() then break end
  if t-t_last>1 then
    t_last = t
    print('Time',t_last_log-t_first_log)
  end
end
exit()