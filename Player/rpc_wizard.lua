dofile'include.lua'
local mp = require'msgpack'
local simple_ipc = require'simple_ipc'
local rep = simple_ipc.new_replier(5555,'*')
--local rep = simple_ipc.new_replier'upenn_rpc'
local util = require'util'

-- TODO: Require all necessary modules
require'vcm'
require'jcm'
require'mcm'

-- TODO: Require all necessary fsm channels
local simple_ipc = require'simple_ipc'
local fsm_channels = {}
-- TODO: Make coroutines for each FSM
for _,sm in ipairs(unix.readdir(CWD)) do
  if sm:find'FSM' then
    fsm_channels[sm] = simple_ipc.new_publisher(sm,true)
  end
end

while true do
  -- NOTE: has_more is innocuous in this situation
  local request, has_more = rep:receive()
  local rpc = mp.unpack(request)
  util.ptable(rpc)

  local status, reply
  -- Shared memory modification
  local shm = rpc.shm
  -- TODO: Make safety checks
  if shm then
    if rpc.val then
      -- Set memory
      local method = 'set_'..rpc.segment..'_'..rpc.key
      local func = _G[shm][method]
      -- Use a protected call
      pcall(func,rpc.val)
    elseif rpc.delta then
      -- Increment/Decrement memory
      local method = rpc.segment..'_'..rpc.key
      local func = _G[shm]['get_'..method]
      pcall(func,rpc.val)
      func = _G[shm]['set_'..method]
      pcall(func,rpc.val)
    else
      -- Get memory
      local method = 'get_'..rpc.segment..'_'..rpc.key
      local func = _G[shm][method]
      -- Use a protected call
      status, reply = pcall(func)
    end
  end -- if shm
  -- State machine events
  local fsm = rpc.fsm
  if fsm then
    local ch = fsm_channels[fsm]
    if ch and type(rpc.evt)=='string' then
      ch:send(rpc.evt)
    else
      reply = 'bad fsm rpc call'
    end
  end

  -- Send the result of the last request
  local ret = rep:send( mp.pack(reply) )
end