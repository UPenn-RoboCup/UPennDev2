dofile'include.lua'

-- Important libraries in the global space
local libs = {
  'Config',
  'unix',
  'util',
  'vector',
  'torch',
  'simple_ipc'
}

mp = require'msgpack'

-- Load the libraries
for _,lib in ipairs(libs) do _G[lib] = require(lib) end
if torch then torch.Tensor = torch.DoubleTensor end

-- Requester
print('Connected to',Config.net.robot.wired,Config.net.reliable_rpc)
local rpc_zmq = simple_ipc.new_requester(Config.net.reliable_rpc,Config.net.robot.wired)

-- FSM communicationg
local fsm_send = function(t,evt)
  local ret = rpc_zmq:send(mp.pack({fsm=t.fsm,evt=evt}))
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
  if func:find('get_')==1 then
    local segment_idx = func:find('_',5)
    assert(segment_idx,'no segment!')
    local segment = func:sub(5,segment_idx-1)
    assert(#segment>0,'Invalid segment!')
    local key = func:sub(segment_idx+1)
    assert(#key>0,'Invalid key!')
    tbl.segment = segment
    tbl.key = key
    tbl.shm = t.shm
  
    return function()
      rpc_zmq:send(mp.pack(tbl))
      local data = rpc_zmq:receive()
      local result = mp.unpack(data)
      return vector.new(result)
    end
    
  elseif func:find('set_')==1 then
    local segment_idx = func:find('_',5)
    assert(segment_idx,'no segment!')
    local segment = func:sub(5,segment_idx-1)
    assert(#segment>0,'Invalid segment!')
    local key = func:sub(segment_idx+1)
    assert(#key>0,'Invalid key!')
    --print('shm tbl',t.shm,segment,key)
    tbl.segment = segment
    tbl.key = key
    tbl.shm = t.shm
    
    return function(val)
      tbl.val = val
      rpc_zmq:send(mp.pack(tbl))
      local data = rpc_zmq:receive()
    end
  end

  return function()
    print'Bad SHM function!'
    return nil
  end
  
end

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

-- Import Body
Body = require'Body'
