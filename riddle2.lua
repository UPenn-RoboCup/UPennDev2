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

-- Load the libraries
for _,lib in ipairs(libs) do _G[lib] = require(lib) end
if torch then torch.Tensor = torch.DoubleTensor end
-- mp
mp = require'msgpack'

-- Requester
print('Connected to',Config.net.robot.wired,Config.net.reliable_rpc)
rpc_zmq = simple_ipc.new_requester(Config.net.reliable_rpc,Config.net.robot.wired)

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
local listing = unix.readdir(HOME..'/Memory')
local shm_vars = {}
for _,mem in ipairs(listing) do
  local found, found_end = mem:find'cm'
  if found then
    local name = mem:sub(1,found_end)
    table.insert(shm_vars,name)
    require(name)
    -- Now reset the functions
    for k,f in pairs(_G[name]) do
      if k:find('get_') or k:find('set_') then
        _G[name][k] = function(val)
          tbl.val = val
          rpc_zmq:send(mp.pack({shm=name,segkeyfun=f,val=val}))
          local data = rpc_zmq:receive()
          local result = mp.unpack(data)
          return vector.new(result)
        end
      end -- find get/set
    end -- for each item in the shm
  end
end

-- Make the body calls
local body_send = function(t,func)
  return function(val)
    rpc_zmq:send(mp.pack({body=func,bargs=val}))
    local data = rpc_zmq:receive()
    local result = mp.unpack(data)
    return vector.new(result)
  end
end

Body = setmetatable({},{__index=body_send})

-- Useful constants
DEG_TO_RAD = math.pi/180
RAD_TO_DEG = 180/math.pi

print( util.color('FSM Channel','yellow'), table.concat(fsm_ch_vars,' ') )
print( util.color('SHM access','blue'), table.concat(shm_vars,' ') )
