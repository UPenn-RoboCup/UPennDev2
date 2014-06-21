dofile'include.lua'

-- Important libraries in the global space
local libs = {
  'Config',
  'Body',
  'unix',
  'util',
  'vector',
	'fun',
}

-- Load the libraries
for _,lib in ipairs(libs) do _G[lib] = require(lib) end
if torch then torch.Tensor = torch.DoubleTensor end
-- mp
mp = require'msgpack'
getch=require'getch'
-- ffi
ok, ffi = pcall(require,'ffi')
ok = nil

local si = require'simple_ipc'

-- FSM communicationg
local fsm_chs = {}

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
rpc_ch = si.new_requester(Config.net.reliable_rpc)

print( util.color('FSM Channel','yellow'), table.concat(fsm_chs,' ') )
print( util.color('SHM access','blue'), table.concat(shm_vars,' ') )


targetvel={0,0,0}
targetvel_new={0,0,0}

function process_keyinput()
  local byte=getch.block();
  if byte then
    -- Walk velocity setting
    if byte==string.byte("i") then      targetvel_new[1]=targetvel[1]+0.02;
    elseif byte==string.byte("j") then  targetvel_new[3]=targetvel[3]+0.1;
    elseif byte==string.byte("k") then  targetvel_new[1],targetvel_new[2],targetvel_new[3]=0,0,0;
    elseif byte==string.byte("l") then  targetvel_new[3]=targetvel[3]-0.1;
    elseif byte==string.byte(",") then  targetvel_new[1]=targetvel[1]-0.02;
    elseif byte==string.byte("h") then  targetvel_new[2]=targetvel[2]+0.02;
    elseif byte==string.byte(";") then  targetvel_new[2]=targetvel[2]-0.02;

    elseif byte==string.byte("1") then      
      body_ch:send'init'
    elseif byte==string.byte("2") then      
      motion_ch:send'bias'


    elseif byte==string.byte("8") then  
      motion_ch:send'stand'
      body_ch:send'stop'
      if mcm.get_walk_ismoving()>0 then 
        print("requesting stop")
        mcm.set_walk_stoprequest(1) 
      end
    elseif byte==string.byte("9") then  
      motion_ch:send'hybridwalk'
    end

    local vel_diff = (targetvel_new[1]-targetvel[1])^2+(targetvel_new[2]-targetvel[2])^2+(targetvel_new[3]-targetvel[3])^2
    if vel_diff>0 then
      targetvel[1],targetvel[2],targetvel[3] = targetvel_new[1],targetvel_new[2],targetvel_new[3]
      print(string.format("Target velocity: %.3f %.3f %.3f",unpack(targetvel)))
      mcm.set_walk_vel(targetvel)
    end
  end
end

local t_last = Body.get_time()
local tDelay = 0.005*1E6

 while 1 do
  process_keyinput(); --why nonblocking reading does not work?
end
