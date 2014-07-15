#!/usr/bin/env luajit
dofile'include.lua'

-- Important libraries in the global space
local libs = {
  'Config',
  'Body',
  'unix',
  'util',
  'vector',
}

-- Load the libraries
for _,lib in ipairs(libs) do _G[lib] = require(lib) end

-- mp
local mp = require'msgpack'
local getch=require'getch'
local si = require'simple_ipc'
require'mcm'
require'gcm'

-- FSM communicationg
local fsm_chs = {}

for _,sm in ipairs(Config.fsm.enabled) do
  local fsm_name = sm..'FSM'
  table.insert(fsm_chs, fsm_name)
  _G[sm:lower()..'_ch'] = si.new_publisher(fsm_name.."!")
end



-- RPC engine
rpc_ch = si.new_requester(Config.net.reliable_rpc)


targetvel={0,0,0}
targetvel_new={0,0,0}


print("Game state:",gcm.get_game_state())

function process_keyinput()
  local byte=getch.block();

  if byte then
    if gcm.get_game_state()==6 then --Testing state
    -- Walk velocity setting
      if byte==string.byte("i") then      targetvel_new[1]=targetvel[1]+0.02;
      elseif byte==string.byte("j") then  targetvel_new[3]=targetvel[3]+0.1;
      elseif byte==string.byte("k") then  targetvel_new[1],targetvel_new[2],targetvel_new[3]=0,0,0;
      elseif byte==string.byte("l") then  targetvel_new[3]=targetvel[3]-0.1;
      elseif byte==string.byte(",") then  targetvel_new[1]=targetvel[1]-0.02;
      elseif byte==string.byte("h") then  targetvel_new[2]=targetvel[2]+0.02;
      elseif byte==string.byte(";") then  targetvel_new[2]=targetvel[2]-0.02;


      elseif byte==string.byte("3") then      
        mcm.set_walk_kickfoot(0)
        mcm.set_walk_kicktype(1)
        mcm.set_walk_steprequest(1)

      elseif byte==string.byte("4") then      
        mcm.set_walk_kickfoot(1)
        mcm.set_walk_kicktype(1)
        mcm.set_walk_steprequest(1)

      elseif byte==string.byte("5") then      
        mcm.set_walk_kickfoot(0)
        mcm.set_walk_kicktype(0)

        mcm.set_walk_steprequest(1)

      elseif byte==string.byte("6") then      
        mcm.set_walk_kickfoot(1)
        mcm.set_walk_kicktype(0)
        mcm.set_walk_steprequest(1)

      elseif byte==string.byte("7") then      
        motion_ch:send'sit'
      elseif byte==string.byte("8") then  
        motion_ch:send'stand'
        body_ch:send'stop'
        if mcm.get_walk_ismoving()>0 then 
          print("requesting stop")
          mcm.set_walk_stoprequest(1) 
        end
      elseif byte==string.byte("9") then  
        motion_ch:send'hybridwalk'
  --      body_ch:send'stepinplace'
      elseif byte==string.byte("f") then        
        head_ch:send'scan'      
      elseif byte==string.byte("0") then      
        gcm.set_game_state(0)
      end



      local vel_diff = (targetvel_new[1]-targetvel[1])^2+(targetvel_new[2]-targetvel[2])^2+(targetvel_new[3]-targetvel[3])^2
      if vel_diff>0 then
        targetvel[1],targetvel[2],targetvel[3] = targetvel_new[1],targetvel_new[2],targetvel_new[3]
        mcm.set_walk_vel(targetvel)
      end
    else  --Game state! 
      if byte==string.byte("0") then      
        gcm.set_game_state(6) --GO to test state     
      elseif byte==string.byte("1") then      
        gcm.set_game_state(0)      
      elseif byte==string.byte("2") then      
        gcm.set_game_state(1)
      elseif byte==string.byte("3") then      
        gcm.set_game_state(2)
      elseif byte==string.byte("4") then      
        gcm.set_game_state(3)      
      elseif byte==string.byte("5") then      
        gcm.set_game_state(4)            
      elseif byte==string.byte("r") then  
        gcm.set_game_role(1-gcm.get_game_role())
      end
    end
  end
end

local t_last = Body.get_time()
local tDelay = 0.005*1E6


local role_names={
  util.color('Goalie','green'),
  util.color('Attacker','red'),  
  'Test'}
local gcm_names={
  util.color('Initial','green'),
  util.color('Ready','green'),
  util.color('Set','green'),
  util.color('Playing','blue'),
  util.color('Finished','green'),
  util.color('Untorqued','red'),    
  util.color('Test','blue'),    
}
local command1='\nKey commands:\n'
  ..'1 : game Initial\n'
  ..'2 : game Ready\n'
  ..'3 : game Set\n'
  ..'4 : game Playing\n'
  ..'5 : game Finished\n'
  ..'r : Toggle role\n'
  ..'0 : Enter test mode\n'

local command2=
  util.color('Test mode\n','blue')
  ..'Key commands:\n'
..'i/j/k/l/,/h/; : control walk velocity\n'

..'3 : Left kick\n'
..'4 : Right kick\n'
..'5 : Left Walkkick\n'
..'6 : Right Walkkick\n'

..'8 : stop walking\n'
..'9 : start walking\n'
..'0 : Enter game mode\n'
  

function show_status()
  os.execute('clear')
  local outstring=''
  if gcm.get_game_state()==6 then --Testing state
    outstring= outstring..command2..string.format(
    "Target velocity: %.3f %.3f %.3f",unpack(targetvel)
    )

  else
    outstring = string.format(
    "Role: %s\nGame state: %s\nMotion state: %s\nBody state: %s\nHead state:%s \n",
    role_names[gcm.get_game_role()+1],    
    gcm_names[gcm.get_game_state()+1],
    gcm.get_fsm_Motion(),
    gcm.get_fsm_Body(),
    gcm.get_fsm_Head()
    )..command1
  end
 
  
  print(outstring)


end

 while 1 do
  --[[
  t=Body.get_time()
  tPassed = t-t_last
  t_last = t
  if tPassed>0.005 then
    process_keyinput();
  end
  unix.usleep(tDelay);
  --]]
  show_status()
  process_keyinput(); --why nonblocking reading does not work?
end
