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

for sm,en in pairs(Config.fsm.enabled) do
  local fsm_name = sm..'FSM'
  table.insert(fsm_chs, fsm_name)
  _G[sm:lower()..'_ch'] = si.new_publisher(fsm_name.."!")
end



-- RPC engine
rpc_ch = si.new_requester(Config.net.reliable_rpc)


targetvel={0,0,0}
targetvel_new={0,0,0}
--Name show_factor value increment
local params={
  {'tStep',1,'0'},
  {'tZMP',1,'0'},
  {'stepHeight',1,'0'},
  {'torsoX',1,'0'},
  {'supportX',1,'0'},
  {'supportY',1,'0'},
  {'hipRollCompensation',180/math.pi,'0'},
  {'ankleRollCompensation',180/math.pi,'0'},
}


for i=1,#params do params[i][3]=Config.walk[params[i][1]] or 0 end
local selected_param = 1










function process_keyinput()
  local byte=getch.block();

  if byte then
    if byte==string.byte("i") then      targetvel_new[1]=targetvel[1]+0.02;
      elseif byte==string.byte("j") then  targetvel_new[3]=targetvel[3]+0.1;
      elseif byte==string.byte("k") then  targetvel_new[1],targetvel_new[2],targetvel_new[3]=0,0,0;
      elseif byte==string.byte("l") then  targetvel_new[3]=targetvel[3]-0.1;
      elseif byte==string.byte(",") then  targetvel_new[1]=targetvel[1]-0.02;
      elseif byte==string.byte("h") then  targetvel_new[2]=targetvel[2]+0.02;
      elseif byte==string.byte(";") then  targetvel_new[2]=targetvel[2]-0.02;
      
      --Cycle through selected params
      elseif byte==string.byte("w") then 
        selected_param = selected_param%#params + 1
      elseif byte==string.byte("q") then  
        selected_param = (selected_param + #params-2) %#params + 1






    elseif byte==string.byte("4") then      
      mcm.set_walk_kicktype(1) --this means testing mode (don't run body fsm)
      mcm.set_walk_kickfoot(1)
      body_ch:send'kick'

    elseif byte==string.byte("5") then      
      mcm.set_walk_kickfoot(0)
      mcm.set_walk_steprequest(1)

    elseif byte==string.byte("6") then      
      mcm.set_walk_kickfoot(1)
      mcm.set_walk_steprequest(1)

    elseif byte==string.byte("7") then      
      motion_ch:send'sit'

    elseif byte==string.byte("0") then      
      gcm.set_game_state(0)      
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
      mcm.set_walk_vel(targetvel)
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
local command1=''

local command2=''
 


function show_status()
  os.execute('clear')
  local outstring=''
  outstring= outstring..command1

--  outstring=outstring..string.format("Target velocity: %.3f %.3f %.3f\n",unpack(targetvel))

  for i=1,#params do

    local paramstr = string.format('%s : %.2f\n',
        params[i][1],params[i][2]*params[i][3])
    if i==selected_param then
      paramstr = util.color(paramstr,'green')
    end
    outstring=outstring..paramstr
  end

  print(outstring)
end

 while 1 do
  show_status()
  process_keyinput(); --why nonblocking reading does not work?
end
