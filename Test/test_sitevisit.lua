#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end

local targetvel = {0,0,0}
local targetvel_new = {0,0,0}
local WAS_REQUIRED

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
--[[
local command1='\nKey commands:\n'
  ..'1 : game Initial\n'
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
..'a : Enter attacker mode\n'
..'g : Enter goalie mode\n'
  --]]

  local command1,command2 = '',''

local function show_status()
  os.execute('clear')
  print("Game role:",gcm.get_game_role())
  print("Game state:",gcm.get_game_state())

  local outstring=''
--  if gcm.get_game_state()==6 then --Testing state
  if gcm.get_game_role()==2 then --Testing state
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

local function update(key_code)
  if type(key_code)~='number' or key_code==0 then return end
	local key_char = string.char(key_code)
	local key_char_lower = string.lower(key_char)
	
	


	if key_char_lower==("i") then      targetvel_new[1]=targetvel[1]+0.02;
	elseif key_char_lower==("j") then  targetvel_new[3]=targetvel[3]+0.1;
	elseif key_char_lower==("k") then  targetvel_new[1],targetvel_new[2],targetvel_new[3]=0,0,0;
	elseif key_char_lower==("l") then  targetvel_new[3]=targetvel[3]-0.1;
	elseif key_char_lower==(",") then  targetvel_new[1]=targetvel[1]-0.02;
	elseif key_char_lower==("h") then  targetvel_new[2]=targetvel[2]+0.02;
	elseif key_char_lower==(";") then  targetvel_new[2]=targetvel[2]-0.02;

	elseif key_char_lower==("1") then
		gcm.set_game_role(2)
		gcm.set_game_state(6)
		body_ch:send'init'
	elseif key_char_lower==("2") then      
		head_ch:send'scan'

	elseif key_char_lower==("3") then      
		lidar_ch:send'pansingle'

	elseif key_char_lower==("q") then      
		body_ch:send'approach'


	elseif key_char_lower==("x") then      
		motion_ch:send'getup'

	elseif key_char_lower==("c") then      
		body_ch:send'stepover'

	elseif key_char_lower==("v") then      
		body_ch:send'stepover1'		


	elseif key_char_lower==("e") then      
		body_ch:send'footrace'		


	elseif key_char_lower==("=") then      
		hcm.set_state_proceed(1)

--		elseif key_char_lower==("w") then      
--			mcm.set_walk_kicktype(1)
--			body_ch:send'approach'

--		elseif key_char_lower==("e") then      
--			mcm.set_walk_kicktype(2)
--			body_ch:send'approach'


	elseif key_char_lower==("s") then      
		arm_ch:send'awesome'


	elseif key_char_lower==("7") then      
		motion_ch:send'sit'
	elseif key_char_lower==("8") then  
		motion_ch:send'stand'
		body_ch:send'stop'
		if mcm.get_walk_ismoving()>0 then 
			print("requesting stop")
			mcm.set_walk_stoprequest(1) 
		end
	elseif key_char_lower==("9") then  
		motion_ch:send'hybridwalk'
--      body_ch:send'stepinplace'
	elseif key_char_lower==("a") then      
		gcm.set_game_role(1)
		gcm.set_game_state(0)
	elseif key_char_lower==("g") then      
		gcm.set_game_role(0)
		gcm.set_game_state(0)
	end
	local vel_diff = (targetvel_new[1]-targetvel[1])^2+(targetvel_new[2]-targetvel[2])^2+(targetvel_new[3]-targetvel[3])^2
	if vel_diff>0 then
		targetvel[1],targetvel[2],targetvel[3] = targetvel_new[1],targetvel_new[2],targetvel_new[3]
		mcm.set_walk_vel(targetvel)
	end	
end

show_status()
if ... and type(...)=='string' then
	WAS_REQUIRED = true
	return {entry=nil, update=update, exit=nil}
end

-- Field of View and timing for the mesh
vcm.set_mesh_sweep({40*DEG_TO_RAD, 1.0})

local getch = require'getch'
local running = true
local key_code
while running do
	key_code = getch.block()
  update(key_code)
end
